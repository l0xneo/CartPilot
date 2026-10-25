using System;
using System.Collections.Generic;
using HarmonyLib;
using Unity.AI.Navigation;
using UnityEngine;
using UnityEngine.AI;

namespace CartPilot;

/// <summary>
/// Cart-sized NavMeshes, baked (async) from the same sources as the game's enemy NavMesh but with the
/// cart's radius, so corridors the cart can't fit through don't exist for the planner.
///
/// Two layers: "comfortable" (radius + preferred extra clearance) is tried first, "tight" is the fallback.
/// Data is only registered with the NavMesh system inside <see cref="Use"/>, so the game's own unfiltered
/// NavMesh queries (enemies call SamplePosition/CalculatePath with areaMask -1) never land on our meshes.
/// </summary>
internal static class CartNavMesh
{
    private class Layer
    {
        public string Name;
        public int AgentTypeId = -1;
        public float Radius;
        public NavMeshData Data;
        public NavMeshDataInstance Instance;
        public AsyncOperation Op;
    }

    private struct BlockedArea
    {
        public Vector3 Pos;
        public float Radius;
        public float Expires;
    }

    private static readonly Layer Comfortable = new() { Name = "comfortable" };
    private static readonly Layer Tight = new() { Name = "tight" };
    private static readonly Layer Squeeze = new() { Name = "squeeze" };
    private static readonly Layer[] Layers = { Comfortable, Tight, Squeeze };
    private static readonly List<BlockedArea> Blocked = new();

    private static NavMeshData bakedFromGameData; // game's data our bake is based on; changes every level
    private static float bakedRadius = -1f;
    private static float bakedExtra = -1f;
    private static int useDepth;

    private static readonly Func<NavMeshSurface, List<NavMeshBuildSource>> CollectSources =
        AccessTools.MethodDelegate<Func<NavMeshSurface, List<NavMeshBuildSource>>>(AccessTools.Method(typeof(NavMeshSurface), "CollectSources"));
    private static readonly Func<NavMeshSurface, List<NavMeshBuildSource>, Bounds> CalculateWorldBounds =
        AccessTools.MethodDelegate<Func<NavMeshSurface, List<NavMeshBuildSource>, Bounds>>(AccessTools.Method(typeof(NavMeshSurface), "CalculateWorldBounds"));

    internal static bool Busy => System.Array.Exists(Layers, l => IsRunning(l.Op));
    internal static bool Ready => !Busy && System.Array.TrueForAll(Layers, l => l.Data != null);

    internal static NavMeshQueryFilter ComfortableFilter => Filter(Comfortable);
    internal static NavMeshQueryFilter TightFilter => Filter(Tight);
    // Thin mesh that still connects narrow spots; box casts decide if the cart fits. Ours (unlike the game mesh) so blocked areas apply.
    internal static NavMeshQueryFilter SqueezeFilter => Filter(Squeeze);
    // Last resort: the game's own mesh (always registered, has every stair enemies can use). Blocked areas don't apply to it.
    internal static NavMeshQueryFilter GameFilter
    {
        get
        {
            var s = GameSurface();
            return new() { agentTypeID = s != null ? s.agentTypeID : 0, areaMask = NavMesh.AllAreas };
        }
    }

    private static NavMeshQueryFilter Filter(Layer l) => new() { agentTypeID = l.AgentTypeId, areaMask = NavMesh.AllAreas };
    private static bool IsRunning(AsyncOperation op) => op != null && !op.isDone;

    /// <summary>Starts a bake if the level or radius changed. Returns true when meshes are ready to query.</summary>
    internal static bool EnsureBaked(float radius, float maxGap)
    {
        var surface = GameSurface();
        if (surface == null || surface.navMeshData == null) return false;

        float extra = Cfg.PreferredExtraClearance.Value;
        bool stale = surface.navMeshData != bakedFromGameData || !Mathf.Approximately(radius, bakedRadius) || !Mathf.Approximately(extra, bakedExtra);
        if (stale && !Busy)
        {
            if (surface.navMeshData != bakedFromGameData) Blocked.Clear();
            bakedFromGameData = surface.navMeshData;
            bakedRadius = radius;
            bakedExtra = extra;
            Comfortable.Radius = radius + extra;
            Tight.Radius = radius;
            Squeeze.Radius = Mathf.Max(0.25f, radius * 0.5f);
            foreach (var l in Layers) l.Data = null;
            baseSources = CollectSources(surface);
            baseBounds = surface.collectObjects == CollectObjects.Volume
                ? new Bounds(surface.center, new Vector3(Mathf.Abs(surface.size.x), Mathf.Abs(surface.size.y), Mathf.Abs(surface.size.z)))
                : CalculateWorldBounds(surface, baseSources);
            Rebuild(surface);
            Plugin.Log.LogInfo($"Baking cart NavMesh (tight r={Tight.Radius:F2}, comfortable r={Comfortable.Radius:F2})...");
            return false;
        }
        if (Ready && useDepth == 0 && (gapsFromGameData != bakedFromGameData || !Mathf.Approximately(maxGap, gapsMax)))
        {
            gapsFromGameData = bakedFromGameData;
            gapsMax = maxGap;
            FindGaps(maxGap);
        }
        return Ready;
    }

    // ---------- Gaps ----------
    // Small holes in the floor have no mesh across, but a cart longer than twice the hole rolls straight over.
    // Found once per level on the game mesh, then added as links on our layers inside Use().

    private struct Gap { public Vector3 A, B, Out; } // floor edge before the hole, floor edge after it, direction across

    private static readonly List<Gap> Gaps = new();
    private static NavMeshData gapsFromGameData;
    private static float gapsMax = -1f;
    private static readonly List<NavMeshLinkData> LinkData = new();
    private static readonly List<NavMeshLinkInstance> LinkInstances = new();
    private static readonly NavMeshPath LinkCheck = new();
    private static bool linksResolved;

    private static Vector3Int Key(Vector3 p) => Vector3Int.RoundToInt(p * 50f);

    private static bool FloorAt(Vector3 p, float y, out float hitY)
    {
        hitY = 0f;
        if (!Physics.Raycast(new Vector3(p.x, y + 0.5f, p.z), Vector3.down, out var h, 0.8f, PathPlanner.WorldMask, QueryTriggerInteraction.Ignore)) return false;
        hitY = h.point.y;
        return true;
    }

    private static void FindGaps(float maxGap)
    {
        Gaps.Clear();
        LinkData.Clear();
        linksResolved = false;
        if (maxGap < 0.2f) return;
        var tri = NavMesh.CalculateTriangulation(); // only the game mesh is registered outside Use()
        var v = tri.vertices;
        var idx = tri.indices;

        // Boundary edges = edges used by one triangle. Weld by position: tiles duplicate vertices.
        var edges = new Dictionary<(Vector3Int, Vector3Int), (int a, int b, int c, int n)>();
        for (int t = 0; t + 2 < idx.Length; t += 3)
            for (int e = 0; e < 3; e++)
            {
                int a = idx[t + e], b = idx[t + (e + 1) % 3], c = idx[t + (e + 2) % 3];
                Vector3Int ka = Key(v[a]), kb = Key(v[b]);
                bool swap = ka.x > kb.x || (ka.x == kb.x && (ka.y > kb.y || (ka.y == kb.y && ka.z > kb.z)));
                var key = swap ? (kb, ka) : (ka, kb);
                edges[key] = edges.TryGetValue(key, out var ex) ? (ex.a, ex.b, ex.c, ex.n + 1) : (a, b, c, 1);
            }

        foreach (var (a, b, c, n) in edges.Values)
        {
            if (n != 1) continue;
            Vector3 p0 = v[a], p1 = v[b];
            var along = PathPlanner.Flat(p1 - p0);
            float len = along.magnitude;
            if (len < 0.05f) continue;
            along /= len;
            var outDir = Vector3.Cross(Vector3.up, along);
            if (Vector3.Dot(outDir, PathPlanner.Flat(v[c] - p0)) > 0f) outDir = -outDir; // away from the triangle

            for (float s = Mathf.Min(0.5f, len * 0.5f); s < len; s += 1f)
            {
                var m = p0 + along * s;
                if (FloorAt(m + outDir * 0.7f, m.y, out _)) continue; // wall or seam, not a drop (mesh radius is under 0.7)
                // Walk out: real floor edge, then where floor resumes at the same height.
                float t1 = -1f, t2 = -1f;
                for (float t = 0f; t <= 0.7f + maxGap + 0.2f; t += 0.1f)
                {
                    bool f = FloorAt(m + outDir * t, m.y, out float fy) && Mathf.Abs(fy - m.y) < 0.2f;
                    if (t1 < 0f) { if (!f) t1 = t; continue; }
                    if (f) { t2 = t; break; }
                }
                if (t1 < 0f || t2 < 0f || t2 - t1 > maxGap) continue;
                Vector3 A = m + outDir * Mathf.Max(0f, t1 - 0.1f), B = m + outDir * t2;
                if (Physics.Linecast(A + Vector3.up * 0.4f, B + outDir * 0.3f + Vector3.up * 0.4f, PathPlanner.WorldMask, QueryTriggerInteraction.Ignore)) continue; // railing/wall
                var mid = (A + B) * 0.5f;
                if (Gaps.Exists(g => ((g.A + g.B) * 0.5f - mid).sqrMagnitude < 0.8f * 0.8f)) continue;
                Gaps.Add(new Gap { A = A, B = B, Out = outDir });
            }
        }
        Plugin.Log.LogInfo($"Small floor gaps the cart can roll over: {Gaps.Count}");
        foreach (var g in Gaps) PathPlanner.Verbose($"Gap {g.A} -> {g.B} ({PathPlanner.Flat(g.B - g.A).magnitude:F2}m)");
    }

    // Needs our meshes registered to snap the link ends onto them.
    private static void ResolveLinks()
    {
        linksResolved = true;
        LinkData.Clear();
        foreach (var l in Layers)
        {
            var f = Filter(l);
            foreach (var g in Gaps)
            {
                if (!NavMesh.SamplePosition(g.A - g.Out * (l.Radius + 0.2f), out var ha, 0.5f, f)) continue;
                if (!NavMesh.SamplePosition(g.B + g.Out * (l.Radius + 0.2f), out var hb, 0.5f, f)) continue;
                if (NavMesh.CalculatePath(ha.position, hb.position, f, LinkCheck) && LinkCheck.status == NavMeshPathStatus.PathComplete
                    && PathPlanner.Length(new List<Vector3>(LinkCheck.corners)) < 6f) continue; // already connected nearby
                LinkData.Add(new NavMeshLinkData
                {
                    startPosition = ha.position, endPosition = hb.position, width = 0f,
                    bidirectional = true, costModifier = -1f, area = 0, agentTypeID = l.AgentTypeId,
                });
            }
        }
        Plugin.Log.LogInfo($"Gap links on cart meshes: {LinkData.Count}");
    }

    /// <summary>Marks an area unwalkable for the planner for a while and rebuilds the affected tiles.</summary>
    internal static void AddBlockedArea(Vector3 pos, float radius)
    {
        Blocked.Add(new BlockedArea { Pos = pos, Radius = radius, Expires = Time.time + Cfg.BlockedAreaLifetime.Value });
        Plugin.Log.LogInfo($"Blocked area at {pos} r={radius:F1} for {Cfg.BlockedAreaLifetime.Value:F0}s");
        pendingRebuild = true;
        Tick();
    }

    /// <summary>Drops expired blocked areas and applies pending changes once no bake is running. Call regularly.</summary>
    internal static void Tick()
    {
        if (Blocked.RemoveAll(b => b.Expires < Time.time) > 0) pendingRebuild = true;
        if (!pendingRebuild || !Ready) return;
        var surface = GameSurface();
        if (surface == null) return;
        pendingRebuild = false;
        Rebuild(surface);
    }

    private static bool pendingRebuild;

    // Level geometry is collected once per bake; collecting is the expensive part, blocked areas only append boxes.
    private static List<NavMeshBuildSource> baseSources;
    private static Bounds baseBounds;

    internal static float TightRadius => Tight.Radius;
    internal static float RadiusOf(NavMeshQueryFilter f) =>
        f.agentTypeID == Comfortable.AgentTypeId ? Comfortable.Radius : f.agentTypeID == Squeeze.AgentTypeId ? Squeeze.Radius
        : f.agentTypeID == Tight.AgentTypeId ? Tight.Radius : NavMesh.GetSettingsByID(f.agentTypeID).agentRadius;

    private static void Rebuild(NavMeshSurface surface)
    {
        var sources = new List<NavMeshBuildSource>(baseSources);
        foreach (var b in Blocked)
        {
            sources.Add(new NavMeshBuildSource
            {
                shape = NavMeshBuildSourceShape.ModifierBox,
                transform = Matrix4x4.Translate(b.Pos),
                size = new Vector3(b.Radius * 2f, 4f, b.Radius * 2f),
                area = 1, // built-in "Not Walkable"
            });
        }

        var bounds = baseBounds;

        foreach (var layer in Layers)
        {
            if (layer.AgentTypeId < 0) layer.AgentTypeId = NavMesh.CreateSettings().agentTypeID;

            var settings = surface.GetBuildSettings();
            settings.agentTypeID = layer.AgentTypeId;
            settings.agentRadius = layer.Radius;

            // UpdateNavMeshDataAsync only rebuilds tiles whose sources changed, so blocked-area updates are cheap.
            if (layer.Data == null)
                layer.Data = new NavMeshData(layer.AgentTypeId) { position = surface.transform.position, rotation = surface.transform.rotation };
            layer.Op = NavMeshBuilder.UpdateNavMeshDataAsync(layer.Data, settings, sources, bounds);
        }
    }

    private static NavMeshSurface GameSurface()
    {
        var lg = LevelGenerator.Instance;
        return lg != null && lg.Generated ? lg.GetComponent<NavMeshSurface>() : null;
    }

    /// <summary>Registers our meshes for the duration of a synchronous query block.</summary>
    internal static Scope Use() => new(true);

    internal readonly struct Scope : IDisposable
    {
        private readonly bool active;

        internal Scope(bool _)
        {
            active = Ready;
            if (!active || useDepth++ > 0) return;
            foreach (var l in Layers) l.Instance = NavMesh.AddNavMeshData(l.Data);
            if (!linksResolved) ResolveLinks();
            foreach (var d in LinkData) LinkInstances.Add(NavMesh.AddLink(d));
        }

        public void Dispose()
        {
            if (!active || --useDepth > 0) return;
            foreach (var li in LinkInstances) li.Remove();
            LinkInstances.Clear();
            foreach (var l in Layers) l.Instance.Remove();
        }
    }
}
