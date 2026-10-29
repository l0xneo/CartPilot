using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace CartPilot;

internal struct Waypoint
{
    public Vector3 Pos;
    public bool Stairs; // segment starting here is part of a stair run (follow strictly, slow)
    public bool Choke;  // narrow spot or door; far side must be clear before entering
    public bool Door;
    public float Room;  // free width beside the cart here (both sides), precomputed so driving needs no NavMesh queries

    public Waypoint(Vector3 pos) { Pos = pos; Stairs = Choke = Door = false; Room = 2f; }
}

internal class PlanResult
{
    public List<Waypoint> Path;
    public bool Partial;
    public string Error;
    public Vector3? FailPoint; // where a rejected route failed (for blocking that area); null = no route at all
    public bool TooNarrow; // the blocked gap was measured and is narrower than the cart
}

/// <summary>
/// Base path from the cart NavMesh, then: push corners off walls, densify, rewrite stairs, validate every
/// segment with a cart-sized box cast (repair or reject), and flag chokepoints/doors.
/// </summary>
internal static class PathPlanner
{
    private const float DenseStep = 0.5f;
    private const float StepClearance = 0.3f;          // box casts start this far above the floor so steps don't count as walls
    private const float StairsStepClearance = 0.45f;

    internal static readonly int WorldMask = LayerMask.GetMask("Default");
    internal static readonly int DoorMask = LayerMask.GetMask("PhysGrabObjectHinge");

    private static readonly NavMeshPath NavPath = new();

    internal static PlanResult Plan(Vector3 from, Vector3 to, CartShape shape)
    {
        var res = new PlanResult();
        using var _ = CartNavMesh.Use();

        // 1. Base path: comfortable mesh, then tight, then a thin squeeze mesh (box casts below decide if the cart really fits).
        //    Roomier tiers are preferred unless a tighter one is much shorter (a roomy detour two rooms away isn't worth it);
        //    with no complete path, the partial one that ends closest to the goal.
        List<Vector3> corners = null;
        var filter = CartNavMesh.TightFilter;
        string tierName = null;
        float bestGap = float.MaxValue;
        var complete = new List<(List<Vector3> c, NavMeshQueryFilter f, string name, float len)>();
        (List<Vector3> c, NavMeshQueryFilter f, string name, float len) blockedFallback = default;
        foreach (var (f, tname) in new[] { (CartNavMesh.ComfortableFilter, "comfortable"), (CartNavMesh.TightFilter, "tight"), (CartNavMesh.SqueezeFilter, "squeeze"), (CartNavMesh.GameFilter, "game") })
        {
            if (!Sample(from, f, out var a) || !Sample(to, f, out var b)) continue;
            if (!NavMesh.CalculatePath(a, b, f, NavPath) || NavPath.status == NavMeshPathStatus.PathInvalid || NavPath.corners.Length < 2) continue;
            // The cart's spot can snap onto a small cut-off patch of mesh; try starting from the mesh around it instead.
            if (NavPath.status == NavMeshPathStatus.PathPartial) TryStartsAround(from, b, f);

            if (NavPath.status == NavMeshPathStatus.PathComplete)
            {
                var c = new List<Vector3>(NavPath.corners);
                // A game-mesh route straight back through a blocked cart or enemy just drives into it again; use it only if nothing else works.
                if (tname == "game" && CartNavMesh.CrossesBlocked(c)) { Verbose("Game mesh route goes through a blocked area, kept as last resort."); blockedFallback = (c, f, tname, Length(c)); continue; }
                complete.Add((c, f, tname, Length(c)));
                continue;
            }
            float gap = (to - NavPath.corners[NavPath.corners.Length - 1]).magnitude; // 3D: ending on the floor above is not "close"
            if (complete.Count > 0 || gap >= bestGap) continue;

            bestGap = gap;
            corners = new List<Vector3>(NavPath.corners);
            filter = f;
            tierName = tname;
            res.Partial = true;
        }
        if (complete.Count == 0 && blockedFallback.c != null) complete.Add(blockedFallback);
        if (complete.Count > 0)
        {
            float shortest = float.MaxValue;
            foreach (var t in complete) shortest = Mathf.Min(shortest, t.len);
            var pick = complete.Find(t => t.len <= shortest * 1.25f + 4f);
            corners = pick.c;
            filter = pick.f;
            tierName = pick.name;
            res.Partial = false;
            bestGap = 0f;
        }
        if (corners == null)
        {
            res.Error = "start or goal is too far from any walkable area";
            return res;
        }
        Verbose($"Base path on {tierName} mesh: {corners.Count} corners{(res.Partial ? $" (partial, {bestGap:F1}m short)" : "")}");


        // Truck interior / inside extractions aren't on the cart mesh: finish with a straight off-mesh leg if the cart physically fits.
        if (res.Partial) TryOffMeshFinish(corners, to, shape, res);

        Verbose($"Corners: {string.Join(" ", corners)}");

        // 2. Corner clearance, 3. densify, 4. stairs, 5. validate, 6. chokes.
        PushCornersOffWalls(corners, filter);
        var path = Densify(corners, filter);
        StairsRewriter.Rewrite(path, shape, filter);
        if (!ValidateAndRepair(path, shape, filter, out var failAt, out bool tooNarrow))
        {
            res.Error = $"segment blocked for a cart this size near {failAt}";
            res.FailPoint = failAt;
            res.TooNarrow = tooNarrow;
            return res;
        }
        MarkChokes(path, shape, filter);

        // Cart parked off the mesh (e.g. inside the truck): start with a straight leg from where it actually is.
        // Added after validation on purpose: the cart already physically occupies that spot.
        if (Flat(path[0].Pos - from).magnitude > 0.3f) path.Insert(0, new Waypoint(from));

        res.Path = path;
        return res;
    }

    // ---------- helpers ----------

    internal static float Length(List<Vector3> pts)
    {
        float len = 0f;
        for (int i = 1; i < pts.Count; i++) len += (pts[i] - pts[i - 1]).magnitude;
        return len;
    }

    private static readonly NavMeshPath AltPath = new();

    /// <summary>Leaves NavPath as the first complete path found from a start around the cart (unchanged if none).</summary>
    private static void TryStartsAround(Vector3 from, Vector3 goal, NavMeshQueryFilter f)
    {
        foreach (float r in new[] { 1.5f, 3f })
        {
            for (int i = 0; i < 8; i++)
            {
                var p = from + Quaternion.Euler(0f, i * 45f, 0f) * Vector3.forward * r;
                if (!NavMesh.SamplePosition(p, out var hit, 1f, f)) continue;
                // Same floor and no wall in between: a start on the floor below sent the cart into a wall.
                if (Mathf.Abs(hit.position.y - from.y) > 0.4f) continue;
                if (Physics.Linecast(from + Vector3.up * 0.5f, hit.position + Vector3.up * 0.5f, WorldMask, QueryTriggerInteraction.Ignore)) continue;
                if (!NavMesh.CalculatePath(hit.position, goal, f, AltPath) || AltPath.status != NavMeshPathStatus.PathComplete) continue;
                NavMesh.CalculatePath(hit.position, goal, f, NavPath);
                Verbose($"Start moved to {hit.position} (cart was on a cut-off patch of mesh)");
                return;
            }
        }
    }

    private const float MaxOffMeshLeg = 10f;

    private static void TryOffMeshFinish(List<Vector3> corners, Vector3 goal, CartShape shape, PlanResult res)
    {
        // Real floor under the goal (players/points sit a bit above it).
        if (Physics.Raycast(goal + Vector3.up * 0.5f, Vector3.down, out var floor, 3f, WorldMask, QueryTriggerInteraction.Ignore))
            goal = floor.point;

        var end = corners[corners.Count - 1];
        float gap = Flat(goal - end).magnitude;
        if (gap < 0.3f || gap > MaxOffMeshLeg) return;

        if (!SegmentClear(end, goal, shape))
        {
            Verbose($"Off-mesh finish {end} -> {goal} blocked for the cart's size.");
            return;
        }
        corners.Add(goal);
        res.Partial = false;
        Verbose($"Off-mesh finish: +{gap:F1}m straight to {goal}");
    }

    /// <summary>Floor spot under a player standing up on something (a crate, a ledge they jumped onto).</summary>
    internal static bool FloorBelow(Vector3 p, out Vector3 floor)
    {
        using var _ = CartNavMesh.Use();
        foreach (float drop in new[] { 1.5f, 2.5f, 4f, 6f })
        {
            if (NavMesh.SamplePosition(p + Vector3.down * drop, out var hit, 3f, CartNavMesh.GameFilter) && hit.position.y < p.y - 1f)
            {
                floor = hit.position;
                return true;
            }
        }
        floor = p;
        return false;
    }

    internal static bool Sample(Vector3 p, NavMeshQueryFilter f, out Vector3 onMesh)
    {
        for (float r = 0.5f; r <= 8f; r *= 2f)
        {
            // Same floor only: a player on a grate/upper floor must not snap onto the floor below (or vice versa).
            if (NavMesh.SamplePosition(p, out var hit, r, f) && Mathf.Abs(hit.position.y - p.y) < 1.5f) { onMesh = hit.position; return true; }
        }
        onMesh = p;
        return false;
    }

    /// <summary>NavMesh corners hug the mesh edge; nudge interior corners inward for extra margin where there's room.</summary>
    private static void PushCornersOffWalls(List<Vector3> corners, NavMeshQueryFilter f)
    {
        float want = Cfg.PreferredExtraClearance.Value;
        for (int i = 1; i < corners.Count - 1; i++)
        {
            var p = corners[i];
            for (int iter = 0; iter < 3; iter++)
            {
                if (!NavMesh.FindClosestEdge(p, out var edge, f) || edge.distance >= want) break;
                var candidate = p + edge.normal * (want - edge.distance);
                if (!NavMesh.SamplePosition(candidate, out var s, 0.3f, f)) break;
                // Don't move into a spot that is tighter, or that the neighbours can't see straight.
                if (!NavMesh.FindClosestEdge(s.position, out var e2, f) || e2.distance <= edge.distance) break;
                if (NavMesh.Raycast(corners[i - 1], s.position, out _, f) || NavMesh.Raycast(s.position, corners[i + 1], out _, f)) break;
                p = s.position;
            }
            corners[i] = p;
        }
    }

    private static List<Waypoint> Densify(List<Vector3> corners, NavMeshQueryFilter f)
    {
        var path = new List<Waypoint> { new(corners[0]) };
        for (int i = 1; i < corners.Count; i++)
        {
            Vector3 a = corners[i - 1], b = corners[i];
            int steps = Mathf.Max(1, Mathf.CeilToInt(Flat(b - a).magnitude / DenseStep));
            for (int s = 1; s <= steps; s++)
            {
                var p = Vector3.Lerp(a, b, s / (float)steps);
                // Real floor height matters for stair detection. Only take the height, and only when the point is
                // actually on the mesh, so off-mesh legs aren't pulled back to the mesh edge.
                // Sample from the previous point's height so it stays on the same surface: from the straight corner-to-corner
                // height it could snap to the floor under the stairs, and that dip made Museum stair routes fail every time.
                var probe = new Vector3(p.x, path[path.Count - 1].Pos.y, p.z);
                if (s < steps && NavMesh.SamplePosition(probe, out var hit, 1f, f) && Flat(hit.position - p).magnitude < 0.2f) p.y = hit.position.y;
                path.Add(new Waypoint(p));
            }
        }
        return path;
    }

    /// <summary>Cart-sized box cast along every segment against world geometry. Tries lateral nudges before giving up.</summary>
    private static bool ValidateAndRepair(List<Waypoint> path, CartShape shape, NavMeshQueryFilter f, out Vector3 failAt, out bool tooNarrow)
    {
        failAt = default;
        tooNarrow = false;
        for (int i = 0; i < path.Count - 1; i++)
        {
            // Steep short stairs: step noses poke above the base->top line, so check higher there.
            // Also the step on/off the stairs, and a slimmer box there: the stair sides grazed the 90% box at exactly
            // half-width on Wizard stairs the cart climbs fine, so every route across them was rejected (V never reached extraction).
            bool stairs = path[i].Stairs || path[i + 1].Stairs;
            float clear = stairs ? StairsStepClearance : StepClearance;
            float widthScale = stairs ? 0.8f : 0.9f;
            if (SegmentClear(path[i].Pos, path[i + 1].Pos, shape, clear, widthScale)) continue;
            // The cart already physically sits at the start; a short first hop from there isn't a reason to reject.
            if (i == 0 && Flat(path[1].Pos - path[0].Pos).magnitude < 1.5f) continue;

            if (TryNudge(path, i + 1, shape, f, clear, widthScale) || TryNudge(path, i, shape, f, clear, widthScale)) continue;

            SegmentHit(path[i].Pos, path[i + 1].Pos, shape, out var hit, clear, widthScale);
            if (TryStraighten(path, i, hit.point, shape, f, clear, widthScale, out float gap)) continue;
            tooNarrow = gap > 0f && gap < 2f * shape.HalfWidth * widthScale;

            failAt = (path[i].Pos + path[i + 1].Pos) * 0.5f;
            Plugin.Log.LogWarning($"Route rejected: segment {i} ({path[i].Pos} -> {path[i + 1].Pos}) hits '{(hit.collider ? hit.collider.name : "?")}' at {hit.point}" +
                (gap > 0f ? $", gap {gap:F2}m for a {2f * shape.HalfWidth:F2}m cart." : "."));
            return false;
        }
        return true;
    }

    private static readonly float[] NudgeOffsets = { 0.2f, -0.2f, 0.4f, -0.4f, 0.6f, -0.6f };

    private static bool TryNudge(List<Waypoint> path, int idx, CartShape shape, NavMeshQueryFilter f, float clear, float widthScale)
    {
        if (idx <= 0 || idx >= path.Count - 1) return false; // never move the start or the goal
        var prev = path[idx - 1].Pos;
        var next = path[idx + 1].Pos;
        var dir = Flat(next - prev).normalized;
        var side = Vector3.Cross(Vector3.up, dir);

        foreach (float o in NudgeOffsets)
        {
            if (!NavMesh.SamplePosition(path[idx].Pos + side * o, out var hit, 0.3f, f)) continue;
            var c = hit.position;
            if (NavMesh.Raycast(prev, c, out _, f) || NavMesh.Raycast(c, next, out _, f)) continue;
            // Same clearance as the check that failed: on stairs the flat-floor box always hits the steps, so nudges there never passed.
            if (!SegmentClear(prev, c, shape, clear, widthScale) || !SegmentClear(c, next, shape, clear, widthScale)) continue;

            var wp = path[idx];
            wp.Pos = c;
            path[idx] = wp;
            Verbose($"Nudged waypoint {idx} by {o:F1}m for clearance");
            return true;
        }
        return false;
    }

    /// <summary>
    /// A doorway crossed at an angle needs more room than the cart's width (a door with stairs right behind it gets a
    /// diagonal path). Measure the gap at the hit and add waypoints that line the cart up square to it.
    /// </summary>
    private static bool TryStraighten(List<Waypoint> path, int i, Vector3 hitPoint, CartShape shape, NavMeshQueryFilter f, float clear, float widthScale, out float gap)
    {
        gap = 0f;
        var a = path[i].Pos;
        var b = path[i + 1].Pos;
        var ab = b - a;
        if (ab.sqrMagnitude < 0.01f || hitPoint == Vector3.zero) return false;

        // Find the two sides of the gap: closest wall hit, then the closest one roughly opposite it.
        var p = a + ab * Mathf.Clamp01(Vector3.Dot(hitPoint - a, ab) / ab.sqrMagnitude);
        var probe = new Vector3(p.x, Mathf.Clamp(hitPoint.y, p.y + 0.3f, p.y + 1.2f), p.z);
        const int Rays = 36;
        var hits = new Vector3?[Rays];
        int nearest = -1;
        float nearestDist = float.MaxValue;
        for (int k = 0; k < Rays; k++)
        {
            float ang = k * Mathf.PI * 2f / Rays;
            if (!Physics.Raycast(probe, new Vector3(Mathf.Cos(ang), 0f, Mathf.Sin(ang)), out var h, 2f, WorldMask, QueryTriggerInteraction.Ignore)) continue;
            hits[k] = h.point;
            if (h.distance < nearestDist) { nearestDist = h.distance; nearest = k; }
        }
        if (nearest < 0) return false;
        Vector3? other = null;
        float otherDist = float.MaxValue;
        for (int k = 0; k < Rays; k++)
        {
            int diff = Mathf.Abs(k - nearest) % Rays;
            if (hits[k] == null || Mathf.Min(diff, Rays - diff) < Rays * 3 / 8) continue; // at least 135 degrees apart
            float dist = Flat(hits[k].Value - probe).magnitude;
            if (dist < otherDist) { otherDist = dist; other = hits[k]; }
        }
        if (other == null) return false;

        var sideA = hits[nearest].Value;
        gap = Flat(other.Value - sideA).magnitude;
        if (gap < 2f * shape.HalfWidth * widthScale) return false;

        var across = Flat(other.Value - sideA).normalized;
        var through = Vector3.Cross(Vector3.up, across);
        if (Vector3.Dot(through, ab) < 0f) through = -through;
        var center = (sideA + other.Value) * 0.5f;
        center.y = p.y;

        foreach (float lead in new[] { shape.HalfLength + 0.4f, shape.HalfLength + 0.9f })
        {
            bool okPre = SampleNear(center - through * lead, f, out var pre) && Vector3.Dot(pre - a, through) > 0.1f;
            bool okPost = SampleNear(center + through * lead, f, out var post) && Vector3.Dot(b - post, through) > 0.1f;
            foreach (var chain in new[] { okPre && okPost ? new[] { pre, post } : null, okPre ? new[] { pre } : null, okPost ? new[] { post } : null })
            {
                if (chain == null) continue;
                bool clearAll = true;
                var from = a;
                foreach (var c in chain) { clearAll &= SegmentClear(from, c, shape, clear, widthScale); from = c; }
                if (!clearAll || !SegmentClear(from, b, shape, clear, widthScale)) continue;

                for (int k = 0; k < chain.Length; k++)
                    path.Insert(i + 1 + k, new Waypoint(chain[k]) { Stairs = path[i].Stairs });
                Verbose($"Lined up square to a {gap:F2}m gap with {chain.Length} waypoint(s)");
                return true;
            }
        }
        return false;
    }

    private static bool SampleNear(Vector3 p, NavMeshQueryFilter f, out Vector3 onMesh)
    {
        bool ok = NavMesh.SamplePosition(p, out var hit, 0.5f, f) && Flat(hit.position - p).magnitude < 0.3f;
        onMesh = ok ? hit.position : p;
        return ok;
    }

    internal static bool SegmentClear(Vector3 a, Vector3 b, CartShape shape, float stepClearance = StepClearance, float widthScale = 0.9f) => !SegmentHit(a, b, shape, out _, stepClearance, widthScale);

    internal static bool SegmentHit(Vector3 a, Vector3 b, CartShape shape, out RaycastHit hit, float stepClearance = StepClearance, float widthScale = 0.9f)
    {
        hit = default;
        var d = b - a;
        float len = d.magnitude;
        if (len < 0.01f) return false;
        float halfH = Mathf.Max(0.1f, (shape.Height - stepClearance) * 0.5f);
        // 90% width: the measured box is the cart's widest point at every height; the real body is rounder.
        var half = new Vector3(shape.HalfWidth * widthScale, halfH, 0.05f);
        var lift = Vector3.up * (stepClearance + halfH);
        // Box pitches with the segment, so it follows stair slopes instead of hitting the steps.
        return Physics.BoxCast(a + lift, half, d / len, out hit, Quaternion.LookRotation(d), len, WorldMask, QueryTriggerInteraction.Ignore);
    }

    private static void MarkChokes(List<Waypoint> path, CartShape shape, NavMeshQueryFilter f)
    {
        // Mesh is shrunk by its radius; add that back and subtract the cart's width = real free room beside the cart.
        float pad = 2f * (CartNavMesh.RadiusOf(f) - shape.HalfWidth);
        for (int i = 0; i < path.Count; i++)
        {
            var wp = path[i];
            var dir = Flat(path[Mathf.Min(i + 1, path.Count - 1)].Pos - path[Mathf.Max(i - 1, 0)].Pos).normalized;
            wp.Door = Physics.CheckSphere(wp.Pos + Vector3.up * 0.8f, shape.HalfWidth + 0.4f, DoorMask, QueryTriggerInteraction.Ignore);
            wp.Room = dir != Vector3.zero ? Mathf.Max(0f, SideRoom(wp.Pos, Vector3.Cross(Vector3.up, dir), f) + pad) : 2f;
            wp.Choke = wp.Door || wp.Room < ChokeWidth;
            path[i] = wp;
        }
    }

    private const float ChokeWidth = 0.5f;

    private static float SideRoom(Vector3 p, Vector3 side, NavMeshQueryFilter f)
    {
        float Room(Vector3 dir) => NavMesh.Raycast(p, p + dir * 2f, out var hit, f) ? hit.distance : 2f;
        return Room(side) + Room(-side);
    }

    internal static Vector3 Flat(Vector3 v) => new(v.x, 0f, v.z);

    internal static void Verbose(string msg)
    {
        if (Cfg.VerboseLog.Value) Plugin.Log.LogInfo(msg);
        else DiagLog.Write("verbose", msg); // always kept in the diagnostic file; the listener catches it when VerboseLog is on
    }
}
