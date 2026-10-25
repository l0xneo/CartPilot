using System.Collections.Generic;
using UnityEngine;

namespace CartPilot;

/// <summary>Host-only path visualisation: a line for the path, small spheres at stair and chokepoint waypoints, one for the lookahead.</summary>
internal class PathDebugDraw
{
    private static Material mat;
    private readonly GameObject root;
    private readonly LineRenderer line;
    private readonly List<GameObject> markers = new();
    private readonly GameObject lookahead;

    internal PathDebugDraw()
    {
        // Internal-Colored ships with every Unity build; Sprites/Default may be stripped.
        if (mat == null) mat = new Material(Shader.Find("Hidden/Internal-Colored") ?? Shader.Find("Sprites/Default"));
        root = new GameObject("CartPilot Debug");
        line = root.AddComponent<LineRenderer>();
        line.material = mat;
        line.widthMultiplier = 0.06f;
        line.startColor = line.endColor = Color.green;
        line.positionCount = 0;
        lookahead = Marker(Color.cyan, 0.25f);
    }

    internal void SetPath(List<Waypoint> path)
    {
        foreach (var m in markers) Object.Destroy(m);
        markers.Clear();
        if (!Cfg.DebugDraw.Value || path == null) { Clear(); return; }

        line.positionCount = path.Count;
        for (int i = 0; i < path.Count; i++)
        {
            line.SetPosition(i, path[i].Pos + Vector3.up * 0.15f);
            if (path[i].Stairs || path[i].Choke)
            {
                var m = Marker(path[i].Stairs ? Color.yellow : Color.red, 0.15f);
                m.transform.position = path[i].Pos + Vector3.up * 0.15f;
                markers.Add(m);
            }
        }
    }

    internal void SetLookahead(Vector3 p)
    {
        lookahead.SetActive(Cfg.DebugDraw.Value);
        lookahead.transform.position = p + Vector3.up * 0.15f;
    }

    internal void Clear()
    {
        line.positionCount = 0;
        lookahead.SetActive(false);
        foreach (var m in markers) Object.Destroy(m);
        markers.Clear();
    }

    internal void Destroy() => Object.Destroy(root);

    private GameObject Marker(Color c, float size)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Object.Destroy(go.GetComponent<Collider>());
        go.transform.SetParent(root.transform);
        go.transform.localScale = Vector3.one * size;
        var r = go.GetComponent<Renderer>();
        r.material = mat;
        r.material.color = c;
        return go;
    }
}
