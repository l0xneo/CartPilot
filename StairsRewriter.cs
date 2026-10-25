using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace CartPilot;

/// <summary>
/// Finds stair runs on a densified path (by slope) and replaces them with a straight, centered approach:
/// entry (aligned before the base) -> base -> exit (past the top), all on the stair centerline.
/// The raw NavMesh path cuts diagonally across stairs, which is what made the old mod fall off.
/// </summary>
internal static class StairsRewriter
{
    private const float SlopeDeg = 15f;       // steeper than this over ~1 m counts as stairs
    private const float MinRise = 0.4f;       // ignore single steps / small ramps
    private const int SlopeWindow = 2;        // dense points (0.5 m each) per slope measurement

    /// <param name="f">The mesh the path came from (narrow stairs may exist only on the game's mesh).</param>
    internal static void Rewrite(List<Waypoint> path, CartShape shape, NavMeshQueryFilter f)
    {
        // Walk runs from the end so index shifts from earlier replacements don't matter.
        var runs = FindRuns(path);
        for (int r = runs.Count - 1; r >= 0; r--)
            RewriteRun(path, runs[r].start, runs[r].end, shape, f);
    }

    private static List<(int start, int end)> FindRuns(List<Waypoint> path)
    {
        var runs = new List<(int, int)>();
        int start = -1, lastSteep = -1;
        var runDir = Vector3.zero;
        for (int i = 0; i + SlopeWindow < path.Count; i++)
        {
            var d = path[i + SlopeWindow].Pos - path[i].Pos;
            float horiz = PathPlanner.Flat(d).magnitude;
            bool steep = horiz > 0.01f && Mathf.Atan2(Mathf.Abs(d.y), horiz) * Mathf.Rad2Deg > SlopeDeg;
            if (steep)
            {
                var dir = PathPlanner.Flat(d).normalized;
                if (start < 0) { start = i; runDir = dir; }
                else if (Vector3.Angle(dir, runDir) > 35f)
                {
                    // Direction changed (switchback / turning stairwell): each straight flight is its own run.
                    AddRun(path, runs, start, i);
                    start = i + 1;
                    runDir = dir;
                }
                lastSteep = i + SlopeWindow;
            }
            else if (start >= 0 && i > lastSteep + 1) // allow a ~1 m landing gap inside a run
            {
                AddRun(path, runs, start, lastSteep);
                start = -1;
            }
        }
        if (start >= 0) AddRun(path, runs, start, lastSteep);
        return runs;
    }

    private static void AddRun(List<Waypoint> path, List<(int, int)> runs, int s, int e)
    {
        if (Mathf.Abs(path[e].Pos.y - path[s].Pos.y) >= MinRise) runs.Add((s, e));
    }

    private static void RewriteRun(List<Waypoint> path, int s, int e, CartShape shape, NavMeshQueryFilter f)
    {
        Vector3 runStart = path[s].Pos, runEnd = path[e].Pos;
        var mid = (runStart + runEnd) * 0.5f;

        var uphill = UphillDirection(mid, runEnd - runStart);
        var travel = Vector3.Dot(PathPlanner.Flat(runEnd - runStart), uphill) >= 0f ? uphill : -uphill;
        var disp = PathPlanner.Flat(runEnd - runStart);
        if (disp.magnitude > 0.5f && Vector3.Angle(travel, disp) > 45f) travel = disp.normalized;

        // Center across the stair width (the mesh is shrunk equally on both sides, so its midline is the stair midline).
        var side = Vector3.Cross(Vector3.up, travel);
        float l = NavMesh.Raycast(mid, mid - side * 6f, out var hl, f) ? hl.distance : 0f;
        float rgt = NavMesh.Raycast(mid, mid + side * 6f, out var hr, f) ? hr.distance : 0f;
        var center = mid + side * ((rgt - l) * 0.5f);
        float Along(Vector3 p) => Vector3.Dot(PathPlanner.Flat(p - center), travel);

        // Where the climb actually starts/ends: last point still at the start height, first point already at the end height.
        float footAlong = Along(runStart), topAlong = Along(runEnd);
        for (int i = s; i <= e; i++)
        {
            float a = Along(path[i].Pos);
            if (Mathf.Abs(path[i].Pos.y - runStart.y) < 0.15f) footAlong = Mathf.Max(footAlong, a);
            if (Mathf.Abs(path[i].Pos.y - runEnd.y) < 0.15f) topAlong = Mathf.Min(topAlong, a);
        }
        float rise = Mathf.Abs(runEnd.y - runStart.y);
        if (topAlong - footAlong < rise * 0.8f) { footAlong = Along(runStart); topAlong = Along(runEnd); }
        if (topAlong - footAlong < rise * 0.8f)
        {
            Flag(path, s, e);
            return;
        }

        float align = shape.HalfLength + 0.8f; // room to straighten out before the first step
        float exit = shape.HalfLength + 0.5f;  // whole cart is off the stairs before turning

        // Stair-line points at the run's real end heights, so tall stairs don't snap to another floor.
        Vector3 Line(float along, float y) { var p = center + travel * along; p.y = y; return p; }
        // Entry/exit are "nice to have": a wall right before the base or right after the top (stairs that turn) leaves less room.
        // In between, every original path point is kept with its own height (landings, multi-flight stairs) and only
        // shifted sideways onto the centerline; one straight base->top line would cut through landings and upper flights.
        var planned = new List<(float along, float y, float back, bool optional, Vector3 raw)>
        {
            (footAlong - align, runStart.y, -align, true, default), // entry: straight in line with the stairs
        };
        for (int i = s; i <= e; i++) planned.Add((Along(path[i].Pos), path[i].Pos.y, 0f, false, path[i].Pos));
        planned.Add((topAlong + exit, runEnd.y, exit, true, default)); // exit

        // Replace every waypoint inside the entry..exit corridor of THIS flight (its height range, near its centerline),
        // but never the start or the goal. The height/side limits keep it off a parallel flight in a switchback stairwell.
        float yMin = Mathf.Min(runStart.y, runEnd.y) - 0.3f, yMax = Mathf.Max(runStart.y, runEnd.y) + 0.3f;
        bool Inside(Vector3 p)
        {
            float along = Along(p);
            return along >= footAlong - align - 0.25f && along <= topAlong + exit + 0.25f
                && Mathf.Abs(Vector3.Dot(PathPlanner.Flat(p - center), side)) < 1.5f
                && p.y >= yMin && p.y <= yMax;
        }
        int first = Mathf.Max(s, 1), last = Mathf.Min(e, path.Count - 2);
        while (first > 1 && Inside(path[first - 1].Pos)) first--;
        while (last < path.Count - 2 && Inside(path[last + 1].Pos)) last++;
        // Only the real start/goal can cut the stair sequence short; ordinary neighbours (e.g. a turn right after the top) can't.
        float startAlong = first - 1 == 0 ? Along(path[0].Pos) : float.MinValue;
        float goalAlong = last + 1 == path.Count - 1 ? Along(path[path.Count - 1].Pos) : float.MaxValue;

        var insert = new List<Waypoint>();
        foreach (var (along, y, back, optional, raw) in planned)
        {
            // Skip only points truly behind the cart or past the goal; points right at the cart still center it.
            if (along < startAlong - 0.1f || along > goalAlong + 0.1f) continue;

            if (!optional)
            {
                // Centered if the mesh is there, otherwise the original point.
                insert.Add(new Waypoint(Snap(Line(along, y), f, out var p) && Mathf.Abs(p.y - y) < 0.5f ? p : raw) { Stairs = true });
                continue;
            }

            // Optional entry/exit: shrink toward the stairs until it lands straight in line (not shoved sideways by a wall).
            float anchor = along - back;
            var anchorPt = Line(anchor, y);
            foreach (float k in new[] { 1f, 0.6f, 0.3f })
            {
                var want = Line(anchor + back * k, y);
                if (!Snap(want, f, out var p) || PathPlanner.Flat(p - want).magnitude > 0.3f) continue;
                if (!(back < 0f ? PathPlanner.SegmentClear(p, anchorPt, shape) : PathPlanner.SegmentClear(anchorPt, p, shape))) continue;
                insert.Add(new Waypoint(p) { Stairs = true });
                break;
            }
        }
        if (first > last) return;

        path.RemoveRange(first, last - first + 1);
        path.InsertRange(first, insert);
        PathPlanner.Verbose($"Stairs {(runEnd.y > runStart.y ? "up" : "down")}: {insert.Count} centered waypoints, rise {Mathf.Abs(runEnd.y - runStart.y):F1}m, run {topAlong - footAlong:F1}m");
    }

    /// <summary>Uphill direction from floor heights in a ring around the stair midpoint (works for both ramps and real steps).</summary>
    private static Vector3 UphillDirection(Vector3 mid, Vector3 fallback)
    {
        const int n = 24;
        var h = new float[n];
        for (int i = 0; i < n; i++)
        {
            var dir = Quaternion.Euler(0f, i * 360f / n, 0f) * Vector3.forward;
            var origin = mid + dir * 0.8f + Vector3.up * 2f;
            if (!Physics.Raycast(origin, Vector3.down, out var hit, 4f, PathPlanner.WorldMask, QueryTriggerInteraction.Ignore))
                return Fallback(fallback);
            h[i] = hit.point.y;
        }
        int best = 0;
        float bestRise = float.MinValue;
        for (int i = 0; i < n; i++)
        {
            float rise = h[i] - h[(i + n / 2) % n];
            if (rise > bestRise) { bestRise = rise; best = i; }
        }
        return bestRise < 0.2f ? Fallback(fallback) : Quaternion.Euler(0f, best * 360f / n, 0f) * Vector3.forward;
    }

    private static Vector3 Fallback(Vector3 d)
    {
        var flat = PathPlanner.Flat(d).normalized;
        return d.y >= 0f ? flat : -flat;
    }

    private static bool Snap(Vector3 p, NavMeshQueryFilter f, out Vector3 onMesh)
    {
        bool ok = NavMesh.SamplePosition(p, out var hit, 1f, f);
        onMesh = hit.position;
        return ok;
    }

    private static void Flag(List<Waypoint> path, int s, int e)
    {
        for (int i = s; i <= e; i++) { var w = path[i]; w.Stairs = true; path[i] = w; }
    }
}
