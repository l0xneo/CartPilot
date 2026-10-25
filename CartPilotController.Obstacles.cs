using UnityEngine;

namespace CartPilot;

/// <summary>
/// Obstacle behaviour:
///   Driving   -> forward box cast for enemies/players/objects/carts, and a far-side check before entering chokepoints.
///   Yielding  -> stopped; resumes after the path is clear for ClearTimeBeforeResume; after BlockedReplanTime the blocker's
///                area is marked blocked and the route is replanned around it.
///   Reversing -> backing to the last non-choke waypoint when the blocker is close or we're inside a chokepoint.
///   Recovering-> barely moved for StuckTimeout: back up, replan (and block the spot if it happens twice in one place).
/// </summary>
internal partial class CartPilotController
{
    private static readonly int BlockerMask = LayerMask.GetMask("Enemy", "Player", "PhysGrabObject", "PhysGrabObjectCart");
    private static readonly RaycastHit[] Hits = new RaycastHit[32];
    private static readonly Collider[] Overlaps = new Collider[32];

    private const float ProbeLift = 0.3f; // probes start above floor clutter / step lips

    private Vector3 blockerPos;
    private float yieldTimer, clearTimer;
    private Vector3 reverseTarget;
    private float reverseTimer;
    private int committedChoke = -1;

    private float commandedSpeed;
    private Vector3 stuckRef;
    private float stuckTimer;
    private Vector3 lastStuckPos;
    private float lastStuckTime = -999f;
    private bool goalBlockNotified;
    private bool stuckRepeat;
    private Vector3 recoverDir;
    private float recoverTimer;

    private void ResetObstacleState()
    {
        committedChoke = -1;
        stuckRef = transform.position;
        stuckTimer = 0f;
        aheadBlockedFor = 0f;
    }

    // ---------- Driving checks ----------

    /// <summary>Returns true if driving should stop this tick (state changed to yield).</summary>
    private bool CheckObstacles(Vector3 travel)
    {
        SmashDoorAhead(travel);
        if (DropAhead(travel, out var edge))
        {
            Hold();
            Plugin.Log.LogWarning($"Cart '{name}': drop ahead at {edge}, not driving off it.");
            if (++rejectedRoutes > 2) { Notify("only route has a drop, stopped"); Stop(); return true; }
            CartNavMesh.AddBlockedArea(edge, CartRadius);
            SetState(State.WaitMesh);
            return true;
        }

        var v = rb.velocity;
        float speed = new Vector2(v.x, v.z).magnitude;
        float stopDist = 0.6f + speed * speed / (2f * 3f);
        if (Probe(travel, Shape.HalfLength + stopDist, out var hitPos, out var what))
        {
            EnterYield(hitPos, what);
            return true;
        }
        if (!ChokeAheadClear(out var far))
        {
            EnterYield(far, "far side of chokepoint occupied");
            return true;
        }
        return PathAheadBlocked();
    }

    private const float ScanInterval = 0.25f, ScanRange = 15f;
    private float aheadScanTimer, aheadBlockedFor;

    /// <summary>Looks along the route ahead (not just in front of the cart); if it stays blocked, reroute before getting there.</summary>
    private bool PathAheadBlocked()
    {
        if ((aheadScanTimer -= Time.fixedDeltaTime) > 0f) return false;
        aheadScanTimer = ScanInterval;
        if (!FindBlockerOnPath(out var at, out var what)) { aheadBlockedFor = 0f; return false; }

        aheadBlockedFor += ScanInterval;
        if (aheadBlockedFor < Cfg.BlockedReplanTime.Value) return false;
        aheadBlockedFor = 0f;
        if (PathPlanner.Flat(at - goal).magnitude < 2f) return false; // blocker sits on the goal; nothing to route around

        Plugin.Log.LogInfo($"Cart '{name}': route ahead blocked by {what} at {at}, picking a new route.");
        CartNavMesh.AddBlockedArea(at, CartRadius + 0.6f);
        SetState(State.WaitMesh);
        return true;
    }

    private bool FindBlockerOnPath(out Vector3 at, out string what)
    {
        at = default;
        what = null;
        float r = Shape.HalfWidth + 0.2f;
        float range = Mathf.Min(ScanRange, RemainingLength());
        for (float d = 2f; d <= range; d += 1.5f) // the close range is the forward probe's job
        {
            var p = PointAhead(d);
            int n = Physics.OverlapSphereNonAlloc(p + Vector3.up * (ProbeLift + r), r, Overlaps, BlockerMask, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < n; i++)
            {
                if (IgnoreCollider(Overlaps[i])) continue;
                at = p;
                what = Describe(Overlaps[i]);
                return true;
            }
        }
        return false;
    }

    /// <summary>Doors/chokepoints: before entering, make sure the far side has room for the whole cart.</summary>
    private bool ChokeAheadClear(out Vector3 farSide)
    {
        farSide = default;
        if (path[seg].Choke) committedChoke = Mathf.Max(committedChoke, seg); // already inside one

        var pos = PathPlanner.Flat(transform.position);
        int start = -1;
        for (int i = seg + 1; i < path.Count; i++)
        {
            if (PathPlanner.Flat(path[i].Pos - pos).magnitude > 3.5f) break;
            if (path[i].Choke) { start = i; break; }
        }
        if (start < 0 || start <= committedChoke) return true;

        int end = start;
        while (end + 1 < path.Count && path[end + 1].Choke) end++;

        // Far side = path point one cart length past the end of the choke.
        float need = Shape.HalfLength * 2f + 0.5f;
        int j = end;
        while (j + 1 < path.Count && need > 0f) { need -= PathPlanner.Flat(path[j + 1].Pos - path[j].Pos).magnitude; j++; }
        farSide = path[j].Pos;
        var dir = PathPlanner.Flat(path[j].Pos - path[end].Pos);
        if (dir.sqrMagnitude < 0.01f) dir = PathPlanner.Flat(path[end].Pos - path[start].Pos);

        if (Occupied(farSide, dir, out var what))
        {
            PathPlanner.Verbose($"Chokepoint at {path[start].Pos}: far side has {what}.");
            return false;
        }
        committedChoke = end;
        PathPlanner.Verbose($"Chokepoint at {path[start].Pos} clear, committing.");
        return true;
    }

    private void CheckStuck(Vector3 travel)
    {
        if (commandedSpeed < 0.4f) { stuckRef = transform.position; stuckTimer = 0f; return; } // turning in place / braking isn't stuck
        if ((stuckTimer += Time.fixedDeltaTime) < Cfg.StuckTimeout.Value) return;

        float moved = PathPlanner.Flat(transform.position - stuckRef).magnitude;
        stuckRef = transform.position;
        stuckTimer = 0f;
        if (moved > 0.3f) return;

        // Static clutter (rubble) isn't in the blocker mask, so the first stuck already marks the spot; stairs get a second try.
        stuckRepeat = !StairsNear(2f) || (Time.time - lastStuckTime < 20f && PathPlanner.Flat(transform.position - lastStuckPos).magnitude < 2f);
        lastStuckPos = transform.position;
        lastStuckTime = Time.time;
        Plugin.Log.LogInfo($"Cart '{name}' stuck (moved {moved:F2}m in {Cfg.StuckTimeout.Value:F1}s){(stuckRepeat ? ", blocking this spot" : "")}, backing up.");
        if (stuckRepeat) CartNavMesh.AddBlockedArea(transform.position + travel * (Shape.HalfLength + 0.3f), CartRadius + 0.4f);

        recoverDir = -travel;
        recoverTimer = 0f;
        SetState(State.Recovering);
    }

    // ---------- States ----------

    private void EnterYield(Vector3 at, string why)
    {
        blockerPos = at;
        yieldTimer = clearTimer = 0f;
        goalBlockNotified = false;
        Plugin.Log.LogInfo($"Cart '{name}' yielding: {why} at {at}.");

        bool close = PathPlanner.Flat(at - transform.position).magnitude < Shape.HalfLength + 1f;
        if ((close || path[seg].Choke) && FindLastSafe(out reverseTarget))
        {
            reverseTimer = 0f;
            SetState(State.Reversing);
        }
        else SetState(State.Yielding);
    }

    private void Yield()
    {
        Hold();
        yieldTimer += Time.fixedDeltaTime;

        // Is the way forward clear? Probe at least as far as where the blocker was.
        var ahead = PointAhead(1.5f);
        var travel = PathPlanner.Flat(ahead - transform.position).normalized;
        float range = Mathf.Max(Shape.HalfLength + 1.5f, PathPlanner.Flat(blockerPos - transform.position).magnitude + 0.5f);
        bool clear = travel != Vector3.zero && !Probe(travel, range, out _, out _) && ChokeAheadClear(out _);

        clearTimer = clear ? clearTimer + Time.fixedDeltaTime : 0f;
        if (clearTimer >= Cfg.ClearTimeBeforeResume.Value)
        {
            Plugin.Log.LogInfo($"Cart '{name}' path clear, resuming.");
            seg = Mathf.Max(0, seg - 12); // we may have backed up past earlier segments
            committedChoke = -1;
            stuckRef = transform.position;
            stuckTimer = 0f;
            SetState(State.Driving);
            return;
        }

        if (yieldTimer < Cfg.BlockedReplanTime.Value) return;
        yieldTimer = 0f;
        if (PathPlanner.Flat(blockerPos - goal).magnitude < 2f)
        {
            Plugin.Log.LogInfo("Blocker is at the goal itself; still waiting.");
            if (!goalBlockNotified) Notify("something is in my spot, waiting");
            goalBlockNotified = true;
            return;
        }
        Plugin.Log.LogInfo($"Blocked for {Cfg.BlockedReplanTime.Value:F0}s, replanning around it.");
        CartNavMesh.AddBlockedArea(blockerPos, CartRadius + 0.6f);
        SetState(State.WaitMesh);
    }

    private void Reverse()
    {
        reverseTimer += Time.fixedDeltaTime;
        var to = PathPlanner.Flat(reverseTarget - transform.position);
        if (to.magnitude < 0.4f || reverseTimer > 3f || Probe(to.normalized, Shape.HalfLength + 0.4f, out _, out _))
        {
            SetState(State.Yielding);
            return;
        }
        SetYawRate(0f); // back straight out, don't swing the body
        ApplyVelocity(to.normalized * 1f);
    }

    private void Recover()
    {
        recoverTimer += Time.fixedDeltaTime;
        bool blockedBehind = Probe(recoverDir, Shape.HalfLength + 0.4f, out _, out _);
        if (recoverTimer < 1f && !blockedBehind)
        {
            SetYawRate(0f);
            ApplyVelocity(recoverDir * 1.2f);
            return;
        }
        Hold();
        if (stuckRepeat) SetState(State.WaitMesh); // blocked area added: wait for the mesh update, then replan
        else Replan();
    }

    private void Hold()
    {
        ApplyVelocity(Vector3.zero);
        SetYawRate(0f);
    }

    private bool FindLastSafe(out Vector3 p)
    {
        var pos = PathPlanner.Flat(transform.position);
        for (int i = seg; i >= 0; i--)
        {
            float d = PathPlanner.Flat(path[i].Pos - pos).magnitude;
            if (d > 6f) break;
            if (!path[i].Choke && d >= 1f) { p = path[i].Pos; return true; }
        }
        p = default;
        return false;
    }

    // ---------- Sensing ----------

    // Same floor layers the game uses for its own ground checks.
    private static readonly int FloorMask = LayerMask.GetMask("Default", "NavmeshOnly", "PlayerOnlyCollision");
    private const float MaxDrop = 1f; // stairs going down drop ~0.3 m over this distance; anything past 1 m is a ledge

    /// <summary>Is there floor just past the cart's front? Stops it driving off ledges/into holes whatever the path says.</summary>
    private bool DropAhead(Vector3 travel, out Vector3 point)
    {
        float ahead = Shape.HalfLength + 0.4f;
        if (StairsNear(ahead + 1f)) { point = default; return false; } // stairs the planner already laid out; this guard is for unplanned ledges
        point = transform.position + travel * ahead + Vector3.up * Shape.BottomY;
        // Doorways read as a drop (no floor collider under the threshold) and stopped the cart at a closed door.
        if (DoorAt(point + Vector3.up, 1.2f) != null) return false;
        // Allow whatever drop the path itself expects there (steep stairs going down), plus the usual margin.
        float expectedDrop = Mathf.Max(0f, point.y - PointAhead(ahead).y);
        // Start well above the cart: on steep stairs going up, the floor ahead is above the cart's bottom.
        const float up = 1.8f;
        bool Floor(Vector3 p) => Physics.Raycast(p + Vector3.up * up, Vector3.down, up + expectedDrop + MaxDrop, FloorMask, QueryTriggerInteraction.Ignore);
        if (Floor(point)) return false;
        // A small hole, not a ledge: floor resumes before the cart's middle would be over it.
        for (float d = 0.2f; d <= MaxGap; d += 0.2f)
            if (Floor(point + travel * d)) return false;
        return true;
    }

    private static readonly int DoorSearchMask = LayerMask.GetMask("PhysGrabObjectHinge", "PhysGrabObject");

    private static PhysGrabHinge DoorAt(Vector3 at, float radius)
    {
        int n = Physics.OverlapSphereNonAlloc(at, radius, Overlaps, DoorSearchMask, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < n; i++)
        {
            var h = Overlaps[i].GetComponentInParent<PhysGrabHinge>();
            if (h != null && !h.broken && !h.dead) return h;
        }
        return null;
    }

    /// <summary>A door right in front of the cart gets smashed off its hinges (same as an enemy breaking it) instead of blocking.</summary>
    private void SmashDoorAhead(Vector3 travel)
    {
        var door = DoorAt(transform.position + travel * (Shape.HalfLength + 0.6f), Shape.HalfWidth + 0.3f);
        if (door == null) return;
        Plugin.Log.LogInfo($"Cart '{name}': smashing door '{door.name}'.");
        door.DestroyHinge();
    }

    private Vector3 ProbeHalfExtents(float halfZ) =>
        new(Shape.HalfWidth + 0.05f, Mathf.Max(0.1f, (Shape.Height - ProbeLift) * 0.5f), halfZ);

    private Vector3 ProbeCenter(Vector3 at, Vector3 half) => at + Vector3.up * (Shape.BottomY + ProbeLift + half.y);

    /// <summary>Cart-wide box cast from the cart's center along <paramref name="dir"/>. Ignores the cart itself and anything inside it (cargo, riders).</summary>
    private bool Probe(Vector3 dir, float dist, out Vector3 hitPos, out string what)
    {
        hitPos = default;
        what = null;
        var half = ProbeHalfExtents(0.1f);
        int n = Physics.BoxCastNonAlloc(ProbeCenter(transform.position, half), half, dir, Hits, Quaternion.LookRotation(dir), dist, BlockerMask, QueryTriggerInteraction.Ignore);
        float best = float.MaxValue;
        for (int i = 0; i < n; i++)
        {
            var h = Hits[i];
            if (h.distance <= 0f || h.distance >= best || IgnoreCollider(h.collider)) continue;
            var local = transform.InverseTransformPoint(h.point);
            if (Mathf.Abs(local.x) < Shape.HalfWidth && Mathf.Abs(local.z) < Shape.HalfLength) continue; // inside the cart
            best = h.distance;
            hitPos = h.point;
            what = Describe(h.collider);
        }
        return what != null;
    }

    private bool Occupied(Vector3 at, Vector3 dir, out string what)
    {
        what = null;
        var half = ProbeHalfExtents(Shape.HalfLength + 0.2f);
        half.x += 0.15f;
        int n = Physics.OverlapBoxNonAlloc(ProbeCenter(at, half), half, Overlaps, Quaternion.LookRotation(dir), BlockerMask, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < n; i++)
        {
            if (IgnoreCollider(Overlaps[i])) continue;
            what = Describe(Overlaps[i]);
            return true;
        }
        return false;
    }

    private bool IgnoreCollider(Collider c)
    {
        if (c.attachedRigidbody == rb || c.transform.IsChildOf(transform)) return true; // own parts (e.g. its button)
        if (IsPushableItem(c)) return true;
        if (c.GetComponentInParent<ExtractionPoint>() != null) return true; // its button and frame sit right at the goal
        // The player we're driving to: arrival handles them, they're not an obstacle.
        return follow != null && c.gameObject.layer == LayerMask.NameToLayer("Player") && PathPlanner.Flat(c.transform.position - follow.position).magnitude < 1.5f;
    }

    /// <summary>Loose non-valuable items (upgrades, weapons, tools) are just pushed aside. Valuables still block: bumping them costs money.</summary>
    private static bool IsPushableItem(Collider c)
    {
        var obj = c.GetComponentInParent<PhysGrabObject>();
        return obj != null
            && obj.GetComponent<ValuableObject>() == null
            && obj.GetComponent<PlayerTumble>() == null      // tumbling player
            && obj.GetComponent<PhysGrabCart>() == null
            && c.GetComponentInParent<EnemyRigidbody>() == null;
    }

    private static string Describe(Collider c)
    {
        var col = $"[{c.name}/{LayerMask.LayerToName(c.gameObject.layer)}]"; // which collider, so false blockers can be traced
        var enemy = c.GetComponentInParent<EnemyParent>();
        if (enemy != null || c.GetComponentInParent<EnemyRigidbody>() != null || c.GetComponentInParent<Enemy>() != null)
            return $"enemy '{(enemy != null ? enemy.enemyName : "?")}'{col}";
        var avatar = c.GetComponentInParent<PlayerAvatar>();
        if (avatar != null) return $"player '{avatar.playerName}'{col}";
        if (c.gameObject.layer == LayerMask.NameToLayer("Player")) return $"player{col}";
        if (c.GetComponentInParent<PhysGrabCart>() != null) return $"cart{col}";
        var obj = c.GetComponentInParent<PhysGrabObject>();
        return obj != null ? $"object '{obj.name.Replace("(Clone)", "")}'{col}" : $"'{c.name}'{col}";
    }
}

internal partial class CartPilotController
{
    private Destination lastDest;
    private float diagTickTimer;

    /// <summary>Writes one diagnostic line: the event plus everything about the cart's situation right now.</summary>
    private void Diag(string evt, string detail)
    {
        if (DiagLog.FilePath == null) return;
        try { DiagLog.Write(evt, $"cart={name} {detail} | {Snapshot()}"); }
        catch (System.Exception e) { DiagLog.Write(evt, $"cart={name} {detail} | snapshot failed: {e.Message}"); }
    }

    private string Snapshot()
    {
        var sb = new System.Text.StringBuilder();
        var p = transform.position;
        var v = rb.velocity;
        sb.Append($"state={state} dest={lastDest} pos=({p.x:F1},{p.y:F1},{p.z:F1}) yaw={transform.eulerAngles.y:F0} room={Room()}");
        var e = transform.eulerAngles;
        sb.Append($" speed={new Vector2(v.x, v.z).magnitude:F2} vy={v.y:F2} cmdSpeed={commandedSpeed:F2} spin={rb.angularVelocity.y * Mathf.Rad2Deg:F0}deg/s tilt=({Mathf.DeltaAngle(0f, e.x):F0},{Mathf.DeltaAngle(0f, e.z):F0}) facingBack={facingBack}");
        var holders = new System.Text.StringBuilder();
        foreach (var g in cart.physGrabObject.playerGrabbing)
            if (g != null && g.playerAvatar != null) holders.Append(g.playerAvatar.playerName).Append(',');
        if (holders.Length > 0) sb.Append($" heldBy=[{holders}]");
        if (follow != null) sb.Append($" following=({follow.position.x:F1},{follow.position.y:F1},{follow.position.z:F1})@{PathPlanner.Flat(follow.position - p).magnitude:F1}");
        sb.Append($" size=(w{Shape.HalfWidth * 2f:F2} l{Shape.HalfLength * 2f:F2} h{Shape.Height:F2}) navR={CartRadius:F2}");

        float itemMass = 0f, value = 0f;
        int count = 0;
        var items = new System.Text.StringBuilder();
        foreach (var item in cart.itemsInCart)
        {
            if (item == null) continue;
            count++;
            float m = item.rb != null ? item.rb.mass : 0f;
            itemMass += m;
            var val = item.GetComponent<ValuableObject>();
            if (val != null) value += val.dollarValueCurrent;
            items.Append($"{item.name.Replace("(Clone)", "")}:{m:F1}kg{(val != null ? $":${val.dollarValueCurrent:F0}" : "")},");
        }
        sb.Append($" cartMass={rb.mass:F1} items={count} itemMass={itemMass:F1} value={value:F0} [{items}]");

        if (state != State.Idle) sb.Append($" goal=({goal.x:F1},{goal.y:F1},{goal.z:F1}) goalDist={PathPlanner.Flat(goal - p).magnitude:F1}");
        if (path != null && seg + 1 < path.Count)
            sb.Append($" seg={seg}/{path.Count - 1} offPath={offPath:F2} left={RemainingLength():F1} stairs={path[seg].Stairs} choke={path[seg].Choke} door={path[seg].Door} room={path[seg].Room:F2}");

        sb.Append($" walls(f/b/l/r)={Wall(transform.forward)}/{Wall(-transform.forward)}/{Wall(-transform.right)}/{Wall(transform.right)}");
        sb.Append($" near=[{Nearby()}]");
        return sb.ToString();
    }

    private static readonly int WallMask = LayerMask.GetMask("Default");

    /// <summary>Gap between the cart's side and the nearest wall in that direction (up to 3 m).</summary>
    private string Wall(Vector3 dir)
    {
        var half = ProbeHalfExtents(0f);
        float own = Mathf.Abs(Vector3.Dot(dir, transform.right)) > 0.5f ? Shape.HalfWidth : Shape.HalfLength;
        return Physics.Raycast(ProbeCenter(transform.position, half), dir, out var hit, own + 3f, WallMask, QueryTriggerInteraction.Ignore)
            ? (hit.distance - own).ToString("F2") : "-";
    }

    private string Nearby()
    {
        var sb = new System.Text.StringBuilder();
        int n = Physics.OverlapSphereNonAlloc(transform.position, Shape.HalfLength + 3f, Overlaps, BlockerMask, QueryTriggerInteraction.Ignore);
        var seen = new System.Collections.Generic.HashSet<Object>();
        for (int i = 0; i < n; i++)
        {
            var c = Overlaps[i];
            if (c.attachedRigidbody == rb || c.transform.IsChildOf(transform)) continue;
            var obj = c.GetComponentInParent<PhysGrabObject>();
            if (obj != null && cart.itemsInCart.Contains(obj)) continue; // cargo is listed with the items
            if (!seen.Add(c.attachedRigidbody != null ? c.attachedRigidbody : c)) continue; // one entry per object
            var at = c.bounds.ClosestPoint(transform.position); // bounds: Collider.ClosestPoint warns on concave meshes
            sb.Append($"{Describe(c)}{(IsPushableItem(c) ? "(push)" : "")}{(IgnoreCollider(c) ? "(ignored)" : "")}@{PathPlanner.Flat(at - transform.position).magnitude:F1},");
        }
        return sb.ToString();
    }

    private string Room()
    {
        int n = Physics.OverlapSphereNonAlloc(transform.position, 0.2f, Overlaps, ~0, QueryTriggerInteraction.Collide);
        for (int i = 0; i < n; i++)
        {
            var rv = Overlaps[i].GetComponentInParent<RoomVolume>();
            if (rv == null) continue;
            var module = rv.GetComponentInParent<Module>();
            var label = (module != null ? module.name : rv.name).Replace("(Clone)", "").Trim().Replace(' ', '_');
            return rv.Truck ? label + "(truck)" : rv.Extraction ? label + "(extraction)" : label;
        }
        return "?";
    }

    private int cargoCount;
    private float cargoValue, cargoLogCooldown, cargoSeen;

    private void DiagTick()
    {
        CargoWatch();
        if ((diagTickTimer -= Time.fixedDeltaTime) > 0f) return;
        diagTickTimer = 1f;
        Diag("tick", "");
    }

    /// <summary>Logs CARGO when items leave the cart or lose value (breaking) while driving, with the spin and speed that caused it.</summary>
    private void CargoWatch()
    {
        int count = 0;
        float value = 0f;
        foreach (var item in cart.itemsInCart)
        {
            if (item == null) continue;
            count++;
            var val = item.GetComponent<ValuableObject>();
            if (val != null) value += val.dollarValueCurrent;
        }
        bool fresh = Time.time - cargoSeen > 0.2f; // first tick of a drive: players may have unloaded it while idle
        cargoSeen = Time.time;
        bool lost = !fresh && (count < cargoCount || value < cargoValue - 1f);
        cargoLogCooldown -= Time.fixedDeltaTime;
        if (lost && cargoLogCooldown > 0f) return; // keep the old baseline so the next line shows the total
        if (lost)
        {
            cargoLogCooldown = 0.5f;
            var gone = new System.Text.StringBuilder();
            foreach (var item in cargoBefore)
                if (item != null && !cart.itemsInCart.Contains(item))
                {
                    var at = item.transform.position - transform.position;
                    gone.Append($"{item.name.Replace("(Clone)", "")}@{PathPlanner.Flat(at).magnitude:F1}m,");
                }
                else if (item == null) gone.Append("(destroyed),");
            Diag("CARGO", $"items {cargoCount}->{count} value {cargoValue:F0}->{value:F0} left=[{gone}] spin={rb.angularVelocity.y * Mathf.Rad2Deg:F0}deg/s speed={PathPlanner.Flat(rb.velocity).magnitude:F2}");
        }
        cargoCount = count;
        cargoValue = value;
        cargoBefore.Clear();
        cargoBefore.AddRange(cart.itemsInCart);
    }

    private readonly System.Collections.Generic.List<PhysGrabObject> cargoBefore = new();
    private float hitLogCooldown;

    /// <summary>Logs HIT for hard knocks while driving (walls, doors, players, enemies), the kind that break valuables.</summary>
    private void OnCollisionEnter(Collision c)
    {
        if (state == State.Idle || DiagLog.FilePath == null || Time.time < hitLogCooldown) return;
        float rel = c.relativeVelocity.magnitude;
        if (rel < 1.5f) return;
        var obj = c.collider.GetComponentInParent<PhysGrabObject>();
        if (obj != null && cart.itemsInCart.Contains(obj)) return; // cargo settling
        hitLogCooldown = Time.time + 0.25f;
        var at = c.contactCount > 0 ? c.GetContact(0).point : c.transform.position;
        Diag("HIT", $"{Describe(c.collider)} relSpeed={rel:F2} impulse={c.impulse.magnitude:F1} at=({at.x:F1},{at.y:F1},{at.z:F1})");
    }
}
