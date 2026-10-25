using System.Collections.Generic;
using Photon.Pun;
using UnityEngine;
using UnityEngine.AI;

namespace CartPilot;

/// <summary>
/// Lives on every big cart. Any client can send a command; only the host plans and drives,
/// because only the host simulates cart physics and has a NavMesh.
/// </summary>
internal partial class CartPilotController : MonoBehaviourPun
{
    internal static readonly List<CartPilotController> All = new();

    private PhysGrabCart cart;
    private Rigidbody rb;
    internal CartShape Shape;

    private void Awake()
    {
        cart = GetComponent<PhysGrabCart>();
        rb = GetComponent<Rigidbody>();
    }

    private void Start()
    {
        Shape = CartShape.Measure(rb);
        Plugin.Log.LogInfo($"Cart '{name}' measured: {Shape}");
        Diag("spawn", "");
    }

    private void OnEnable() => All.Add(this);
    private void OnDisable() => All.Remove(this);

    private float lastDrivingTime = -999f;

    /// <summary>Is this point inside a cart that is driving (or stopped under 2 s ago, while cargo settles)?</summary>
    internal static bool CarryingWhileDriving(Vector3 pos)
    {
        foreach (var c in All)
        {
            if (Time.time - c.lastDrivingTime > 2f || c.cart.inCart == null) continue;
            var local = c.cart.inCart.InverseTransformPoint(pos); // "In Cart" is a unit box scaled to the bed
            if (Mathf.Abs(local.x) < 0.6f && Mathf.Abs(local.z) < 0.6f && local.y > -0.6f && local.y < 1.5f) return true;
        }
        return false;
    }

    internal static CartPilotController Nearest(Vector3 pos)
    {
        CartPilotController best = null;
        float bestDist = float.MaxValue;
        foreach (var c in All)
        {
            float d = (c.transform.position - pos).sqrMagnitude;
            if (d < bestDist) { bestDist = d; best = c; }
        }
        return best;
    }

    // ---------- Commands ----------

    internal void SendCommand(Destination dest, Vector3 playerPos)
    {
        if (SemiFunc.IsMasterClientOrSingleplayer()) CartPilotGoToRPC((int)dest, playerPos);
        else photonView.RPC(nameof(CartPilotGoToRPC), RpcTarget.MasterClient, (int)dest, playerPos);
    }

    [PunRPC]
    private void CartPilotGoToRPC(int dest, Vector3 playerPos, PhotonMessageInfo info = default)
    {
        if (!SemiFunc.IsMasterClientOrSingleplayer()) return;
        if (!System.Enum.IsDefined(typeof(Destination), dest))
        {
            Plugin.Log.LogWarning($"Ignoring unknown destination {dest} from client.");
            return;
        }
        commander = info.Sender; // null when the host/singleplayer pressed the key
        GoTo((Destination)dest, playerPos);
    }

    private Photon.Realtime.Player commander;

    /// <summary>Logs a failure and shows it on the screen of whoever gave the command.</summary>
    private void Notify(string message)
    {
        Plugin.Log.LogWarning($"Cart '{name}': {message}");
        Diag("FAIL", message);
        if (commander == null || commander.IsLocal) CartPilotNotifyRPC(message);
        else photonView.RPC(nameof(CartPilotNotifyRPC), commander, message);
    }

    private string notice;
    private float noticeUntil;

    [PunRPC]
    private void CartPilotNotifyRPC(string message)
    {
        notice = "Cart: " + message;
        noticeUntil = Time.time + 3f;
    }

    // The game's big center message must be re-sent every frame (it hides itself after 0.2 s), like extraction messages do.
    private void Update()
    {
        if (notice != null && Time.time < noticeUntil)
            SemiFunc.UIBigMessage(notice, "{:O}", 20f, Color.white, new Color(1f, 0.55f, 0f));
    }

    private void GoTo(Destination dest, Vector3 playerPos)
    {
        lastDest = dest;
        Diag("command", $"{dest} by {(commander == null ? "host" : commander.NickName)}");
        if (BeingHandled())
        {
            Notify("someone is holding me");
            return;
        }
        HudReset();
        extraction = null;
        extractionInside = false;
        if (!ResolveDestination(dest, playerPos, out goal, out follow))
        {
            Notify(dest == Destination.Extraction || dest == Destination.ExtractionOpen ? "no extraction point available" : $"can't find the {dest.ToString().ToLower()}");
            HudSend(HudState.NoPath);
            return;
        }
        // Same target just failed and the cart hasn't moved: answer now instead of replanning (and rebuilding the mesh) again.
        if (Time.time - failTime < 3f && (goal - failGoal).magnitude < 1f && (transform.position - failPos).magnitude < 0.5f)
        {
            Notify("still no way there");
            HudSend(HudState.NoPath);
            return;
        }
        Plugin.Log.LogInfo($"Cart '{name}' -> {dest} at {goal}");
        // Don't park on top of the player.
        arriveDistance = follow != null ? Shape.HalfLength + 1.2f : 0.8f;
        rejectedRoutes = 0;
        SetState(State.WaitMesh);
    }

    // ---------- State machine ----------

    private enum State { Idle, WaitMesh, Driving, Yielding, Reversing, Recovering }

    private State state = State.Idle;
    private Vector3 goal;
    private Transform follow;
    private float arriveDistance;
    private int rejectedRoutes;
    private float failTime = -10f;
    private Vector3 failGoal, failPos;
    private ExtractionPoint extraction; // set while heading to an extraction
    private bool extractionInside;      // second leg: button pressed, now driving inside
    private bool extractionStayOutside; // press the button but don't drive in

    private List<Waypoint> path;
    private int seg;          // current path segment (path[seg] -> path[seg+1])
    private float segT;       // 0..1 along it
    private float offPath;    // horizontal distance from the path
    private bool facingBack;  // driving with the cart's back end first
    private float lastPlanTime;
    private float followCheckTimer;
    private PathDebugDraw debug;

    private float CartRadius => (Cfg.CartRadius.Value > 0f ? Cfg.CartRadius.Value : Shape.HalfWidth) + Cfg.ClearanceMargin.Value;

    private void SetState(State s)
    {
        if (s != state)
        {
            PathPlanner.Verbose($"Cart '{name}': {state} -> {s}");
            Diag("state", $"{state}->{s}");
        }
        if ((s == State.Idle) != (state == State.Idle)) SetSlippery(s != State.Idle);
        state = s;
    }

    /// <summary>
    /// An unheld cart is "Locked" with high-friction colliders, which eats our velocity every physics step.
    /// While driving, use the same slippery materials the game uses when a player handles the cart.
    /// </summary>
    private void SetSlippery(bool on)
    {
        if (!on && cart.currentState != PhysGrabCart.State.Locked) return; // a player took it; the game set its own materials
        foreach (var c in cart.capsuleColliders) c.material = on ? cart.physMaterialSlippery : cart.physMaterialNormal;
        foreach (var c in cart.cartInside) c.material = on ? SemiFunc.PhysicMaterialPhysGrabObject() : cart.physMaterialNormal;
    }

    private void OnDestroy() => debug?.Destroy();

    private void FixedUpdate()
    {
        if (!SemiFunc.IsMasterClientOrSingleplayer()) return;
        CartNavMesh.Tick();
        if (state == State.Idle) return;
        lastDrivingTime = Time.time;
        DiagTick();
        HudTick();

        if (BeingHandled())
        {
            Plugin.Log.LogInfo("Player took the cart, stopping.");
            Stop(HudState.Hidden);
            return;
        }

        switch (state)
        {
            case State.WaitMesh:
                if (CartNavMesh.EnsureBaked(CartRadius, MaxGap)) Replan();
                break;
            case State.Driving:
                Drive();
                break;
            case State.Yielding:
                Yield();
                break;
            case State.Reversing:
                Reverse();
                break;
            case State.Recovering:
                Recover();
                break;
        }
    }

    // Not cart.cartBeingPulled: the game only clears it once nobody is in the grab area, so it stays true after letting go.
    private bool BeingHandled() => cart.physGrabObject.playerGrabbing.Count > 0;

    private void Stop(HudState result = HudState.NoPath)
    {
        SetState(State.Idle);
        path = null;
        debug?.Clear();
        rb.velocity = new Vector3(0f, rb.velocity.y, 0f);
        rb.angularVelocity = Vector3.zero;
        yawCmd = 0f;
        if (result == HudState.NoPath) { failTime = Time.time; failGoal = follow != null ? follow.position : goal; failPos = transform.position; }
        HudSend(result);
    }

    /// <param name="sticky">Player moved while we drive: keep the current route unless the new one is about as short
    /// (stops it flipping to a route through other rooms every couple of seconds).</param>
    private void Replan(bool sticky = false)
    {
        lastPlanTime = Time.time;
        var oldGoal = goal;
        if (follow != null) goal = follow.position;

        var from = transform.position + Vector3.up * Shape.BottomY; // floor under the cart
        var planTimer = System.Diagnostics.Stopwatch.StartNew();
        var res = PathPlanner.Plan(from, goal, Shape);
        // Player up somewhere the cart can't climb (jumped onto it): park on the floor below them instead.
        // A partial route that still ends up on the player's level is kept; only a route stuck below falls back.
        if (follow != null && Unreached(res, goal) && (res.Path == null || res.Path[res.Path.Count - 1].Pos.y < goal.y - 1f)
            && PathPlanner.FloorBelow(goal, out var below))
        {
            var under = PathPlanner.Plan(from, below, Shape);
            if (!Unreached(under, below))
            {
                Plugin.Log.LogInfo($"Cart '{name}': player is up out of reach, going to the floor below at {below}.");
                res = under;
                goal = below;
            }
        }
        Diag("plantime", $"{planTimer.Elapsed.TotalMilliseconds:F1}ms");
        if (sticky && path != null)
        {
            float keep = RemainingLength() + PathPlanner.Flat(goal - oldGoal).magnitude + 4f;
            if (res.Path == null || res.Partial || PathLength(res.Path) > keep)
            {
                PathPlanner.Verbose($"Player moved; new route is longer or worse, keeping the current one.");
                goal = oldGoal;
                return;
            }
        }
        if (res.Path == null)
        {
            Plugin.Log.LogWarning($"Cart '{name}': route rejected ({res.Error}).");
            if (res.FailPoint == null)
            {
                Notify("no way through for me, stopped");
                Stop();
                return;
            }
            // The only route is a player-width gap; blocking it and retrying finds nothing new.
            if (res.TooNarrow)
            {
                Notify("too narrow for the cart here, stopped");
                Stop();
                return;
            }
            if (++rejectedRoutes > 2)
            {
                Notify("no way through for me, stopped");
                Stop();
                return;
            }
            // Make the planner avoid the failing spot and try again once the mesh is updated.
            CartNavMesh.AddBlockedArea(res.FailPoint.Value, CartRadius + 0.5f);
            SetState(State.WaitMesh);
            return;
        }

        if (res.Partial) Plugin.Log.LogInfo("Goal not fully reachable for the cart; driving as close as possible.");
        // Partial route that ends where the cart already is: it used to "arrive" on the spot over and over.
        if (res.Partial && PathLength(res.Path) < 1f && PathPlanner.Flat(goal - transform.position).magnitude > arriveDistance + 1f)
        {
            Notify("can't get any closer, stopped");
            Stop();
            return;
        }
        path = res.Path;
        seg = 0;
        segT = 0f;
        ResetObstacleState();
        debug ??= new PathDebugDraw();
        debug.SetPath(path);
        Diag("plan", $"ok waypoints={path.Count} partial={res.Partial}");
        SetState(State.Driving);
    }

    /// <summary>No route, or one that ends well away from the target (3D: the floor under a ledge is not the ledge).</summary>
    private static bool Unreached(PlanResult r, Vector3 target) =>
        r.Path == null ? r.FailPoint == null : r.Partial && (r.Path[r.Path.Count - 1].Pos - target).magnitude > 1.5f;

    // ---------- Pure pursuit ----------

    private void Drive()
    {
        UpdateProgress();

        if (offPath > 1.2f && Time.time - lastPlanTime > 0.5f)
        {
            PathPlanner.Verbose($"Off path by {offPath:F1}m, replanning.");
            Replan();
            return;
        }
        if (follow != null && (followCheckTimer -= Time.fixedDeltaTime) < 0f)
        {
            followCheckTimer = 2f;
            if (PathPlanner.Flat(follow.position - goal).magnitude > 3f) { Replan(sticky: true); return; }
        }

        float remaining = RemainingLength();
        // Close to the player: done, whatever the path says (otherwise it drives into them and pops up).
        if (follow != null && PathPlanner.Flat(follow.position - transform.position).magnitude < arriveDistance + 0.3f
            && Mathf.Abs(follow.position.y - transform.position.y) < 1.5f) remaining = 0f; // same floor only, not the player upstairs
        // A path that ends short of the player (no room for the cart there) is driven to its end, not "arrived" at the start.
        bool endsAtGoal = PathPlanner.Flat(path[path.Count - 1].Pos - goal).magnitude < 1f;
        if (remaining < (endsAtGoal ? arriveDistance : 0.5f))
        {
            // A partial path "arrives" wherever it ends; don't press an extraction button from across the map.
            if (extraction != null && PathPlanner.Flat(goal - transform.position).magnitude > 3f)
            {
                Notify("can't reach the extraction, stopped");
                Stop();
                return;
            }
            if (extraction != null && !extractionInside)
            {
                // Same as the player pressing the button (OnClick ignores it unless the point is Idle and unlocked).
                extraction.OnClick();
                if (extractionStayOutside)
                {
                    Plugin.Log.LogInfo($"Cart '{name}' at extraction: pressed button, staying outside.");
                    Stop(HudState.Arrived);
                    return;
                }
                Plugin.Log.LogInfo($"Cart '{name}' at extraction: pressed button, driving inside.");
                extractionInside = true;
                goal = extraction.transform.position;
                arriveDistance = 0.5f;
                Replan();
                return;
            }
            // End of a kept route but the player walked on: plan fresh from here.
            if (follow != null && endsAtGoal && Time.time - lastPlanTime > 1f && PathPlanner.Flat(follow.position - transform.position).magnitude > arriveDistance + 2f
                && Mathf.Abs(follow.position.y - goal.y) < 1.5f) // not when parked under a player up on a ledge
            {
                Replan();
                return;
            }
            // Route ended (target snapped onto mesh near the cart) but the real target is still far away.
            if (PathPlanner.Flat(goal - transform.position).magnitude > arriveDistance + 1f)
            {
                Plugin.Log.LogInfo($"Cart '{name}' route ended {PathPlanner.Flat(goal - transform.position).magnitude:F1}m from target.");
                Notify("can't get any closer, stopped");
                Stop();
                return;
            }
            Plugin.Log.LogInfo($"Cart '{name}' arrived.");
            Stop(HudState.Arrived);
            return;
        }

        bool stairs = StairsNear(1.5f);
        float speed = TargetSpeed(remaining, stairs);
        float look = stairs ? Cfg.StairsLookahead.Value : Mathf.Lerp(0.8f, Cfg.Lookahead.Value, speed / Mathf.Max(0.1f, Cfg.MaxSpeed.Value));
        var target = PointAhead(look);
        debug?.SetLookahead(target);

        var travel = PathPlanner.Flat(target - transform.position).normalized;
        if (CheckObstacles(travel)) return;

        Steer(target, speed);
        if (stairs) StairsAssist();
        CheckStuck(travel);
    }

    /// <summary>Velocity toward the lookahead point, yaw toward the travel direction. Speed drops while badly misaligned so the long body doesn't sweep walls.</summary>
    private void Steer(Vector3 target, float speed)
    {
        var dir = PathPlanner.Flat(target - transform.position);
        if (dir.sqrMagnitude < 0.0001f) return;
        dir.Normalize();

        var fwd = PathPlanner.Flat(transform.forward).normalized;
        // Pick whichever end is closer to the travel direction, with hysteresis so it doesn't flip-flop.
        float angFwd = Vector3.Angle(fwd, dir);
        if (facingBack ? angFwd < 60f : angFwd > 120f) facingBack = !facingBack;
        var body = facingBack ? -fwd : fwd;

        float yawErr = Vector3.SignedAngle(body, dir, Vector3.up);
        float yawRate = Mathf.Clamp(yawErr * Mathf.Deg2Rad * 6f, -4f, 4f);
        SetYawRate(yawRate);

        float align = Mathf.Clamp01(1f - (Mathf.Abs(yawErr) - 25f) / 50f); // full speed under 25 deg, turn in place only past 75
        commandedSpeed = speed * align;
        ApplyVelocity(dir * commandedSpeed);
    }

    private float yawCmd;

    private void SetYawRate(float yawRate)
    {
        // Spinning fast flings cargo out of the cart ends (4 rad/s emptied a full cart in half a second on Manor).
        // Cap the cart-end speed at 1.5 m/s and the sideways pull while driving at 3 m/s^2.
        float speed = PathPlanner.Flat(rb.velocity).magnitude;
        float max = Mathf.Min(1.5f / Mathf.Max(0.5f, Shape.HalfLength), 3f / Mathf.Max(0.1f, speed));
        yawRate = Mathf.Clamp(yawRate, -max, max);
        // Ramp our own command, not the measured spin: friction under heavy cargo zeroes the measured value every tick,
        // so ramping from it never got past 0.05 rad/s and the cart sat still trying to turn (Arctic, 18 items, 20 s).
        yawCmd = Mathf.MoveTowards(yawCmd, yawRate, 2.5f * Time.fixedDeltaTime);
        rb.angularVelocity = new Vector3(0f, yawCmd, 0f);
    }

    private void ApplyVelocity(Vector3 horizontal)
    {
        var v = rb.velocity;
        // Strong enough to get over step edges; the game's own pull uses far more (up to 2 m/s per step).
        // Gentle, so cargo isn't jolted (guns in the cart fire on hard knocks); the hard push is only for climbing stairs.
        float accel = state == State.Driving && StairsNear(1.5f) ? 30f : 10f; // 5 lost to floor friction and topped out near 2 m/s
        var flat = Vector3.MoveTowards(new Vector3(v.x, 0f, v.z), horizontal, accel * Time.fixedDeltaTime);
        rb.velocity = new Vector3(flat.x, v.y, flat.z);
    }

    private float TargetSpeed(float remaining, bool stairs)
    {
        float max = Cfg.MaxSpeed.Value;

        // Turn sharpness over the next few meters.
        var here = PointAhead(0f);
        var near = PointAhead(1f);
        var far = PointAhead(3f);
        float turn = Vector3.Angle(PathPlanner.Flat(near - here), PathPlanner.Flat(far - near));
        float speed = Mathf.Lerp(max, Mathf.Min(max, Cfg.CornerSpeed.Value), Mathf.Clamp01(turn / 90f));

        // Clearance: room beyond the cart's own footprint (tight mesh edge distance).
        // Precomputed at plan time: registering our meshes every physics tick tanked the frame rate.
        speed *= Mathf.Lerp(0.5f, 1f, Mathf.Clamp01(path[seg].Room * 0.5f));

        if (stairs) speed = Mathf.Min(speed, Cfg.StairsSpeed.Value);
        if (DoorNear(2.5f)) speed = Mathf.Min(speed, Cfg.DoorSpeed.Value);
        speed = Mathf.Min(speed, Mathf.Sqrt(2f * 2f * Mathf.Max(0f, remaining - arriveDistance)) + 0.3f); // brake for arrival
        return Mathf.Max(speed, Mathf.Min(Cfg.MinSpeed.Value, 0.5f));
    }

    private void StairsAssist()
    {
        var a = path[seg].Pos;
        var b = path[Mathf.Min(seg + 1, path.Count - 1)].Pos;
        float run = PathPlanner.Flat(b - a).magnitude;
        if (b.y <= a.y + 0.05f || run < 0.05f) return; // only when climbing

        // Drive along the slope, not just horizontally, so the cart rides up over the step edges.
        float climb = commandedSpeed * (b.y - a.y) / run;
        if (rb.velocity.y < climb) rb.velocity = new Vector3(rb.velocity.x, climb, rb.velocity.z);

        if (Cfg.StairsUpAssist.Value <= 0f) return;
        float mass = rb.mass;
        foreach (var item in cart.itemsInCart)
            if (item != null && item.rb != null) mass += item.rb.mass;
        rb.AddForce(Vector3.up * (Cfg.StairsUpAssist.Value * mass * -Physics.gravity.y));
    }

    private void UpdateProgress()
    {
        var pos = PathPlanner.Flat(transform.position);
        float best = float.MaxValue;
        int last = Mathf.Min(seg + 10, path.Count - 1); // forward window only, so self-overlapping paths don't jump back
        for (int i = seg; i < last; i++)
        {
            Vector3 a = PathPlanner.Flat(path[i].Pos), b = PathPlanner.Flat(path[i + 1].Pos);
            var ab = b - a;
            float t = ab.sqrMagnitude > 0f ? Mathf.Clamp01(Vector3.Dot(pos - a, ab) / ab.sqrMagnitude) : 0f;
            float d = (a + ab * t - pos).magnitude;
            if (d < best) { best = d; seg = i; segT = t; }
        }
        offPath = best;
    }

    /// <summary>Point on the path <paramref name="dist"/> meters (horizontal) past the cart's closest point.</summary>
    private Vector3 PointAhead(float dist)
    {
        var p = Vector3.Lerp(path[seg].Pos, path[seg + 1].Pos, segT);
        for (int i = seg + 1; i < path.Count; i++)
        {
            var end = path[i].Pos;
            float l = PathPlanner.Flat(end - p).magnitude;
            if (l >= dist && l > 0f) return Vector3.Lerp(p, end, dist / l);
            dist -= l;
            p = end;
        }
        return path[path.Count - 1].Pos;
    }

    private float RemainingLength()
    {
        float len = PathPlanner.Flat(path[seg + 1].Pos - transform.position).magnitude;
        for (int i = seg + 1; i < path.Count - 1; i++) len += PathPlanner.Flat(path[i + 1].Pos - path[i].Pos).magnitude;
        return len;
    }

    private static float PathLength(List<Waypoint> p)
    {
        float len = 0f;
        for (int i = 1; i < p.Count; i++) len += PathPlanner.Flat(p[i].Pos - p[i - 1].Pos).magnitude;
        return len;
    }

    // Holes up to this wide get crossed: the front reaches the far side before the middle leaves the near one.
    private float MaxGap => Shape.HalfLength * 0.9f;

    private bool StairsNear(float ahead) => AnyAhead(ahead, w => w.Stairs);
    private bool DoorNear(float ahead) => AnyAhead(ahead, w => w.Door);

    private bool AnyAhead(float ahead, System.Func<Waypoint, bool> pred)
    {
        var pos = PathPlanner.Flat(transform.position);
        for (int i = seg; i < path.Count; i++)
        {
            if (PathPlanner.Flat(path[i].Pos - pos).magnitude > ahead && i > seg + 1) break;
            if (pred(path[i])) return true;
        }
        return false;
    }

    /// <summary>Turns a destination into a world goal. <paramref name="follow"/> is set when the goal moves (a player).</summary>
    private bool ResolveDestination(Destination dest, Vector3 playerPos, out Vector3 goal, out Transform follow)
    {
        goal = default;
        follow = null;
        switch (dest)
        {
            case Destination.Player:
                var player = NearestPlayer(playerPos);
                if (player == null) return false;
                follow = player.transform;
                goal = follow.position;
                return true;

            case Destination.Extraction:
            case Destination.ExtractionOpen:
                var ep = PickExtraction(playerPos);
                if (ep == null) return false;
                // Self-Driving Cart parks 4 m along the extraction's forward; that side is the open front.
                goal = ep.transform.position + ep.transform.forward * 4f;
                extraction = ep;
                extractionStayOutside = dest == Destination.ExtractionOpen;
                return true;

            case Destination.Truck:
                var t = TruckTransform();
                if (t == null) return false;
                goal = t.position;
                return true;
        }
        return false;
    }

    private static PlayerAvatar NearestPlayer(Vector3 pos)
    {
        PlayerAvatar best = null;
        float bestDist = float.MaxValue;
        foreach (var p in SemiFunc.PlayerGetAll())
        {
            if (p == null || p.isDisabled) continue;
            float d = (p.transform.position - pos).sqrMagnitude;
            if (d < bestDist) { bestDist = d; best = p; }
        }
        return best;
    }

    // The game's failsafe teleport spot for players is on the truck floor.
    private static Transform TruckTransform() =>
        TruckSafetySpawnPoint.instance != null ? TruckSafetySpawnPoint.instance.transform
        : TruckHealer.instance != null ? TruckHealer.instance.transform : null;

    private ExtractionPoint PickExtraction(Vector3 playerPos)
    {
        var rd = RoundDirector.instance;
        if (rd == null) return null;

        var truckT = TruckTransform();
        bool furthestFromTruck = Cfg.ExtractionFurthestFromTruck.Value && truckT != null;
        Vector3 truck = furthestFromTruck ? truckT.position : default;
        if (Cfg.ExtractionFurthestFromTruck.Value && truckT == null) Plugin.Log.LogWarning("No truck found; picking extraction by player distance.");
        Vector3 reference = Cfg.ExtractionClosestToCart.Value ? transform.position : playerPos;

        ExtractionPoint best = null;
        float bestScore = float.MaxValue;
        foreach (var go in rd.extractionPointList)
        {
            if (go == null) continue;
            var ep = go.GetComponent<ExtractionPoint>();
            if (ep == null) continue;

            // Only one extraction can run at a time; if one is active, that's the one.
            if (ep.currentState == ExtractionPoint.State.Active) return ep;
            if (ep.currentState != ExtractionPoint.State.Idle || ep.isLocked) continue;

            float score = furthestFromTruck
                ? -(ep.transform.position - truck).sqrMagnitude
                : (ep.transform.position - reference).sqrMagnitude;
            if (score < bestScore) { bestScore = score; best = ep; }
        }
        return best;
    }
}
