using System.Collections.Generic;
using BepInEx;
using BepInEx.Configuration;
using BepInEx.Logging;
using HarmonyLib;
using UnityEngine.InputSystem;

namespace CartPilot;

[BepInPlugin(Guid, Name, Version)]
public class Plugin : BaseUnityPlugin
{
    public const string Guid = "neo.cartpilot";
    public const string Name = "Cart Pilot";
    public const string Version = "0.2.9";

    internal static ManualLogSource Log;

    private void Awake()
    {
        Log = Logger;
        DiagLog.Init();
        Cfg.Bind(Config);
        new Harmony(Guid).PatchAll();
        Log.LogInfo($"{Name} {Version} loaded.");
    }
}

/// <summary>Where a key press sends the cart. Add a value here, a key in <see cref="Cfg"/>, and a case in CartPilotController.ResolveDestination.</summary>
internal enum Destination
{
    Player = 0,
    Extraction = 1,
    Truck = 2,
    ExtractionOpen = 3,
}

internal static class Cfg
{
    // Destination keys, iterated by the input patch.
    internal static readonly List<(ConfigEntry<Key> key, Destination dest)> DestinationKeys = new();
    internal static ConfigEntry<Key> SelectCartKey;

    // Extraction choice
    internal static ConfigEntry<bool> ExtractionFurthestFromTruck;
    internal static ConfigEntry<bool> ExtractionClosestToCart;

    // Cart size / clearance
    internal static ConfigEntry<float> CartRadius;
    internal static ConfigEntry<float> ClearanceMargin;
    internal static ConfigEntry<float> PreferredExtraClearance;

    // Driving
    internal static ConfigEntry<float> MaxSpeed;
    internal static ConfigEntry<float> MinSpeed;
    internal static ConfigEntry<float> CornerSpeed;
    internal static ConfigEntry<float> StairsSpeed;
    internal static ConfigEntry<float> DoorSpeed;
    internal static ConfigEntry<float> Lookahead;
    internal static ConfigEntry<float> StairsLookahead;
    internal static ConfigEntry<float> StairsUpAssist;

    // Obstacles
    internal static ConfigEntry<float> BlockedReplanTime;
    internal static ConfigEntry<float> ClearTimeBeforeResume;
    internal static ConfigEntry<float> StuckTimeout;
    internal static ConfigEntry<float> BlockedAreaLifetime;

    // Debug
    internal static ConfigEntry<bool> HudEnabled;
    internal static ConfigEntry<float> HudX;
    internal static ConfigEntry<float> HudY;
    internal static ConfigEntry<float> HudLength;
    internal static ConfigEntry<float> HudScale;

    internal static ConfigEntry<bool> DebugDraw;
    internal static ConfigEntry<bool> VerboseLog;

    internal static void Bind(ConfigFile cfg)
    {
        DestinationKeys.Add((cfg.Bind("Keys", "ComeToPlayer", Key.F, "Cart drives to you."), Destination.Player));
        DestinationKeys.Add((cfg.Bind("Keys", "GoToExtraction", Key.G, "Cart drives to the next extraction point, presses its button and drives inside."), Destination.Extraction));
        DestinationKeys.Add((cfg.Bind("Keys", "GoToTruck", Key.H, "Cart drives back inside the truck."), Destination.Truck));
        DestinationKeys.Add((cfg.Bind("Keys", "OpenExtraction", Key.V, "Cart drives to the next extraction point and presses its button, but stays outside."), Destination.ExtractionOpen));
        SelectCartKey = cfg.Bind("Keys", "SelectCart", Key.B, "Switch which cart your keys command. Shows every cart and the money inside it. Until you pick one, the nearest cart is used.");

        ExtractionFurthestFromTruck = cfg.Bind("Extraction", "FurthestFromTruck", true,
            "Go to the unfinished extraction point furthest from the truck. Wins over ClosestToCart if both are on. An already active extraction always takes priority.");
        ExtractionClosestToCart = cfg.Bind("Extraction", "ClosestToCart", false,
            "Go to the unfinished extraction point closest to the cart. (Both off = closest to the player who pressed the key.)");

        CartRadius = cfg.Bind("Cart", "CartRadius", 0f, "Cart half-width in meters. 0 = measure from the cart's colliders.");
        ClearanceMargin = cfg.Bind("Cart", "ClearanceMargin", 0.15f, "Extra space kept between cart side and walls (added to the half-width when baking the cart NavMesh).");
        PreferredExtraClearance = cfg.Bind("Cart", "PreferredExtraClearance", 0.35f, "Routes with this much more clearance are preferred; falls back to the tight route if none exists.");

        MaxSpeed = cfg.Bind("Driving", "MaxSpeed", 4.5f, "Top speed (m/s). The game caps a pulled cart at 5.");
        MinSpeed = cfg.Bind("Driving", "MinSpeed", 0.8f, "Lowest cruising speed in tight spaces.");
        CornerSpeed = cfg.Bind("Driving", "CornerSpeed", 3f, "Speed through a 90 degree corner (gentler turns are faster). Raise for quicker corners, lower if it clips walls.");
        StairsSpeed = cfg.Bind("Driving", "StairsSpeed", 2f, "Speed on and right before stairs.");
        DoorSpeed = cfg.Bind("Driving", "DoorSpeed", 2.5f, "Speed near doors (the cart smashes doors in its way).");
        Lookahead = cfg.Bind("Driving", "Lookahead", 1.6f, "Pure-pursuit lookahead distance at full speed (m).");
        StairsLookahead = cfg.Bind("Driving", "StairsLookahead", 0.6f, "Lookahead on stairs. Short = follows the stair centerline strictly.");
        StairsUpAssist = cfg.Bind("Driving", "StairsUpAssist", 1f, "Fraction of cart+cargo weight countered while driving (0 = none, 1 = weightless). Height follows the path line either way.");

        BlockedReplanTime = cfg.Bind("Obstacles", "BlockedReplanTime", 2f, "Seconds the path (anywhere ahead, not just in front of the cart) stays blocked before the cart picks a new route.");
        ClearTimeBeforeResume = cfg.Bind("Obstacles", "ClearTimeBeforeResume", 1f, "Path must be clear this long before the cart moves again.");
        StuckTimeout = cfg.Bind("Obstacles", "StuckTimeout", 2.5f, "If the cart barely moves for this long while driving, it backs up and replans.");
        BlockedAreaLifetime = cfg.Bind("Obstacles", "BlockedAreaLifetime", 25f, "Seconds a blocked area is avoided by the planner.");

        HudEnabled = cfg.Bind("HUD", "ShowProgressLine", true, "Show a small line with the cart moving toward its target on the screen of whoever sent it.");
        HudX = cfg.Bind("HUD", "PositionX", 0.5f, "Horizontal center of the line, 0 = left edge, 1 = right edge.");
        HudY = cfg.Bind("HUD", "PositionY", 0.08f, "Vertical position of the line, 0 = top, 1 = bottom.");
        HudLength = cfg.Bind("HUD", "Length", 260f, "Line length in pixels (at 1080p, before Scale).");
        HudScale = cfg.Bind("HUD", "Scale", 1f, "Size of the whole thing.");

        DebugDraw = cfg.Bind("Debug", "DrawPath", false, "Draw the planned path (host only). Green = normal, yellow = stairs, red = chokepoint.");
        VerboseLog = cfg.Bind("Debug", "VerboseLog", false, "Log state changes and planner details.");
    }
}
