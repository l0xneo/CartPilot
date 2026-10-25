using HarmonyLib;
using UnityEngine;
using UnityEngine.InputSystem;

namespace CartPilot;

/// <summary>Reads destination keys for the local player (same hook as Self-Driving Cart).</summary>
[HarmonyPatch(typeof(PlayerController), "Update")]
internal static class PlayerControllerUpdatePatch
{
    private static void Postfix(PlayerController __instance)
    {
        var kb = Keyboard.current;
        if (kb == null || ChatOpen()) return;

        foreach (var (key, dest) in Cfg.DestinationKeys)
        {
            if (key.Value == Key.None || !kb[key.Value].wasPressedThisFrame) continue;

            var cart = CartPilotController.Nearest(__instance.transform.position);
            if (cart == null)
            {
                Plugin.Log.LogInfo("No cart to command.");
                return;
            }
            cart.SendCommand(dest, __instance.transform.position);
        }
    }

    private static bool ChatOpen() =>
        ChatManager.instance != null && ChatManager.instance.chatState == ChatManager.ChatState.Active;
}

/// <summary>Guns misfire on impacts; riding in a self-driving cart shouldn't count.</summary>
[HarmonyPatch(typeof(ItemGun), nameof(ItemGun.Misfire))]
internal static class ItemGunMisfirePatch
{
    private static bool Prefix(ItemGun __instance) => !CartPilotController.CarryingWhileDriving(__instance.transform.position);
}

/// <summary>Attaches the controller to every big cart on every client (needed so the RPC exists everywhere).</summary>
[HarmonyPatch(typeof(PhysGrabCart), "Start")]
internal static class PhysGrabCartStartPatch
{
    private static void Postfix(PhysGrabCart __instance)
    {
        // shortcut: small carts are skipped (like Self-Driving Cart) since the cart NavMesh is baked for one radius; add a second bake if small carts are wanted.
        if (__instance.isSmallCart || __instance.GetComponent<CartPilotController>() != null) return;
        __instance.gameObject.AddComponent<CartPilotController>();
    }
}
