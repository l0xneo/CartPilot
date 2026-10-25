using UnityEngine;
using Photon.Pun;

namespace CartPilot;

/// <summary>Progress bar on the commander's screen: a dot slides from start to target, with the distance left.</summary>
internal partial class CartPilotController
{
    private enum HudState : byte { Hidden, Driving, Arrived, NoPath }

    // Host side
    private float hudTotal = -1f; // longest remaining length seen since the command (the bar's full length)
    private float hudRemaining;
    private float hudSendTime;

    // Commander side
    private HudState hud;
    private float hudProgress, hudShown, hudMeters;
    private float hudUntil;

    private void HudReset() { hudTotal = -1f; hudRemaining = 0f; }

    private void HudSend(HudState s)
    {
        float progress = s == HudState.Arrived ? 1f : hudTotal > 0f ? 1f - hudRemaining / hudTotal : 0f;
        float meters = s == HudState.Arrived ? 0f : hudRemaining;
        if (commander == null || commander.IsLocal) CartPilotHudRPC((byte)s, progress, meters);
        else photonView.RPC(nameof(CartPilotHudRPC), commander, (byte)s, progress, meters);
    }

    /// <summary>Called every physics step while not idle; sends the progress a few times a second.</summary>
    private void HudTick()
    {
        if (path != null && state == State.Driving)
        {
            hudRemaining = RemainingLength();
            if (hudRemaining > hudTotal) hudTotal = hudRemaining;
        }
        if (Time.time < hudSendTime) return;
        hudSendTime = Time.time + 0.2f;
        HudSend(HudState.Driving);
    }

    [PunRPC]
    private void CartPilotHudRPC(byte s, float progress, float meters)
    {
        if (hud != HudState.Driving && (HudState)s == HudState.Driving) hudShown = 0f; // new trip starts at the left
        hud = (HudState)s;
        hudProgress = Mathf.Clamp01(progress);
        hudMeters = meters;
        // Driving updates keep coming; if they stop (host left) the bar goes away.
        hudUntil = Time.time + (hud == HudState.Driving ? 2f : 3f);
    }

    private static readonly Color HudPanel = new(0.05f, 0.05f, 0.07f, 0.55f);
    private static readonly Color HudTrack = new(1f, 1f, 1f, 0.15f);
    private static readonly Color HudCart = new(1f, 0.6f, 0.15f);
    private static readonly Color HudOk = new(0.35f, 0.85f, 0.45f);
    private static readonly Color HudFail = new(0.95f, 0.3f, 0.3f);
    private static GUIStyle hudLabel;

    private void OnGUI()
    {
        if (!Cfg.HudEnabled.Value || hud == HudState.Hidden) return;
        if (Time.time > hudUntil) { hud = HudState.Hidden; return; }
        if (Event.current.type != EventType.Repaint) return;

        // Several carts on the move: stack their bars.
        int row = 0;
        foreach (var c in All) { if (c == this) break; if (c.hud != HudState.Hidden) row++; }

        float goalProgress = hud == HudState.Arrived ? 1f : hudProgress;
        hudShown = Mathf.Lerp(hudShown, goalProgress, 1f - Mathf.Exp(-6f * Time.deltaTime));
        float s = Cfg.HudScale.Value * Screen.height / 1080f;
        float a = Mathf.Clamp01((hudUntil - Time.time) / 0.4f); // fade out at the end
        var accent = hud == HudState.Arrived ? HudOk : hud == HudState.NoPath ? HudFail : HudCart;

        float len = Cfg.HudLength.Value * s, textW = 70f * s, pad = 12f * s, h = 26f * s;
        float w = pad + len + 10f * s + textW;
        float x = Cfg.HudX.Value * Screen.width - w / 2f;
        float y = Cfg.HudY.Value * Screen.height + row * (h + 6f * s);
        float mid = y + h / 2f;

        Round(x, y, w, h, h / 2f, HudPanel, a);
        float tx = x + pad, th = 4f * s;
        Round(tx, mid - th / 2f, len, th, th / 2f, HudTrack, a);
        Round(tx, mid - th / 2f, Mathf.Max(th, hudShown * len), th, th / 2f, accent, a);

        // Cart marker: white ring around an accent dot.
        float d = 12f * s, cx = tx + hudShown * len, ring = 2f * s;
        Round(cx - d / 2f, mid - d / 2f, d, d, d / 2f, Color.white, a);
        Round(cx - d / 2f + ring, mid - d / 2f + ring, d - 2f * ring, d - 2f * ring, d / 2f, accent, a);

        string text = hud == HudState.Arrived ? "Arrived" : hud == HudState.NoPath ? "No path" : $"{Mathf.CeilToInt(hudMeters)} m";
        hudLabel ??= new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };
        hudLabel.fontSize = Mathf.RoundToInt(13f * s);
        var textColor = hud == HudState.Driving ? Color.white : accent;
        textColor.a = 0.9f * a;
        GUI.color = textColor;
        GUI.Label(new Rect(tx + len + 10f * s, y, textW, h), text, hudLabel);
        GUI.color = Color.white;
    }

    /// <summary>Filled rounded rectangle.</summary>
    private static void Round(float x, float y, float w, float h, float r, Color c, float alpha)
    {
        c.a *= alpha;
        GUI.DrawTexture(new Rect(x, y, w, h), Texture2D.whiteTexture, ScaleMode.StretchToFill, true, 0f, c, 0f, Mathf.Min(r, w / 2f, h / 2f));
    }
}
