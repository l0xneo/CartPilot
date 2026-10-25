using System;
using System.IO;
using System.Text;
using BepInEx;
using BepInEx.Logging;
using UnityEngine;

namespace CartPilot;

/// <summary>
/// Diagnostic log for bug hunting: one new file per game launch in BepInEx/CartPilotLogs (never overwritten),
/// with every CartPilot log message plus cart snapshots (level, room, size, cargo, nearby obstacles) at each event.
/// </summary>
internal static class DiagLog
{
    private static StreamWriter writer;
    internal static string FilePath;

    internal static void Init()
    {
        try
        {
            var dir = Path.Combine(Paths.BepInExRootPath, "CartPilotLogs");
            Directory.CreateDirectory(dir);
            FilePath = Path.Combine(dir, $"session_{DateTime.Now:yyyy-MM-dd_HH-mm-ss}.log");
            writer = new StreamWriter(FilePath, true, Encoding.UTF8) { AutoFlush = true }; // flushed per line so a crash loses nothing
            Write("session", $"{Plugin.Name} {Plugin.Version} started");
            BepInEx.Logging.Logger.Listeners.Add(new Listener());
        }
        catch (Exception e)
        {
            Plugin.Log.LogError($"Diagnostic log disabled: {e.Message}");
        }
    }

    internal static void Write(string evt, string text)
    {
        if (writer == null) return;
        try { writer.WriteLine($"{DateTime.Now:HH:mm:ss.fff}\t{Time.time:F2}\t{Level()}\t{evt}\t{text}"); }
        catch { /* never let logging break the game */ }
    }

    private static string Level()
    {
        var rm = RunManager.instance;
        if (rm == null || rm.levelCurrent == null) return "level=?";
        return $"level={rm.levelCurrent.name}#{rm.levelsCompleted}";
    }

    /// <summary>Mirrors everything Plugin.Log prints (warnings, planner rejects, blocked areas) into the file.</summary>
    private class Listener : ILogListener
    {
        public void LogEvent(object sender, LogEventArgs e)
        {
            if (e.Source == Plugin.Log) Write(e.Level == LogLevel.Info ? "log" : e.Level.ToString().ToLower(), e.Data?.ToString());
        }

        public void Dispose() { }
    }
}
