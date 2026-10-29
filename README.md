# Cart Pilot

BepInEx mod for R.E.P.O. The cart drives itself to wherever you tell it.

- F - to you
- G - to extraction, opens it, drives in
- H - back to the truck
- V - opens extraction, stays outside
- B - switch which cart you control (shows the money in each)

Keys and speeds are in `BepInEx/config/neo.cartpilot.cfg`. Logs go to `BepInEx/CartPilotLogs`.

## Install

Easiest: install it from Thunderstore with a mod manager (r2modman or Thunderstore Mod Manager). It installs BepInEx for you.

Manual: install [BepInExPack](https://thunderstore.io/c/repo/p/BepInEx/BepInExPack/) 5.4.2305 into the game folder, start the game once, then put `CartPilot.dll` in `BepInEx/plugins`.

## Build

`dotnet build` (see `CartPilot.csproj` for game reference paths).

Feedback and bugs: https://discord.gg/gNum2WsuFZ
