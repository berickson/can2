# Isaac Sim + ROS 2 Stack — white-crash

Sim + training environment for **white-crash** (tracked skid-steer tank), built for
the RSSC Clean-The-Room (Can-Do 2) contest. Not really about winning the contest — the
real goal is a reusable, photorealistic Isaac Sim setup that doubles as a base for
future vision-based robots and a virtual-FPV environment for behavioral-cloning
training. Longer term: RMA-style online system identification (motor strength,
actuation lag, mass, floor friction) fed back into the policy at runtime.

## Architecture

Two Docker containers over DDS:

```
┌─────────────────────────────┐        DDS         ┌──────────────────────────────┐
│ Container A: Isaac Sim 6.0.1 │  (Fast DDS, same   │  Container B: my ROS 2 code   │
│  sim-only, network_mode=host │   ROS_DOMAIN_ID)   │  ROS 2 Lyrical Luth (26.04)   │
│  internal ROS bridge = Jazzy │ <───────────────>  │  white-crash nodes / app logic│
└─────────────────────────────┘                    └──────────────────────────────┘
```

Isaac Sim's ROS2 bridge only supports Humble/Jazzy (officially), and experimentally
supports natively-installed distros only on Ubuntu 22.04/24.04 — not Lyrical's 26.04.
Bridging at the DDS/RTPS wire level (matching `ROS_DOMAIN_ID` + Fast DDS both sides)
decouples our app code's ROS distro from whatever Isaac supports internally, so we can
run app code on newer Lyrical regardless.

## Current status

Both containers running and verified talking to each other over real ROS2 topics
(Isaac Sim's Action Graph → `/clock` → seen live in the lyrical container).

- **`docker/`** — sparse checkout of `IsaacSim/tools/docker` (NVIDIA's maintained
  compose file), plus our own `docker/lyrical/` service. `docker/.env.example` →
  copy to `docker/.env` (gitignored, host-specific paths) to configure.
- **`docker/setup-host.sh`** — one-shot host prereq installer (compose plugin, NVIDIA
  Container Toolkit, GPU check, cache dirs). Safe to re-run.
- Bring up: `cd docker && docker compose -p isim up --build -d`
- GUI: `http://localhost:8210` (WebRTC stream — Isaac's standalone container has no
  X11 path, only browser streaming)
- Shell into app container: `docker compose -p isim exec -it lyrical bash`
  (ROS auto-sourced)
- `white_crash_ws/src` — empty so far, this is where white-crash ROS2 packages go
- `scenes/` — mounted at `/isaac-sim/scenes` in the container. Save stage files here
  (File → Save As) so scene state survives container restarts — the running stage is
  otherwise in-memory only and gets wiped on any `docker compose up` that recreates
  isaac-sim.

**Gotchas worth remembering:**
- The isaac-sim container has no standalone `ros2` CLI — its ROS2 bridge only runs
  inside the live Kit process. To test it, build an Action Graph via the GUI's Script
  Editor (Window → Script Editor), not a container shell.
- Isaac Sim node types are `isaacsim.ros2.bridge.*` / `isaacsim.core.nodes.*` — NVIDIA's
  own tutorial docs still reference the older `omni.isaac.ros2_bridge.*` names in
  places; trust the extension source under `/isaac-sim/exts/` over the docs if unsure.
- Cross-distro `ros2 topic echo` may print type-hash warnings for custom message types;
  cosmetic if data is still flowing.
- Blurry/fixed-resolution viewport in the browser? It's a **native Kit viewport
  setting**, not a WebRTC/compose thing — gear icon on the viewport toolbar → Render
  Resolution → check **Fill Viewport** (or pick a fixed preset like HD1080P/2K/UHD).
  Default is HD720P. Don't waste time on `--/app/renderer/resolution/*`,
  `--/app/livestream/allowResize`, or similar compose-level flags — tried all of those,
  none of them touch this; the viewport resolution is unrelated to the WebRTC stream
  negotiation entirely.

## Next up

- Bring white-crash's actual model into Isaac Sim (scene/articulation), then wire its
  real ROS2 topics (not just the `/clock` test graph).
- No native track primitive in PhysX — plan is a skid-steer approximation (row of
  hidden wheels per side) with aggressive friction/drive-gain randomization, relying on
  RMA online adaptation to close the tracked-vs-wheeled gap rather than modeling tracks
  faithfully.
- Isaac Lab (RMA/BC training tooling) — not needed yet, revisit once past scene
  authoring.
- Lyrical container is on the official `ros:lyrical-ros-base-resolute` image.

## Reference

- Isaac Sim docs — https://docs.isaacsim.omniverse.nvidia.com/latest/
- Isaac Sim `tools/docker` (what we're running) — https://github.com/isaac-sim/IsaacSim/tree/main/tools/docker
- ROS 2 Lyrical Luth release notes — https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html
