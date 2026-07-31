# Isaac Sim + ROS 2 Stack — Design & Setup Notes

Working design doc for the white-crash simulation + training environment. Captures
the decisions made so far and *why*, so future-me (and any agent working in this
repo) has the reasoning, not just the conclusions.

_Last updated: 2026-07-28. Version facts verified against current docs on that date —
re-verify tags/versions before acting, they move. (2026-07-27 entry's "6.0 is dev-only"
call was already wrong by the next day — 6.0.1 had gone GA. Don't trust a version claim
here without re-checking if it's more than a few days old.)_

---

## 1. Goal

- Bring **white-crash** (tracked skid-steer tank) into simulation as the platform
  for the RSSC Clean-The-Room (Can-Do 2) contest.
- Primary aim is not winning — it's a **reusable, photorealistic Isaac sim** that
  serves double duty:
  - a base for future vision-based robots, and
  - a virtual-FPV environment for behavioral-cloning training sessions.
- Longer term: RMA-style **online system identification** (motor strength, actuation
  lag, mass, floor friction) fed back into the policy at runtime.

## 2. Architecture — the short version

Two containers, orchestrated with **docker-compose**, talking over **DDS**:

```
┌─────────────────────────────┐        DDS         ┌──────────────────────────────┐
│ Container A: Isaac Sim 6.0.1 │  (Fast DDS, same   │  Container B: my ROS 2 code   │
│  sim-only                    │   ROS_DOMAIN_ID)   │  ROS 2 Lyrical Luth (26.04)   │
│  internal ROS bridge = Jazzy │ <───────────────>  │  white-crash nodes / app logic│
└─────────────────────────────┘                    └──────────────────────────────┘
```

The whole point of the split: **it decouples my application ROS distro from whatever
Isaac supports internally.** Isaac Sim 6.0.1's bridge officially validates Humble/Jazzy
only; it experimentally supports any natively-installed distro, but only on Ubuntu
22.04/24.04 — Lyrical's 26.04 base isn't covered even by that. I want my code on the
newer Lyrical LTS regardless. DDS is the seam that lets those two coexist.

## 3. Decisions & rationale

### Container route (not pip/binary)
Container is the reproducible path and matches how NVIDIA ships everything. The
pip/conda route is the fragile one (exact Python 3.11, torch pinning). A committed
compose file *is* the reproducible base the project goal calls for.

### docker-compose (not hand-rolled `docker run`)
- Two containers with fiddly shared state (cache mounts, `ROS_DOMAIN_ID`, matching
  RMW, `--network=host`) — as `docker run` flags that's an error-prone wall; as
  compose it's a readable block set once.
- NVIDIA drives its own Isaac Sim docker tooling and Isaac Lab deployment through
  compose, so this swims with the current.
- Caveat: GPU passthrough + rootless (default since 5.1) + host networking is where
  compose files drift between versions — **adapt NVIDIA's maintained file, don't
  author from scratch or paste blog YAML.** Did this: sparse-checked out
  `tools/docker/` from `github.com/isaac-sim/IsaacSim` (main branch) into `docker/` in
  this repo, rather than hand-copying YAML from a docs page — the docs page doesn't
  even inline the compose file, just points at the repo.
- `docker compose` (v2 plugin) isn't in Docker's own apt repo if Docker itself came
  from Ubuntu's `docker.io` package (it did, here) rather than `docker-ce` from
  `download.docker.com`. Ubuntu 24.04 ships its own compose-v2 build as
  `docker-compose-v2` — use that instead of chasing Docker's official repo setup.

### Isaac Sim = sim-only, bridge stays on Jazzy
Isaac Sim 6.0.1's ROS 2 bridge is validated against Humble and Jazzy only (confirmed:
container logs show it loading internal rclpy for `jazzy`). I don't run Lyrical
*inside* Isaac — Isaac just needs to emit/consume topics. Its internal bridge stays
Jazzy; that's fine because my nodes live in Container B.

### Application code = ROS 2 Lyrical Luth
- Newer LTS → 5-year support runway (to May 2031).
- Runs on its own Ubuntu 26.04 base, isolated from Isaac's 24.04 base (containers
  isolate the userland — no conflict).
- **This rules out Isaac Lab's built-in `ros2` image extension for app code** — that
  extension is Humble-based, two LTS generations behind where I want to be.

### Bridging: DDS at the wire level
- Interop is RTPS (wire) level, not ROS-distro level.
- Keep **Fast DDS on both sides** (it's the default on both Jazzy and Lyrical — free)
  + matching `ROS_DOMAIN_ID`.
- **Standard messages** (`sensor_msgs`, `geometry_msgs`, `nav_msgs`, …) interoperate
  across distros because the type is defined by message content.
- **Custom messages** must be defined identically on both sides (share the interface
  package so type hashes line up).
- Cross-distro `ros2 topic echo` may print type-hash *warnings*; for standard types
  data still flows — cosmetic, don't chase it as a bug.

### GUI: WebRTC browser streaming, not X11
This was wrong in the original plan — corrected after actually standing up the
container. NVIDIA's *current* containerized workflow for standalone Isaac Sim (not
Isaac Lab) has no X11 path at all; their `tools/docker/docker-compose.yml` runs
`isaac-sim` headless and pairs it with a `web-viewer` sidecar service that streams the
GUI over WebRTC to a browser tab (`http://localhost:8210`). That's what we're running.
- Works fine for authoring/debugging white-crash's scene & articulation — full
  interactive GUI, just via browser instead of a native window. Image is a bit soft at
  default bitrate; adjustable later if it matters, not a blocker.
- The prebuilt NGC image (`nvcr.io/nvidia/isaac-sim:6.0.1`) works directly with this
  compose file via `ISAAC_SIM_IMAGE=...` in `.env` — no local Isaac Sim source build
  needed for this use case.
- Isaac Lab's own X11-capable `container.py start` path (self-built image) may still be
  relevant *later* for RMA/BC training tooling specifically — untested, revisit then.

### Tracked-robot modeling (physics)
Isaac Sim / PhysX has **no native track primitive** (unlike Gazebo's TrackedVehicle).
Still no official solution as of mid-2026. For a flat-floor corral-and-push task:
- Use a **skid-steer approximation** — model each side as a *row of hidden wheels*
  along the track footprint (the long contact patch reproduces turn-in-place scrub
  far better than two point contacts).
- **Randomize turning/lateral friction and left-right drive-gain mismatch
  aggressively** so the real track's slip falls inside the training distribution.
- The **RMA online-adaptation plan absorbs the tracked-vs-wheeled gap** — don't model
  tracks faithfully; randomize hard and adapt at runtime. Feed effective
  yaw-rate-per-command-differential into the online estimator as a slip proxy.
- Watch turn-in-place specifically: a 2-wheel diff-drive pivots ~frictionlessly; a
  real tracked tank scrubs (more torque to rotate, floor-friction-dependent). That's
  the dynamic the policy will over-trust if the sim is too clean.

## 4. Verified facts / versions (2026-07-28)

| Thing | Value | Notes |
|---|---|---|
| Isaac Sim stable | **6.0.1 GA** | 5.1.0 (yesterday's pin) is superseded; 6.0.0 GA also exists, 6.0.1 is newer |
| Isaac Sim Python | **3.11 only** | don't source system ROS into Isaac's terminal |
| Isaac Sim ROS bridge | **Humble / Jazzy** officially; experimental native-distro loading on **Ubuntu 22.04/24.04 only** | still no Lyrical (26.04) coverage, even experimentally |
| Isaac Sim container GUI | **WebRTC browser streaming** (`web-viewer` sidecar, port 8210) | no X11 path in the standalone-container docs at all |
| Rootless containers | default since **5.1** | old 4.x compose files → permission errors; container user is uid **1234** |
| Isaac Lab | 2.3.x | `ros2` extension = Humble; prebuilt NGC = headless-only; X11 path untested here |
| ROS 2 Lyrical Luth | released **2026-05-22**, LTS to **May 2031** | Ubuntu 26.04 "Resolute" Tier 1; Patch Release 1 = 2026-06-23 |
| Lyrical default RMW | **Fast DDS** (rmw_fastrtps_cpp) | unchanged from Jazzy/Kilted |
| Docker on this host | `docker.io` 29.1.3 (Ubuntu package, not `docker-ce`) | compose v2 via Ubuntu's `docker-compose-v2` package |
| NVIDIA driver / GPU | 580.159.03, RTX 4090 | GPU passthrough verified working |

## 5. Setup sequence (high level)

1. **Host prereqs** ✅ — driver 580.159.03, `docker-compose-v2` + NVIDIA Container
   Toolkit installed via `docker/setup-host.sh` (idempotent, re-run is safe). GPU
   passthrough verified with a throwaway `nvidia-smi` container.
2. **NGC access** ✅ — API key generated, `docker login nvcr.io --username '$oauthtoken'`
   done. (Ran interactively so the key never sat in shell history/chat logs — worth
   keeping that habit; also worth rotating any key that *did* get pasted somewhere it
   shouldn't have.)
3. **Container A (Isaac Sim)** ✅ — running. `docker/` in this repo is a sparse
   checkout of `tools/docker/` from the IsaacSim GitHub repo; `docker/.env` pins
   `ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.1` (prebuilt, no local build) and
   points cache/data mounts at `~/docker/isaac-sim` (uid 1234). Brought up with
   `docker compose -p isim up --build -d` from `docker/` (the `--build` is only for the
   lightweight `web-viewer` sidecar, not Isaac Sim itself). GUI confirmed working at
   `http://localhost:8210`.
4. **Container B (Lyrical)** ✅ — running. `docker/lyrical/Dockerfile` builds
   `ros:lyrical-ros-base-resolute` (official image, confirmed on Docker Hub — resolved
   the hand-rolled-vs-official open decision in favor of official) plus `ros-dev-tools`
   for colcon/rosdep. `white_crash_ws/src` on the host mounts to `/workspace` in the
   container — that's where white-crash packages will live once written. Interactive
   shells (`docker compose -p isim exec -it lyrical bash`) auto-source ROS via
   `/etc/bash.bashrc` (note: only works for *interactive* `exec`, not `bash -lc`, since
   `docker exec` skips the image's own entrypoint script either way).
5. **Wire over DDS** ✅ — verified with a throwaway `ros:jazzy-ros-base-noble`
   container standing in for Isaac's internal Jazzy bridge (the real bridge only runs
   inside a live Isaac Sim scene via Action Graph, not invokable from a plain shell —
   no `ros2` CLI in the isaac-sim container, `rclpy` is bundled per-distro under
   `/isaac-sim/exts/isaacsim.ros2.core/{jazzy,humble}/` for internal use only). Probe
   container published `/handshake` (`std_msgs/String`) on host network, domain 0;
   lyrical container's `ros2 topic list`/`topic echo` saw it and received the actual
   message content. No type-hash warnings even appeared. Confirms the cross-distro
   DDS mechanism the whole architecture depends on. Probe container removed after.
   **Still open:** wiring the *real* Isaac Sim ROS2 bridge (Action Graph, once
   white-crash's scene exists) — that's scene-building work, not infra verification.
6. **Later: Isaac Lab** — revisit X11-vs-WebRTC question for Lab specifically when
   moving to RMA/BC training; not needed for scene authoring, which is covered by
   Container A's WebRTC GUI now.

## 6. Open decisions

- **Lyrical container**: hand-rolled vs. official Lyrical apt-on-26.04 base.
- **Lab timing**: plain Isaac Sim compose now (fold Lab in at training) vs. adopt
  Lab's compose base now. Leaning plain-Isaac-first since I want the GUI for
  authoring the track model before any training — and that's now confirmed to work
  without Lab at all, via the WebRTC web-viewer.
- ~~Which compose file to adapt~~ — resolved: `tools/docker/docker-compose.yml` from
  `github.com/isaac-sim/IsaacSim` (main branch), sparse-checked into `docker/` here.

## 7. Reference links

- Isaac Sim container install — https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html
- Isaac Sim ROS 2 landing — https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/ros2_landing_page.html
- Isaac Sim ROS 2 install — https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html
- Isaac Sim release notes (GA status per version) — https://docs.isaacsim.omniverse.nvidia.com/latest/overview/release_notes.html
- Isaac Sim `tools/docker/` (compose file + web-viewer, what we're actually running) — https://github.com/isaac-sim/IsaacSim/tree/main/tools/docker
- Isaac Sim ROS workspaces repo — https://github.com/isaac-sim/IsaacSim-ros_workspaces
- Isaac Lab Docker guide — https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html
- NVIDIA Container Toolkit install guide — https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
- ROS 2 Lyrical Luth release notes — https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html

## 8. What's actually running right now

- `docker/` in this repo = sparse checkout of `IsaacSim/tools/docker` (main branch).
- `docker/.env` — image pin + host paths, not committed-secret-bearing, safe to keep.
- `docker/setup-host.sh` — one-shot host prereq installer (compose plugin, NVIDIA
  Container Toolkit, GPU check, cache dirs). Re-run is safe if a fresh machine needs it.
- `docker compose -p isim ps` → `isim-isaac-sim-1` and `isim-web-viewer-1`, both
  healthy. GUI at `http://localhost:8210`.
- Cache/data persisted at `~/docker/isaac-sim/` and `~/.cache/ov/hub/` (uid 1234), so a
  `docker compose down` + `up` won't lose shader cache / config.
- `docker compose -p isim ps` → `isim-isaac-sim-1`, `isim-web-viewer-1`,
  `isim-lyrical-1`, all up. `white_crash_ws/src` at the project root is the
  (currently-empty) ROS 2 workspace for white-crash packages.
