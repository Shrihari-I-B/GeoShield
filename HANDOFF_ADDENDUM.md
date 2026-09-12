# GeoShield — Handoff Addendum

**Read this AFTER `HANDOFF_1.md` and `HANDOFF_2.md`. It supersedes parts of
both.** Two things changed materially since those documents were written.

---

# 1. AUTOWARE NOW RUNS LOCALLY IN DOCKER — EC2 IS OPTIONAL

## What changed

The whole stack now runs on the laptop. AWS is no longer required for
simulation.

**Hardware, correcting an earlier assumption:**

| | Value |
|---|---|
| CPU | Intel Core Ultra 5 125H, **18 threads** |
| GPU | **Intel Arc Graphics (MTL)** — not basic integrated |
| RAM | 16 GB total (~12 GB available with browsers closed) |
| Disk | 123 GB, ~72 GB free after the image |
| Windowing | **Wayland** (XWayland bridges X11 apps) |

Earlier sections assumed the laptop was too weak. It is not.

## The image

```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-devel
```

- Digest `sha256:405225eda6c05161bfde39cc7885511f3f4d9699d126891891420dd80c2e024a`
- Disk usage 11.1 GB, content size 2.62 GB
- Autoware **0.50.0**
- Setup script at `/opt/autoware/setup.bash`

## Verified working

| Item | Result |
|---|---|
| GPU passthrough | **OpenGL 4.6**, no libGL errors |
| Without `--device=/dev/dri` | falls back to software, OpenGL 4.5, `failed to load driver: iris` |
| RViz2 | opens, renders, **31 fps** |
| Node count | **135** (EC2 was 122) |
| RAM in use | ~5 GB used, **9 GB still available** |
| `/planning/trajectory` | **10.014 Hz**, std dev 0.004 s — matches the 10 Hz target |
| Nishi-Shinjuku point cloud | renders, `Global Status: Ok` |
| Full drive | pose → goal → Auto → `Motion: Moving`, vehicle drives the route |

**10.014 Hz is the important number.** The planner holds real-time on this CPU,
so the laptop fully replaces EC2 for simulation.

## Exact startup procedure

### Terminal 1 — Autoware + RViz

```bash
xhost +local:docker
```

```bash
docker run -it --rm --name autoware \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --group-add video \
  -v ~/autoware_map:/autoware_map \
  -v ~/Development/projects/geoshield:/geoshield \
  --net=host \
  ghcr.io/autowarefoundation/autoware:universe-devel \
  bash
```

Inside (prompt becomes `root@HP-Laptop-15-fd1xxx:/autoware#`):

```bash
source /opt/autoware/setup.bash
```

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/autoware_map/<MAPDIR> vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

**Note: no `rviz:=false`.** RViz is wanted here. Then leave this terminal alone.

### Terminal 2 — working shell inside the same container

```bash
docker exec -it autoware bash
source /opt/autoware/setup.bash
ros2 node list | wc -l          # expect ~135
```

`--name autoware` is required for this. `--rm` means exiting Terminal 1
destroys the container and Terminal 2 loses its target.

### Terminal 3 — laptop host, for GeoShield

```bash
cd ~/Development/projects/geoshield
```

Runs `differential_verify.py`, `attack_injector.py`, git.

## Mounts

| Host | Container |
|---|---|
| `~/autoware_map` | `/autoware_map` |
| `~/Development/projects/geoshield` | `/geoshield` |

All 22 scripts, `data/` and `results/` are visible at `/geoshield` inside the
container.

## Gotchas found

- **A second `docker run` creates a separate container** that cannot see the
  first one's ROS graph. Use `docker exec -it autoware bash`.
- **Same hostname on host and container** because of `--net=host`. Tell them
  apart by the user: `root@` is the container, `shrihari-i-b@` is the laptop.
- **`~/autoware_map/cl_map` on the laptop contained only `pointcloud_map.pcd`**
  — the `.osm` had only ever existed on EC2. A map directory needs both files
  or the launch shows `Global Status: Error` and an empty grid.
- Goal placed too near the start gives `Routing: Arrived` immediately. Place it
  200–300 m away.

---

# 2. THE CENTRELINE DoS FINDING DOES NOT REPRODUCE — CORRECTION

## What HANDOFF_2 says (§7.5, §8.2)

> Injecting a single explicit centreline — geometrically identical to
> Autoware's own computed centreline — into a 979-lanelet map prevents the
> planning stack from producing any trajectory. The vehicle is immobilised.

## What was actually observed

| Build | Map | Result |
|---|---|---|
| EC2, Autoware **0.52.0**, source-compiled, CUDA packages missing | `centerline_zero.osm` | no trajectory, 1 unique position, vehicle immobile |
| Docker, Autoware **0.50.0**, prebuilt image | `centerline_zero.osm` | **`Routing: Set`, `Motion: Moving`, vehicle drives normally** |

**The attack does not immobilise Autoware 0.50.0.**

## Correct statement

> Centreline injection produced no trajectory on Autoware 0.52.0 (EC2,
> source-compiled). The same map drives normally on Autoware 0.50.0 (official
> Docker image). The cause is unresolved and at least three variables differ:
> Autoware version, build configuration (source vs prebuilt, CUDA packages
> absent vs present), and host environment.

## What is unaffected

**Differential verification still catches centreline injection at precision
1.000, recall 1.000.** It detects the map modification, not its downstream
effect on the planner. That result stands regardless.

## Files that need editing

- `HANDOFF_2.md` §7.5 and §8.2
- `PHASE_PLAN.md`
- `REVIEW1_SLIDES.md`
- Git commit `f4e12d7` states the finding in its message — cannot be edited,
  should be superseded by a later commit

## To resolve it properly

Run the identical map on both builds with the readiness gate applied, varying
only the Autoware version. Both are still available. **This is a side quest —
not a priority.**

---

# 3. WHICH BUILD IS THE REFERENCE

**Recommendation: Autoware 0.50.0, the Docker image.**

- Local, no network dependency
- Pinned to a public digest — anyone can reproduce it
- GPU-accelerated RViz for live demonstration
- Planner holds 10.014 Hz

EC2 (0.52.0) becomes a secondary observation. Results must state which build
produced them; mixing the two without saying so is not defensible.

**[UNVERIFIED]** — no results have yet been produced on 0.50.0. Everything in
`RESULTS.md` came from 0.52.0.

---

# 4. IMMEDIATE NEXT STEPS — REVISED

## Priority 1 — Correct the record (2 hours, laptop)

1. `RESULTS.md` still quotes **1.414 m** for the g3.0 attack. The corrected
   value is **0.051 m** — the rest was an endpoint artefact (see HANDOFF_2 §7.4).
2. Copy `truncate_common()` and `arc_lengths()` from `compare_runs.py` into
   `frechet_analysis.py`.
3. Correct the centreline DoS claim in all four files listed above.
4. Add the differential verification results to `RESULTS.md` — currently absent
   despite being the strongest material.

## Priority 2 — Reproduce the width attack locally (2 hours)

Never yet run on 0.50.0. This decides whether the demo can be laptop-only.

```bash
# in the container
mkdir -p /autoware_map/g3_map
cp /geoshield/data/route_g3.0.osm /autoware_map/g3_map/lanelet2_map.osm
cp /autoware_map/nishishinjuku_autoware_map/pointcloud_map.pcd /autoware_map/g3_map/
```

Relaunch with `map_path:=/autoware_map/g3_map`, then pose → goal → Auto.
Compare against the clean run.

## Priority 3 — Complete the differential verification table (half a day, laptop)

`differential_verify.py` has only been run against `width_ramp` and
`centerline_injection`. Run it against `speed_spoof`, `oneway_flip`,
`connectivity_break`, `tunnel_bridge_flip`, `width_step`. Those code paths
exist but were never exercised.

## Priority 4 — Scripted runs in the container (half a day)

`scenario_runner.py` is at `/geoshield/scenario_runner.py`. Running it inside
the container gives deterministic poses *and* live RViz — better than either
machine gave separately.

**Note:** it currently targets the EC2 topic set. Verify `/planning/trajectory`
and the ADAPI engage service exist on 0.50.0 before relying on it.

## Priority 5 — Record `/map/vector_map_marker`

Less urgent now. RViz renders the map natively, so this is only needed if
Foxglove remains part of the demo.

## Priority 6 — `geoshield_verifier` ROS node (3 days)

Unchanged. Now buildable and testable locally.

## Priority 7 — Paper

Unchanged, plus the four fixes in HANDOFF_2 §6.3.

---

# 5. HOUSEKEEPING

| Item | Action |
|---|---|
| `sanity` and `cl_zero` bags/results | **Still on EC2, uncommitted.** `sanity` is the clean-map control that made the centreline comparison valid. Pull them down. |
| Two EC2 instances | g4dn `i-0a6adcc9b89f1125c` is stopped but still billing for its EBS volume. m7i `i-09c2cd3d38a524082` was running at last contact, IP `13.200.252.66`. Inventory both, pull everything, then terminate g4dn. |
| `~/Development/awsim_deprecated/` | Delete. |
| `data/bags/clean_run_good/` | Contains `tampered_run1_0.mcap` — directory was renamed, file was not. |

---

# 6. THE DEMO, AS IT NOW STANDS

Entirely local. Nothing that can fail on a network.

1. **Terminal 1** — launch clean map, RViz opens, pose → goal → Auto, vehicle
   drives its lane in 3D
2. **Terminal 3** — show `attack_injector.py` editing the map, 8 lanelets
3. **Terminal 1** — relaunch tampered map, drive, show the effect
4. **Terminal 3** — `differential_verify.py` → **REJECT**, 8 lanelets named
   with reasons
5. Show the detectability table: what is caught, what is not, and the
   measurements that explain why

Steps 1–3 need Priority 2 completed first, to know how the attack behaves on
0.50.0.

---

# 7. WHAT TO TELL THE NEW CHAT

Upload `HANDOFF_1.md`, `HANDOFF_2.md`, this addendum, and the code files. Then:

> Read the handoff documents. The addendum supersedes parts of HANDOFF_2 — in
> particular, the centreline DoS finding does not reproduce on Autoware 0.50.0,
> and Autoware now runs locally in Docker with GPU-accelerated RViz at 10 Hz,
> so EC2 is optional. Start with Priority 1: correcting the stale 1.414 m figure
> in RESULTS.md and the centreline claim.