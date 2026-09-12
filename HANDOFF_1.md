# GeoShield — Handoff Document (Sections 1–5)

Generated at the end of a long working session. Everything below is reported
from what actually happened in that conversation. Items I could not verify are
marked **[UNVERIFIED]**.

---

# 1. PROJECT OVERVIEW

GeoShield is a final-year B.E. Computer Science major project (KSIT Bengaluru,
subject code BCS786) that builds an attack-and-defence pipeline for
high-definition (HD) map tampering in autonomous vehicles. It extends Sato et
al., "WIP: Evaluation of Threats and Impacts of HD Map Tampering Attacks in
Autonomous Driving" (USENIX VehicleSec 2025), which demonstrated that editing
lane geometry in a Lanelet2 HD map deviates an autonomous vehicle's trajectory
but proposed no defence.

The project injects reproducible tampering into a real Lanelet2 HD map, runs
the real Autoware planning stack on both clean and tampered maps, measures the
resulting trajectory deviation, and attempts to detect and reject the tampering
before the map reaches the planner.

**What "done" looks like:** a working attack → detect → prevent loop, with
measured detection metrics, measured end-to-end trajectory impact, and a live
demonstration the project jury can watch. The jury has seen one review
presentation and asked specifically to see the attack and the defence live, in
a 3D view.

---

# 2. TECH STACK & ENVIRONMENT

## Laptop (primary development)

| Item | Value |
|---|---|
| Machine | HP Laptop 15-fd1xxx, hostname `HP-Laptop-15-fd1xxx` |
| User | `shrihari-i-b` |
| OS | Ubuntu 24.04 |
| GPU | Intel integrated only, OpenGL 4.6 (no NVIDIA) |
| Python | 3.12.3 |
| ROS | ROS 2 Jazzy desktop installed (Autoware messages NOT installed) |
| Installed tools | Git, GitHub CLI (`gh`), VS Code, Docker engine, Docker Compose, tmux, jq, ripgrep, fd-find, Foxglove Studio |

Python packages installed during the session (all with
`--break-system-packages`):

```
requests
scikit-learn 1.9.0   (pulled joblib 1.5.3, narwhals 2.24.0, threadpoolctl 3.6.0)
matplotlib 3.6.3     (already present via system packages)
mcap 1.4.0
mcap-ros2-support 0.5.7
zstandard 0.25.0
```

## AWS EC2

| Item | Value |
|---|---|
| Region | `ap-south-1` (Asia Pacific, Mumbai) |
| Account | Shrihari I B (936687422554) |
| Original instance | `Autoware-Build2`, `i-0a6adcc9b89f1125c`, `g4dn.4xlarge`, AZ `ap-south-1c` — **stopped**, capacity errors on restart |
| Current instance | `autoware-ami`, `i-09c2cd3d38a524082`, `m7i.4xlarge`, AZ `ap-south-1a`, 100 GiB gp3 |
| AMI | `ami-02bbdc929efff0163` — "Ubuntu 24.04, ROS 2 Jazzy, NVI…" |
| Security group | `launch-wizard-8`, SSH from "My IP" only |
| Key pair | `autoware-key.pem`, stored at `~/.ssh/autoware-key.pem`, mode 400 |
| Public IPs seen | `52.66.229.10`, then `13.127.160.171`, then `13.126.175.228` (changes on every stop/start — no Elastic IP attached) |
| VPC / subnet | `vpc-02abf3f40dda217dc`, `subnet-092ed2fdf14ac7990` |

**On EC2:** Ubuntu 24.04.4 LTS, kernel `7.0.0-1010-aws`, ROS 2 Jazzy, Autoware
Universe built from source at `~/autoware/install/setup.bash`.

The Autoware build is **missing all CUDA/TensorRT packages** — these warnings
appear on every source and are harmless for this project:

```
not found: ".../autoware_cuda_utils/share/autoware_cuda_utils/local_setup.bash"
not found: ".../autoware_tensorrt_common/share/..."
not found: ".../autoware_tensorrt_classifier/share/..."
not found: ".../autoware_tensorrt_plugins/share/..."
not found: ".../bevdet_vendor/share/..."
not found: ".../cuda_blackboard/share/..."
not found: ".../autoware_cuda_pointcloud_preprocessor/share/..."
not found: ".../autoware_ground_segmentation_cuda/share/..."
```

`~/.bashrc` on EC2 was appended with:
```bash
source ~/autoware/install/setup.bash 2>/dev/null
```

## Map data

| Item | Value |
|---|---|
| Map | Nishi-Shinjuku, Tokyo — TIER IV / AWSIM release |
| Download | `https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip` (redirects to autowarefoundation/AWSIM), 56 MB zip |
| Contents | `lanelet2_map.osm` (10,567,529 bytes), `pointcloud_map.pcd` (148,514,545 bytes), `LICENSE` |
| Lanelets | 979 total — 884 `road`, 84 `crosswalk`, 8 `walkway`, 3 `road_shoulder` |
| Nodes / ways / relations | 36,936 nodes, 5,172 ways, 732 regulatory elements |
| Coordinate frame | Metric, via `local_x` / `local_y` tags (MGRS) |
| Explicit centrelines | **Zero** (`grep -c 'role="centerline"'` returns 0) |

## External services

- **Overpass API** — endpoints used: `overpass-api.de`, `overpass.kumi.systems`,
  plus two more added later. Requires a proper `User-Agent` header or returns
  HTTP 406.
- **GitHub** — `https://github.com/Shrihari-I-B/GeoShield`, branch `main`,
  authenticated via `gh auth login`.

## Simulator

Autoware **planning simulator**, launched headless:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/<MAPDIR> \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  rviz:=false
```

**AWSIM is NOT used** — see Section 6.

## Confirmed Autoware interface (this build)

| Purpose | Topic / service |
|---|---|
| Initial pose | `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`) |
| Goal | `/planning/mission_planning/goal` (`geometry_msgs/msg/PoseStamped`) |
| Engage | `/api/operation_mode/change_to_autonomous` — **SERVICE**, type `autoware_adapi_v1_msgs/srv/ChangeOperationMode` |
| Stop | `/api/operation_mode/change_to_stop` — same type |
| Planned trajectory | `/planning/trajectory` (`autoware_planning_msgs/msg/Trajectory`) — **NOT** `/planning/scenario_planning/trajectory` |
| Driven state | `/localization/kinematic_state` (`nav_msgs/msg/Odometry`) |
| Vector map | `/map/vector_map` |
| Map markers | `/map/vector_map_marker` (`visualization_msgs/MarkerArray`) — **not yet recorded** |

Node count when fully started: **122**.

---

# 3. ARCHITECTURE

## Conceptual design

GeoShield is a **map integrity gate at the ingest boundary** — between the HD
map file and Autoware's planner. It never touches sensors.

```
        HD Map File (Lanelet2 .osm)          OpenStreetMap (Overpass API)
                    |                                    |
                    +------------> RoadSegment <---------+
                                    schema
                                       |
                        +--------------+--------------+
                        |                             |
                  Tier 1                          Tier 2
            plausibility rules            Isolation Forest (31 feats)
                        |                             |
                        +------------+----------------+
                                     |
                              Score Fusion
                                     |
                    score > gamma ---+--- score <= gamma
                          |                     |
                  Tier 3 OSM verify        Verified Map
                          |                     |
                          +---------------------+
                                     |
                          Autoware Planning -> Vehicle Control
```

Tiers 1 and 2 run **in parallel**, not cascaded. Only Tier 3 is gated by the
threshold gamma.

**Phase 7 added a fourth path** (see Section 6): differential verification,
which compares a candidate map version against the previous trusted version
instead of analysing a single snapshot.

## Two tracks, one detection core

| | Track A | Track B |
|---|---|---|
| Source | OSM regions via Overpass | Nishi-Shinjuku Lanelet2 map |
| Purpose | detection metrics at scale | end-to-end safety impact |
| Output | precision / recall / F1 | Fréchet trajectory deviation |
| Hardware | laptop | EC2 (Autoware) |

Both normalise into the common `RoadSegment` schema.

## Machine split (strict)

| Machine | Runs |
|---|---|
| **Laptop** | everything reading/writing `.osm`: injector, adapters, detection, repair, differential verification, figures, git |
| **EC2** | anything needing ROS: `ros2 launch`, `scenario_runner.py`, `frechet_analysis.py`, `ros2 bag` |

Only two things cross: `.osm` maps going up via `scp`, result JSONs and rosbags
coming down.

## Terminal discipline (established after repeated failures)

- **Terminal A** — EC2, inside tmux session `aw`. Runs `ros2 launch`. Never
  typed into again after launch.
- **Terminal B** — EC2, separate SSH. Runs scenarios and analysis.
- **Terminal C** — laptop, local. Generates attacks, runs detection, `scp`, git.

## Mandatory execution order for a simulator run

```
1. LAPTOP   generate the attacked map
2. LAPTOP   scp it to EC2
3. EC2-B    cp it into <MAPDIR>/lanelet2_map.osm
4. EC2-A    Ctrl-C, relaunch Autoware
5.          WAIT for readiness (see below)
6. EC2-B    run the scenario
7. EC2-B    compute Frechet
```

Autoware loads the map **once, at launch**. Copying a new file after launch has
no effect. Readiness gate that finally worked:

```bash
until [ "$(ros2 node list 2>/dev/null | wc -l)" -gt 100 ]; do sleep 10; done; echo READY
```

---

# 4. CURRENT FILE STRUCTURE

## Laptop

```
~/Development/
├── autoware_ws/
├── awsim_deprecated/          (renamed; AWSIM abandoned)
├── projects/
│   └── geoshield/             <- git repo, github.com/Shrihari-I-B/GeoShield
│       ├── .gitignore
│       ├── README.md
│       ├── RESULTS.md
│       ├── data/              (gitignored)
│       │   ├── bags/
│       │   │   ├── clean_run_good/   (contains tampered_run1_0.mcap — renamed dir)
│       │   │   ├── route_attack/
│       │   │   ├── route_g3.0/
│       │   │   ├── route_g4.5/
│       │   │   ├── route_g4.5_retry/
│       │   │   ├── route_g6.0/
│       │   │   ├── tampered_run1/
│       │   │   ├── tampered_run2/
│       │   │   └── tampered_run3/
│       │   ├── dataset/
│       │   │   ├── clean_features.json
│       │   │   ├── examples.json
│       │   │   └── meta.json
│       │   ├── campaign_labels.json / _meta.json / _segments.json
│       │   ├── centerline_attack.osm
│       │   ├── centerline_labels.json
│       │   ├── centerline_1m.osm
│       │   ├── centerline_1m_labels.json
│       │   ├── centerline_single.osm
│       │   ├── cl_single_labels.json
│       │   ├── centerline_zero.osm
│       │   ├── cl_zero_labels.json
│       │   ├── ramp_labels.json / _meta.json / _segments.json
│       │   ├── repaired_g3.0.osm
│       │   ├── route_g3.0.osm / _labels.json / _meta.json / _segments.json
│       │   ├── route_g4.5.osm / _labels.json / _meta.json / _segments.json
│       │   ├── route_g6.0.osm / _labels.json / _meta.json / _segments.json
│       │   ├── route_labels.json / _meta.json / _segments.json
│       │   ├── tampered_ramp.osm
│       │   └── tampered_route.osm
│       ├── results/
│       │   ├── ablation.json
│       │   ├── clean_map_false_positives.txt
│       │   ├── density_munich.txt
│       │   ├── density_nishi-shinjuku.txt
│       │   ├── detectability.json
│       │   ├── detectability_by_type.json
│       │   ├── detectability_v2.json
│       │   ├── detectability_v3.json
│       │   ├── diff_centerline.json
│       │   ├── diff_g3.0.json
│       │   ├── lanelet2_nishishinjuku_widths.txt
│       │   ├── repair_g3.0.json
│       │   ├── results_frechet.json
│       │   ├── results_g3.0.json
│       │   ├── results_g4.5.json
│       │   ├── results_g6.0.json
│       │   ├── results_repaired_g3.0.json
│       │   ├── results_route_attack.json
│       │   ├── tier1_ramp.json / _v2.json / _v3.json
│       │   ├── tier2_ramp.json
│       │   ├── width_delta_formulations.txt
│       │   ├── attack_figure.png
│       │   ├── compare.png
│       │   └── demo_figure.png
│       ├── tests/             (empty)
│       ├── analyze_widths.py
│       ├── attack_figure.py
│       ├── attack_injector.py
│       ├── build_dataset.py
│       ├── centerline_attack.py
│       ├── compare_runs.py
│       ├── detectability.py
│       ├── differential_verify.py
│       ├── evaluate.py
│       ├── features.py
│       ├── lanelet2_adapter.py
│       ├── make_demo_figure.py
│       ├── osm_adapter.py
│       ├── repair.py
│       ├── road_segment.py
│       ├── scenario_runner.py       (also on EC2)
│       ├── separability.py
│       ├── tier1_rules.py
│       ├── tier2_iforest.py
│       ├── topo_diag.py
│       └── verify_run.py
├── ros2_ws/
├── scripts/
└── tools/

~/autoware_map/
├── nishishinjuku_autoware_map/
│   ├── LICENSE
│   ├── lanelet2_map.osm
│   └── pointcloud_map.pcd
└── nishishinjuku_autoware_map.zip

~/.ssh/autoware-key.pem
```

**Note:** the cache directory `~/.cache/geoshield/overpass/` holds Overpass
responses keyed by query hash.

## EC2

```
/home/ubuntu/
├── autoware/                  (Autoware Universe build)
│   └── install/setup.bash
├── autoware_map/
│   ├── nishishinjuku_autoware_map/
│   │   ├── lanelet2_map.osm
│   │   └── pointcloud_map.pcd
│   ├── tampered_map/          (was tampered_ramp)
│   ├── route_map/             (swapped per experiment)
│   ├── cl_map/                (centreline experiments)
│   ├── tampered_ramp.osm
│   ├── tampered_route.osm
│   ├── route_g3.0.osm
│   ├── route_g4.5.osm
│   ├── route_g6.0.osm
│   ├── repaired_g3.0.osm
│   ├── centerline_attack.osm
│   ├── centerline_1m.osm
│   ├── centerline_single.osm
│   └── centerline_zero.osm
├── bags/
│   ├── clean_run1/  clean_run2/  clean_run3/  clean_run_good/
│   ├── cl_final/  cl_ready/  cl_single/  cl_zero/  centerline_1m/
│   ├── centerline_attack/
│   ├── route_attack/  route_g3.0/  route_g4.5/  route_g4.5_retry/  route_g6.0/
│   ├── sanity/
│   └── tampered_run1/  tampered_run2/  tampered_run3/
├── scenario.json
├── scenario_clean.json
├── scenario_original.json
├── scenario_runner.py
├── frechet_analysis.py
├── results_frechet.json
├── results_g3.0.json / _g4.5.json / _g6.0.json
├── results_route_attack.json
├── results_repaired_g3.0.json
├── results_cl_1m.json / _cl_single.json / _cl_final.json / _cl_ready.json / _cl_zero.json
└── results_centerline.json
```

## `.gitignore` contents

```
data/
__pycache__/
*.pyc
.venv/
*.mcap
*.db3
```

---

# 5. CODE WRITTEN SO FAR

**Important:** rather than re-typing ~300 KB of source inline (which risks
transcription errors), the actual final files are attached to this handoff.
Those files ARE the exact latest versions. Upload them to the new chat.

The table below gives the manifest: path, purpose, status, and which earlier
versions are superseded.

| # | File | Purpose | Status |
|---|---|---|---|
| 1 | `road_segment.py` | Common `RoadSegment` dataclass + `Provenance`; haversine, curvature, `max_safe_speed_kph`, `missing()`, `summarise()`. Shared schema for both adapters. | Working, tested |
| 2 | `osm_adapter.py` | Track A adapter. Overpass query with User-Agent, 4 endpoints, retry/backoff, on-disk cache at `~/.cache/geoshield/overpass/`, `--density` report, `--dry-run`, `--no-cache`. Built-in areas: `nishi-shinjuku`, `munich-centre`, `berlin-mitte`, `sf-soma`. | Working, tested |
| 3 | `lanelet2_adapter.py` | Track B adapter. Parses Lanelet2 XML, computes width by arc-length resampling of both boundaries, infers topology from shared endpoint nodes, stores metric centreline/boundary endpoints in `raw_tags` (`_cl_start_x` etc). `--summary`, `--widths`. | Working, tested |
| 4 | `attack_injector.py` | 7 attack types: `width_ramp`, `width_step`, `speed_spoof`, `oneway_flip`, `tunnel_bridge_flip`, `connectivity_break`, plus `--campaign` mixed mode. `--target` for route-specific injection, `--total-gain`, `--out-map` writes real tampered XML by displacing left-boundary nodes along the local normal. | Working, tested |
| 5 | `features.py` | 31 features in 4 families: intrinsic, relational, chain context, missingness. Includes `cl_lateral_shift`, `cl_jump_succ`, `boundary_asym_succ` (latter two proved always-zero). `standardise()` uses clean-set statistics only. | Working; 2 features dead |
| 6 | `tier1_rules.py` | 5 rules: R1 monotonic trend, R2 taper anomaly, R3/R4 speed plausibility + neighbour, R5 topology. Calibrates thresholds from a clean reference map. Chain builder follows forks (beam-limited DFS, max_branch 3, cap 20000). | Working; recall 1.000, precision 0.023 |
| 7 | `tier2_iforest.py` | Isolation Forest novelty detector, fit on clean map only, PR-AUC, threshold sweep, rank reporting. | Working |
| 8 | `build_dataset.py` | Generates N seeded campaigns, pools into `examples.json` with `rows`, `labels`, `magnitudes`, `attack_types`, `run_ids`, `segment_ids`. | Working |
| 9 | `evaluate.py` | Pooled ablation: single-feature rules vs Isolation Forest, split by run. | Working |
| 10 | `detectability.py` | Magnitude-stratified detection, **per attack type** (units differ per type). Fixed 5% flag budget. | Working |
| 11 | `repair.py` | Detect + repair. Chain-trend detection (R1), chain interpolation for ramp anchoring, **iterative damped** geometry correction (damping 0.6, 6 iterations). | Working but **repair proved infeasible** |
| 12 | `scenario_runner.py` | **Runs on EC2.** Route discovery via BFS over successor graph avoiding regulatory-element lanelets; publishes `/initialpose` and goal; calls ADAPI engage service; records rosbag. | Working |
| 13 | `frechet_analysis.py` | **Runs on EC2.** Reads two bags via `rosbag2_py`, computes `d_Fp` and `d_Fe`. **Does NOT truncate to common arc length** — this caused the endpoint artefact. | Working but flawed — see §7 |
| 14 | `differential_verify.py` | **Phase 7, the working defence.** Compares candidate map against previous trusted version: width, centreline position, speed, oneway, connectivity, plus `structural_diff()` comparing which member roles exist. Groups flags into connected runs. Exit code 2 on REJECT. | **Working — best result** |
| 15 | `centerline_attack.py` | Injects explicit `role="centerline"` members with laterally shifted geometry, ramped along the run. | Working (writes valid XML); **effect is DoS not steering** |
| 16 | `compare_runs.py` | Side-by-side clean vs tampered map + trajectory + deviation plot. **Includes `truncate_common()`** — the endpoint-artefact fix. | Working |
| 17 | `attack_figure.py` | Sato-style dark paired figure, zoomed on attack, drivable areas shaded, cyan trajectory, orange tampered lanelets. | Working |
| 18 | `make_demo_figure.py` | Earlier figure generator: overview + zoom + deviation. **Does NOT truncate** — produces the incorrect 1.41 m. Superseded by `compare_runs.py`. | Superseded |
| 19 | `analyze_widths.py` | Width delta in 3 formulations (abs / rel / gradient), road-only filter, Sato comparison. | Working |
| 20 | `separability.py` | Best-achievable-F1 ceiling for chain trend statistic. | Working |
| 21 | `topo_diag.py` | Topology diagnostic: degree distributions, chain lengths with/without forks. | Working |
| 22 | `verify_run.py` | Verifies injected ramps are genuinely connected chains. | Working |

## Superseded versions (do not reuse)

- `make_demo_figure.py` — superseded by `compare_runs.py` (no truncation)
- Earlier `tier1_rules.py` calibrating at p99.5 — superseded by p90 operating point
- Earlier `repair.py` using per-lanelet width deviation — superseded by chain trend
- Earlier single-pass `apply_repair()` — superseded by iterative version
- Earlier `differential_verify.py` without `structural_diff()` — missed centreline injection entirely (recall 0.000)

## Documents produced

| File | Contents |
|---|---|
| `STUDY_NOTES.md` | 10-module conceptual breakdown with 8 flagged ambiguities |
| `RESULTS.md` | Results section — **STALE**, still quotes the 1.414 m endpoint artefact |
| `DEMO.md` | Foxglove setup and 5-minute presentation script |
| `RUNBOOK.md` | Review-day command sequence with what-to-say |
| `REVIEW1_SLIDES.md` | Slide-by-slide content for the college template |
| `PHASE_PLAN.md` | Phases 7–11 completion plan |
| `demo_mockup.html` | Early HTML mockup of the demo console |

---

*Sections 6–11 follow in the next message.*