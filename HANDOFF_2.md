# GeoShield — Handoff Document (Sections 6–11)

Continues from `HANDOFF_1.md`. Same rules: only what actually happened in the
session; unverified items marked **[UNVERIFIED]**.

---

# 6. DECISIONS MADE

## 6.1 Adopted

| # | Decision | Reason |
|---|---|---|
| 1 | **Autoware planning simulator, headless (`rviz:=false`)** instead of AWSIM | Laptop has Intel integrated graphics only; AWSIM requires NVIDIA drivers + Vulkan. The planning simulator exercises map → planning → control, which is exactly what the study measures. Confirmed working: full stack came up with all CUDA packages missing. |
| 2 | **Stay on Ubuntu 24.04 + ROS 2 Jazzy** | Initially I recommended downgrading to 22.04 + Humble for tutorial compatibility. Reversed: Autoware was already compiled on 24.04/Jazzy, and rebuilding would cost weeks for compatibility no longer needed once AWSIM was dropped. |
| 3 | **No DDS across the internet** | Original plan had AWSIM on laptop and Autoware on EC2 connected by ROS 2/DDS. Rejected: DDS discovery uses UDP multicast (does not traverse the internet), point-cloud bandwidth is tens of MB/s upstream, and control loops need millisecond latency. Everything ROS-related runs on one machine; only `.osm` files and rosbags cross. |
| 4 | **EC2 runs headless; visualisation happens on the laptop from rosbags** | Eliminates the NVIDIA/DCV/OpenGL problems previously encountered on the cloud instance. |
| 5 | **Migrate g4dn.4xlarge → m7i.4xlarge (CPU only)** | Triggered by an AWS "insufficient capacity" error in ap-south-1c. Verified the planning simulator runs fine on CPU. Roughly a fifth of the cost, better availability. Reportable finding: HD map integrity evaluation needs no GPU. |
| 6 | **Tiers 1 and 2 run in PARALLEL, not cascaded** | The original v3.0 spec cascaded them (only Tier-1 failures reach Tier 2). That caps whole-system recall at Tier 1's recall — the ML could never catch anything the rules missed, defeating its purpose. Only Tier 3 (the network call) is gated by gamma. |
| 7 | **HD map is the artefact under test; OSM is the independent witness** | The uploaded draft paper ingested from Overpass *and* verified against Overpass — circular. Keeping them separate makes Tier 3 a genuine external check. |
| 8 | **Isolation Forest trained on CLEAN maps only (novelty detection)** | Training on the mixed set lets the attack become part of "normal" and detection collapses. Enforced in code: `fit()` takes clean segments, and standardisation statistics come from the clean set. |
| 9 | **Threat model = supply-chain / update-pathway compromise**, not network MITM | Matches Sato et al., so results are comparable. Real HD maps arrive over authenticated OTA tile delivery, so network MITM is the weaker model. |
| 10 | **Scripted poses, never hand-clicked** | Headless means no "2D Pose Estimate" button. This is methodologically correct rather than a workaround: hand-clicked poses vary by centimetres between runs, which is the same magnitude as the effect being measured. |
| 11 | **Route selection avoids lanelets with regulatory elements** | 436 of 979 lanelets carry a regulatory element. Routing through them hit a map defect: `[FATAL] behavior_velocity_planner: No stop line at traffic_light_reg_elem_id = 1391, please fix the map!` Routing around it avoids editing the map (which would have to be applied identically to clean and tampered copies, adding a confound). |
| 12 | **Route discovery by BFS over the successor graph, not euclidean distance** | The first picker chose two lanelets 838 m apart that were in different connected components; the planner refused to route. BFS gives goals reachable by construction. |
| 13 | **Chain builder follows forks and keeps junctions** | Measured: strict no-fork chains have median length 3 (shorter than the attack); with forks, 643 chains reach length ≥5 vs 290 strict. The injector also walks successors and *chooses* branches, so the detector must search paths too. |
| 14 | **Tier 1 calibrates at p90, not p99.5** | p99.5 of clean chain trends is 0.948 while the ramp attack produces 0.1–0.5 — the threshold sat above the signal, guaranteeing zero recall. Deliberate design statement: Tier 1's job is recall; Tier 2 removes false positives. |
| 15 | **Repair uses chain-trend detection, not per-lanelet width deviation** | Per-lanelet repair achieved recall 0.000 and corrupted the map badly enough to break localisation. |
| 16 | **Repair geometry correction is iterative and damped (0.6, ≤6 passes)** | Single-pass overshot by a mean factor of 1.88 because adjacent lanelets share boundary nodes and corrections compound along a ramp. A fixed 1/1.88 factor would be wrong too, since coupling varies. Iterative converges: mean width error 3.157 m → 0.025 m (99.2% reduction). |
| 17 | **Differential verification (Phase 7) — the working defence** | Single-snapshot detection has a measured ceiling (F1 0.220). Every method in the HD map change-detection literature compares the map against something external. Under a security threat model the adversary controls the map but not its history, so the previous version is the reference. Result: recall 1.000 vs 0.02. |
| 18 | **`structural_diff()` added to differential verification** | The first version compared only field *values* and missed centreline injection entirely (recall 0.000) — the attack adds an element without changing any geometric value. Comparing which member roles exist catches it at precision 1.000. |
| 19 | **Attack injector randomises every parameter** | If the injector always wrote the same value, a model would memorise the injector rather than learn implausibility. Ramp length 4–9, total gain 1.2–3.0 m, speed factors sampled both up and down. |
| 20 | **Attack budget fixed at 3%** | Declared assumption, not a tuning knob. Sets class imbalance, Isolation Forest contamination, and which metrics are honest. |
| 21 | **Train/test split BY RUN, never by segment** | Splitting by segment would put a lanelet's clean version in training and its tampered version in test. |
| 22 | **PR-AUC reported, never accuracy** | At ~3% positives, a model that always says "clean" scores 97% accuracy while catching nothing. ROC-AUC is also inflated by the large negative class. |
| 23 | **Matplotlib figures as the primary demo visual** | A 1.4 m deviation on a 393 m route is nearly invisible in a wide 3D shot. The figure shows map, both trajectories, zoom, and deviation-vs-threshold in one image. |
| 24 | **Foxglove over RViz2 for local bag inspection** | Foxglove reads message definitions embedded in the `.mcap`; RViz resolves types from the local ROS install, and the laptop has no Autoware message packages. |

## 6.2 Explicitly REJECTED — do not re-suggest

| Rejected | Why |
|---|---|
| **AWSIM** | Needs NVIDIA/Vulkan; laptop has Intel integrated graphics. Its value is photorealistic camera/LiDAR simulation, which this study never uses. Contributes nothing to any measurement. `~/Development/awsim_deprecated/` is the abandoned directory. |
| **GraphSAGE** | Cut. Relational features already encode neighbourhood context. Evidence: Isolation Forest with 31 features got 0.133 on large ramps while a single hand-crafted feature got 0.420 — more capacity on the same features made things worse. Also, the attack does not change the graph: 975 edges before, 975 after, zero lanelets with changed successors. |
| **Hidden Markov Model** | Never built. Phase 3 measured the sequence-detection ceiling at F1 0.220; attacked chains sit inside the clean distribution (attack median trend 0.235 vs clean p90 0.324). Listed as future work with a specific hypothesis: sequence-level aggregation of weak per-lanelet evidence may recover signal a single summary statistic cannot. |
| **More feature engineering** | Seven formulations tested (absolute delta, relative delta, gradient, chain trend, taper, boundary asymmetry, centreline shift). All overlap with honest variation. The problem is the evidence, not the estimator. |
| **A MarkerArray converter node for the map** | Unnecessary — Autoware already publishes `/map/vector_map_marker` as a standard `visualization_msgs/MarkerArray`. It simply was not being recorded. |
| **RViz2 on the laptop** | Cannot decode `autoware_planning_msgs/Trajectory` without Autoware's message packages installed locally. |
| **RViz2 on EC2 via DCV/VNC** | Previously attempted and abandoned; the headless + rosbag path replaced it. |
| **`gdown` / the generic `sample-map-planning`** | Not needed once the Nishi-Shinjuku map was obtained directly from the AWSIM GitHub release. Blocked by PEP 668 anyway. |
| **Map-file editing to fix traffic light 1391** | Would have to be applied identically to clean and tampered copies, adding a confound. Routing around it is cleaner. |
| **Repair as a deliverable** | Built and evaluated; proved infeasible. Precision ≤0.055 at every threshold. Reported as a finding, not shipped as a feature. |
| **FGSM/PGD, label flipping, training-data poisoning, backdoor attacks** | Out of scope per the frozen v3.0 spec — these belong to adversarial ML, not HD map metadata security. |

## 6.3 Corrections made to the uploaded draft paper

Four issues identified that still need fixing in the paper:

1. **Circular trust model** — the draft ingests from Overpass and verifies against Overpass.
2. **MITM threat model** — should be supply-chain / update-pathway.
3. **Inverted SRTM elevation rule** — the draft says "tunnel terrain should sit lower than nearby ground". SRTM is a Digital Surface Model capturing the top of canopy/structures, so sampling at a mountain tunnel returns the hill *above* it. Correct formulation compares divergence between road grade and terrain grade. Also: SRTM covers 60°N–56°S (excludes Scandinavia above 60°N) at ~30 m, from a February 2000 snapshot. Copernicus DEM or NASADEM suggested instead.
4. **Sato et al. is not cited** despite being the closest prior work.

---

# 7. PROBLEMS ENCOUNTERED & FIXES

## 7.1 Environment and tooling

| # | Problem | Cause | Fix |
|---|---|---|---|
| 1 | Overpass HTTP 406 | Missing `User-Agent`; Overpass blocks generic `python-requests` | Added `HEADERS` with a proper User-Agent |
| 2 | Overpass HTTP 429 | Mirror rate-limited | Four endpoints, retry with exponential backoff (5s/10s/20s), on-disk cache at `~/.cache/geoshield/overpass/` |
| 3 | `pip install` "externally-managed-environment" | PEP 668 on Ubuntu 24.04 | `--break-system-packages` |
| 4 | GitHub push rejected: "Password authentication is not supported" | Removed in 2021 | `gh auth login` |
| 5 | `git push` rejected, non-fast-forward | Remote had a README commit | `git pull --rebase origin main`; `__pycache__/*.pyc` blocked the rebase → `rm -rf __pycache__` |
| 6 | Nothing pushed to GitHub for several sessions | Only `git commit` was ever run; no remote/push | `git remote add origin`, `git push -u origin main` |
| 7 | `tmux` session died repeatedly | `Ctrl-Z` suspends tmux (`[1]+ Stopped`); must use `Ctrl-b d` to detach | Established terminal discipline (Section 3) |
| 8 | `scp` "path canonicalization failed" | Destination directory did not exist on EC2 | `mkdir -p ~/autoware_map` first |
| 9 | Wrong map used in a run | `cp` executed before `scp` had finished | Verify timestamp with `ls -la` before relaunching |
| 10 | SSH connection timed out | Security group allows "My IP"; the laptop's IP changed (once because the network switched to IPv6-primary — `curl ifconfig.me` returned a `2401:...` address) | Update the security group's SSH source to "My IP" |
| 11 | EC2 "insufficient capacity" for g4dn.4xlarge | AWS capacity in ap-south-1c | Launched the AMI on m7i.4xlarge in ap-south-1a |
| 12 | New file appeared not to take effect (happened ~4 times) | Browser download did not overwrite, or landed elsewhere | Habit adopted: `grep -c "<distinctive string>" file.py` before running anything |
| 13 | Pasted multi-line commands silently swallowed | `source setup.bash` takes 10–30 s and consumes buffered input | One command at a time, wait for the prompt |

## 7.2 Autoware integration

| # | Problem | Cause | Fix |
|---|---|---|---|
| 14 | `Package 'autoware_launch' not found` | Commands run on the laptop instead of EC2 | Check the prompt / `hostname` |
| 15 | `engage: The passed service type is invalid` | Shell had not sourced `~/autoware/install/setup.bash`, so Autoware message types were unresolvable | Source before running; later added to `.bashrc` |
| 16 | `ros2 bag record` captured 0 messages | Same cause — recorder subprocess inherits the unsourced environment and cannot resolve `autoware_planning_msgs/msg/Trajectory` | Source the shell before running the scenario |
| 17 | `trajectory: NO -- routing failed` while routing had actually succeeded | Runner polled `/planning/scenario_planning/trajectory`; this build publishes on `/planning/trajectory` | `sed -i 's|/planning/scenario_planning/trajectory|/planning/trajectory|g'` |
| 18 | `[FATAL] behavior_velocity_planner: No stop line at traffic_light_reg_elem_id = 1391` | Map defect: a traffic light with no associated stop line. Routing succeeded (306 m trajectory generated) but the velocity stage aborted | Route picker excludes lanelets with regulatory elements |
| 19 | `KeyError: 'separation_m'` | Renamed to `route_length_m` in the picker but the print statement was not updated | `sed -i "s/sc\['separation_m'\]/sc.get('route_length_m', 0)/"` |
| 20 | `/planning/trajectory` recorded 0 messages | Recorder subscribed before the trajectory existed; ROS 2 bag does not retroactively capture | Increased pre-record sleep 2 s → 4 s |
| 21 | `ros2 topic hz` timeout traceback | `hz` never returns on its own; the 8 s subprocess timeout fired after recording completed | Removed the probe (cosmetic only — the bag was valid) |
| 22 | Runs failing intermittently with `! no kinematic_state` | Stack not finished starting; fixed sleeps were unreliable on CPU | Readiness gate: `until [ "$(ros2 node list 2>/dev/null \| wc -l)" -gt 100 ]; do sleep 10; done` |

## 7.3 Detection logic

| # | Problem | Cause | Fix |
|---|---|---|---|
| 23 | R4 speed rule fired on 273/979 lanelets (28%) | `a_lat_max = 3.0 m/s²` measures "this is a dense urban map", not implausibility | Raised to 6.0; skip junction lanelets and those under 15 m; require ratio ≥ 1.25 |
| 24 | `n_clean_chains` collapsed 643 → 4 | The R4 patch applied `_is_junction` to the chain builder; 43.8% of road lanelets carry `turn_direction` | Removed the junction filter from the chain path (kept in R4 only) |
| 25 | R1 recall 0.000 despite firing 15 times | Calibrating at p99.5 (0.948) put the threshold above the attack signal (0.1–0.5) | Calibrate at p90 |
| 26 | Chain builder produced almost no chains | Stopped at forks; median no-fork chain length is 3 | Beam-limited DFS following up to 3 branches, preferring the branch that continues the width drift |
| 27 | `centreline_gap_succ` measured 0.000 on every segment | `geometry` is stored in lat/lon, where a 0.5 m lateral shift is ~5e-6 degrees — below the precision written into the file | Compute in the `local_x`/`local_y` metric frame; adapter now stores `_cl_start_x/_y`, `_cl_end_x/_y`, `_lb_*`, `_rb_*` in `raw_tags` |
| 28 | `cl_jump_succ` and `boundary_asym_succ` still 0.000 | **Structural, not a bug:** consecutive lanelets SHARE boundary nodes, so displacing a boundary moves the neighbour too. There is no discontinuity at the join. The signature is intra-lanelet | Features remain in the code but are dead; documented as a finding |
| 29 | Magnitude-stratified analysis showed recall collapse in the top band | Units artefact: `magnitude` is metres for width attacks but km/h for speed spoofs (max 100.6) and 1.0 for categorical flips. Pooling put a 50 km/h change in the same band as a 30 m width change | `stratified_by_type()` — bands only compare within an attack type |
| 30 | Per-lanelet repair: TP 0, FP 9, recall 0.000, and it broke localisation | Threshold 2.5 × p90 = 2.065 m while each ramp step is 0.375 m. The 9 flagged lanelets were legitimately wide intersection segments | Rebuilt around chain-trend detection |
| 31 | Repair overshot by mean factor 1.88 | Adjacent lanelets share boundary nodes; corrections compound along a ramp | Iterative damped correction (0.6, ≤6 passes) |
| 32 | Chain-based repair: precision ≤ 0.055 at every threshold; recall collapsed to 0.000 above p99 | Attacked chains rank below the 25 highest-scoring clean chains | **Unresolved — repair abandoned as infeasible** |
| 33 | `differential_verify.py` missed centreline injection entirely (recall 0.000) | Compared field values only; centreline injection changes no geometric value | Added `structural_diff()` comparing member roles → precision 1.000 |

## 7.4 Measurement errors caught

| # | Problem | Cause | Fix |
|---|---|---|---|
| 34 | **`d_Fe` reported as 1.414 m — WRONG** | Endpoint artefact. The two runs each recorded for a fixed 90 s; the tampered run was 1.4 m further along when recording stopped. With no clean sample beyond that point, every trailing attacked sample matched the clean path's final point, and offset grew monotonically to exactly the endpoint gap. Evidence: last 10 offsets 0.474 → 1.414; max excluding last 20 points = 0.051; endpoint gap = 1.414 | `truncate_common()` in `compare_runs.py` cuts both paths to the shorter one's arc length. **Corrected value: 0.051 m** |
| 35 | Mislabelled bag | `tampered_run1` was recorded while the launch still had the clean map loaded | Renamed to `clean_run_good` (still contains `tampered_run1_0.mcap`) |
| 36 | Overwrote `scenario.json` with the tampered-map route, then backed up the *new* file as `scenario_original.json` | Sequencing mistake | Regenerated from the clean map; also saved as `scenario_clean.json` |
| 37 | Hypothesised the attack changed the routing graph | **Disproved by measurement:** 975 edges before and after, zero lanelets with changed successors. Boundary displacement moves node coordinates, not node IDs | Hypothesis withdrawn |

## 7.5 Unresolved

| Problem | Status |
|---|---|
| **Centreline injection immobilises the vehicle instead of steering it** | Root cause identified but not fixed. Zero-shift centreline (geometrically identical to Autoware's computed one) also immobilises → presence alone is sufficient. Consistent with Autoware issue #10295: `getCenterLinePath` cannot clip the path when `use_waypoints` is true and the lane has an embedded centreline. **No `use_waypoints` parameter exists in this build** — `grep -rni "waypoint"` in `autoware_behavior_path_planner/share/` returns nothing, so the conversion is unconditional. |
| **Repair infeasible** | Documented as a finding; not fixed. |
| **`RESULTS.md` is stale** | Still quotes the 1.414 m artefact; has no differential-verification results. |
| **`frechet_analysis.py` lacks arc-length truncation** | All `results_*.json` produced on EC2 by that script carry the artefact. Only `compare_runs.py` is corrected. |
| **`oneway_flip` (0.021) and `connectivity_break` (0.120) barely detected by single-snapshot methods** | These are topological attacks against geometric features. A graph reachability check would likely catch both. Not built. Note: differential verification catches both at REJECT severity 1.0. |

---

# 8. CURRENT STATE

## 8.1 Working and verified

**Track A — detection pipeline (laptop)**

- `osm_adapter.py` — 782 drivable ways parsed for Nishi-Shinjuku, 2,496 for Munich
- `lanelet2_adapter.py` — 979 lanelets, 100% width/speed/oneway coverage, 82.8% with successors, metric frame confirmed
- `attack_injector.py` — 7 attack types, targeted mode, writes valid tampered XML
- `tier1_rules.py` — 5 rules, recall 1.000 at 28.5% flag rate
- `tier2_iforest.py`, `build_dataset.py`, `evaluate.py`, `detectability.py` — full pipeline over 29,370 examples
- `differential_verify.py` — **the working defence**

**Track B — simulation (EC2)**

- Autoware planning simulator on CPU (m7i.4xlarge), 122 nodes
- `scenario_runner.py` — deterministic route discovery, pose publishing, ADAPI engage, rosbag recording
- Clean baseline `clean_run_good`: 3,479 driven points, 4,468 odometry messages, 338.62 m driven
- `sanity` run (clean map, post-centreline-experiments): `trajectory: yes`, `engage: ok`, 4,468 messages

**Figures**

- `results/attack_figure.png` — Sato-style dark paired figure
- `results/compare.png` — corrected, shows 0.051 m

## 8.2 Key measured results

### OSM attribute density

| Attribute | Nishi-Shinjuku (782 ways) | Munich (2,496 ways) |
|---|---|---|
| `highway` | 100.0% | 100.0% |
| `oneway` | 57.7% | 38.0% |
| `lanes` | 32.2% | 34.6% |
| `maxspeed` | 13.2% | 54.5% |
| `tunnel` | 8.1% | 9.1% |
| `bridge` | 3.8% | 0.4% |
| **`width`** | **0.3%** | **5.9%** |

### Honest width variation (973 connected road pairs)

| Metric | median | p90 | p95 | p99 |
|---|---|---|---|---|
| absolute [m] | 0.139 | 0.775 | 1.147 | 2.194 |
| relative | 0.044 | 0.219 | 0.306 | 0.711 |
| gradient [m/m] | 0.0039 | 0.0412 | 0.0930 | 0.3309 |

Median road lanelet length 32.9 m. Sato's +0.5 m attack expressed in each
metric: 0.500 m absolute, 0.167 relative, 0.0152 gradient — **below p90 on all
three**.

Width distribution: min 1.49, p05 2.60, median 3.12, mean 3.39, p95 5.38,
max 11.79, stdev 0.96 m.

### Detection by attack type (5% flag budget, 29,370 examples, 955 positives)

| Attack | Best detector | Recall |
|---|---|---|
| `speed_spoof` | `speed_vs_nbr_median` | **0.846** |
| `speed_spoof` | `d_speed_succ` | 0.782 |
| `width_step` > 1 m | `width_vs_nbr_median` | **1.000** |
| `width_ramp` 2–5 m | `width_vs_nbr_median` | 0.640 |
| `width_ramp` 2–5 m | `rel_width_succ` | 0.577 |
| `width_ramp` 1–2 m | `width_vs_nbr_median` | 0.201 |
| `width_ramp` < 1 m | any | ~0.02 |
| `connectivity_break` | isolation forest | 0.120 |
| `oneway_flip` | isolation forest | 0.021 |

Detectability threshold for width ramps: recall 0.014 below 1.0 m, 0.420 above.

### Ablation

| Configuration | F1 | PR-AUC | Recall |
|---|---|---|---|
| rule: `chain_trend_max` | 0.158 | 0.074 | 0.651 |
| rule: `width_vs_nbr_median` | 0.150 | 0.065 | 0.137 |
| rule: `rel_width_succ` | 0.117 | 0.064 | 0.701 |
| **isolation forest** | **0.089** | **0.052** | **0.437** |
| rule: `width_z_local` | 0.062 | 0.030 | 1.000 |
| rule: `width_taper` | 0.062 | 0.028 | 1.000 |

**Isolation Forest is beaten by single hand-crafted features.**

Separability ceiling for the chain trend statistic:

| chain length | best F1 |
|---|---|
| 3 | 0.196 |
| 4 | 0.158 |
| 5 | 0.188 |
| 6 | 0.208 |
| 7 | 0.220 |

### Repair — infeasible

Per-lanelet: TP 0, FP 9, FN 8, recall 0.000. Broke localisation.

Chain-based:

| threshold | flagged | precision | recall |
|---|---|---|---|
| p95 | 180 | 0.028 | 0.625 |
| p99 | 55 | 0.055 | 0.375 |
| p99.5 | 25 | 0.000 | 0.000 |
| p99.9 | 8 | 0.000 | 0.000 |

Clean-map false-positive baseline at p99: **49 of 979 lanelets (5.0%)**.

### Trajectory deviation (Autoware, 393.2 m route, 8 lanelets)

| Condition | `d_Fp` [m] | `d_Fe` [m] | Outcome |
|---|---|---|---|
| tamper off-route | 0.020 | 0.083 | noise floor |
| ramp, 2.0 m total | 0.312 | 0.204 | under threshold |
| ramp, 3.0 m total | 1.998 | **1.414 → corrected 0.051** | see §7.4 |
| ramp, 4.5 m total | — | — | ego failed to localise |
| ramp, 6.0 m total | — | — | ego failed to localise |

Sato et al. for reference: `d_Fe` 0.6049 / 0.8419 / 1.0965 m at `w_l` 3.5 / 4.0
/ 4.5 m; route not completed at 5.0 m. Safety threshold `th` = 0.5 m (3.0 m
lane, 1.895 m vehicle).

**Route overlap governs effectiveness:** identical injector, magnitude and seed
gave 0.083 m off-route and (uncorrected) 1.414 m on-route.

**Map change vs behavioural change:** lanelet 3002013 went 3.058 → 6.639 m
(+117.1%), centreline displaced 2.79 m — yet driven deviation was 0.051 m.

### Differential verification — the working defence

Against `route_g3.0.osm` (width ramp):

```
lanelets      : 979 → 979   (+0 / -0)
changes found : 33   accepted 0   suspect 14   rejected 19
coordinated runs: 7
TP 8   FP 10   FN 0
precision 0.444   recall 1.000   F1 0.615
VERDICT: REJECT
```

Against `centerline_attack.osm`:

```
changes found : 8   rejected 8
TP 8   FP 0   FN 0
precision 1.000   recall 1.000   F1 1.000
VERDICT: REJECT
```

Clean vs clean control: **0 changes, VERDICT ACCEPT.**

Note on the 10 FPs in the width case: those lanelets share boundary nodes with
tampered ones, so their geometry genuinely moved. The label file records only
what the injector explicitly targeted. Arguably a strength, not a false
positive.

### Centreline injection — denial of service

| Map | Trajectory msgs | Unique positions | Result |
|---|---|---|---|
| Clean (control, same stack, readiness gate) | 147 pts | 3,479 | drives 393 m |
| +1 centreline, **0.0 m shift** | 0 | 1 | never moves |
| +1 centreline, 0.8 m shift | 0 | 0 | never moves |
| +6 centrelines, 1.0 m | 0 | 1 | never moves |
| +8 centrelines, 2.0 m | 0 | 1 | never moves; `Not found safe pull out path, publish stop path` |

`Using waypoint centerline` appears repeatedly in the log — Autoware **is**
reading the injections. Zero-shift also fails, so presence alone is sufficient.

## 8.3 Written but untested

- **[UNVERIFIED]** `attack_figure.py --span 400` full-map render — suggested but never run
- **[UNVERIFIED]** `differential_verify.py` against `speed_spoof`, `oneway_flip`, `connectivity_break`, `tunnel_bridge_flip` maps — the code paths exist and would fire, but were never exercised
- **[UNVERIFIED]** `repair.py` on the centreline attack

## 8.4 Broken or stale right now

| Item | State |
|---|---|
| `RESULTS.md` | Stale — quotes 1.414 m, no differential verification |
| `frechet_analysis.py` | No arc-length truncation; all EC2-produced `results_*.json` carry the artefact |
| `cl_jump_succ`, `boundary_asym_succ` | Dead features (structurally always zero) |
| `centerline_attack.py` | Produces DoS, not steering |
| `repair.py` | Functional but the approach is infeasible |
| `~/Development/awsim_deprecated/` | Should be deleted |
| `data/bags/clean_run_good/` | Contains `tampered_run1_0.mcap` (renamed directory) |
| EC2 instance | **[UNVERIFIED]** — running at last contact; should be stopped |

---

# 9. IMMEDIATE NEXT STEPS

Roughly one week available. Jury has asked for a live 3D view of the attack and
the defence acting against it.

## Priority 1 — Fix the stale record (1–2 hours, laptop only)

1. Add `truncate_common()` to `frechet_analysis.py` (copy from `compare_runs.py`)
2. Re-run all EC2 Fréchet analyses, or re-derive locally from the bags
3. Update `RESULTS.md` §4 with corrected `d_Fe` values and the endpoint-artefact explanation
4. Add the differential verification results to `RESULTS.md` — currently absent, and they are the strongest material

## Priority 2 — Complete the differential verification evaluation (half a day, laptop)

Run `differential_verify.py` against every attack type, not just width ramp and
centreline. Produces a full defence table to sit beside the detection table.

## Priority 3 — Record `/map/vector_map_marker` (half a day, EC2)

Extend `TOPICS` in `scenario_runner.py`:

```python
TOPICS = [
    "/planning/trajectory",
    "/localization/kinematic_state",
    "/map/vector_map_marker",        # the road, as markers
    "/planning/mission_planning/route_marker",
    "/vehicle/status/velocity_status",
    "/initialpose",
    "/tf", "/tf_static",
]
```

`/map/vector_map_marker` is latched and published once at startup — start
recording before the map loads, or replay from the beginning.

Then re-record: `demo_clean`, `demo_attack`, `demo_protected`,
`demo_clean_protected`.

## Priority 4 — `geoshield_verifier` ROS 2 node (3 days)

In `ros2_ws/src/geoshield_verifier/`. Subscribes `/map/vector_map` and
`/localization/kinematic_state`; publishes `/geoshield/integrity_status`
(OK / SUSPECT / TAMPERED), `/geoshield/flagged_markers` (red overlays), and
`/geoshield/report`. The detection logic exists; this is ROS plumbing.

Demo against the attacks that are actually caught: speed spoofing (0.846),
width steps (1.000), and differential verification (1.000).

## Priority 5 — Rehearse, and record a video fallback (2 days)

Record the whole sequence the day before. Never demo live without a recording
of the same thing in hand.

## Priority 6 — Paper (1 week, parallel)

Rewrite with measured results; fix the four issues in §6.3.

## Blockers

| Blocker | Notes |
|---|---|
| **EC2 IP changes on every stop/start** | No Elastic IP. Requires a security-group edit each session. Attaching an Elastic IP would remove this. |
| **Security group is "My IP" only** | Breaks on every network change. |
| **Centreline attack cannot steer in this build** | No `use_waypoints` parameter exists. Would require patching Autoware source. |
| **Old g4dn instance still holds an EBS volume** | Billing even while stopped. Terminate once m7i is confirmed sufficient. |

---

# 10. OPEN QUESTIONS

## For you to decide

1. **Which story leads the final paper?**
   - (A) Negative result: "geometric HD map verification cannot detect gradual tampering"
   - (B) Detectability characterisation: "GeoShield works on 3 of 4 attack classes; we measured where the boundary is"
   - (C) Differential verification: "map integrity must be differential, not absolute"

   My recommendation was **(C) with (B) as supporting evidence** — it is the
   only framing with a working defence at recall 1.000. Not yet confirmed by you.

2. **Live Autoware in the demo, or recordings?** You pushed back on my
   recommendation of recordings. Current suggestion: do both — recordings as the
   narrative, live capability held in reserve.

3. **Should the centreline DoS finding be a headline or a footnote?** It is
   novel (zero-displacement immobilisation) but depends on an upstream defect.

4. **How much time on Autoware source patching?** Making the centreline attack
   steer rather than immobilise likely means patching `getCenterLinePath`.

5. **Batch number, team names and USNs** for the review slides — left blank in
   `REVIEW1_SLIDES.md`.

## Things I was waiting on

- Berlin and SF density outputs (interrupted by Overpass rate limits; Tokyo and
  Munich are sufficient)
- Confirmation of whether the EC2 instance is currently running or stopped
- **[UNVERIFIED]** whether the last `cl_zero` and `sanity` results were committed and pushed

---

# 11. CONVENTIONS & PREFERENCES

## Code style

- Python 3.12, `from __future__ import annotations`
- Standard library preferred; `requests`, `scikit-learn`, `matplotlib`, `mcap` only where necessary
- Dataclasses for records (`RoadSegment`, `Change`, `Finding`)
- `argparse` CLI on every script, with `--out` for artefacts
- Every random operation seeded (`--seed`, default 42 or 7)
- Every script runnable standalone; no framework
- Snake_case files and functions; `_leading_underscore` for module-private
- Metric-frame keys in `raw_tags` prefixed with `_`: `_cl_start_x`, `_width_std`

## Comment style

Comments explain **why**, and specifically cite the measurement that motivated
the code. Example from `differential_verify.py`:

```python
# WHY THIS IS SEPARATE FROM THE FIELD DIFF. A centreline-injection attack
# adds an explicit `role="centerline"` member. The boundaries are untouched,
# so every geometric field we compare -- width, computed centre, speed,
# direction, connectivity -- is identical between versions. The field diff
# reported ACCEPT with recall 0.000 against exactly this attack.
```

Module docstrings carry the measured numbers that justify the design.

## Verification habits established

- `grep -c "<distinctive string>" file.py` after every file transfer, before running
- One command at a time on EC2; wait for the prompt
- Check the prompt (`shrihari-i-b@HP-Laptop-15` vs `ubuntu@ip-172-31-...`) before running anything
- `ls -la` on the map file to confirm the timestamp before relaunching
- Readiness gate before every scenario run

## Git

Descriptive commit messages stating the finding, not the action:

```
"Phase 5: repair infeasible. Per-lanelet detection recall 0.000; chain
detection precision <=0.055 at all thresholds. Ramp attacks are effective
and undetectable in the same band."

"Centreline injection with ZERO displacement immobilises the vehicle.
Geometrically identical to Autoware's computed centreline - presence alone
is sufficient. Consistent with Autoware issue #10295."
```

Workflow: `git add -A && git commit -m "..." && git push`

## How you asked me to respond

- **Direct.** Say when something is wrong, including my own errors.
- **Flag ambiguity rather than assuming.** You explicitly asked for
  "if any part of the project design itself is ambiguous or underspecified,
  flag it explicitly instead of assuming."
- **Act as an expert academic advisor and major project evaluator.**
- **Research before answering** rather than relying on memory — you asked for
  this explicitly when the detection approach stalled.
- **Do not over-optimise for the demo at the expense of the project.** You
  corrected me on this: *"He didn't say anything. He only expect us to show our
  project in a 3D model like he can understand. So the main thing is our
  project."*
- **Concise responses**, tables over prose where they carry more.
- **Explain concepts when asked**, plain language first, technical depth second.
- Reply in English.

## Working principles (from the frozen v3.0 spec, still binding)

1. Laptop is the primary development environment
2. AWS only for compute-intensive tasks
3. GitHub is the single source of truth
4. Build incrementally, validate each phase before proceeding
5. Favour simplicity unless complexity shows measurable benefit
6. Prioritise reproducibility and clear experimental evaluation
7. Every ML component must justify inclusion through comparative experiments

---

*End of handoff.*