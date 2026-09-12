# GeoShield — Results

Draft results section. Every number below is measured, with the producing
command noted. Nothing here is projected or expected.

Map: Nishi-Shinjuku Lanelet2 vector map (TIER IV / AWSIM release), 979
lanelets, 884 of subtype `road`. Same map used by Sato et al., VehicleSec 2025,
so figures are directly comparable.

**Reference build: Autoware 0.50.0, official Docker image**
(`ghcr.io/autowarefoundation/autoware:universe-devel`, digest
`sha256:405225eda6c05161bfde39cc7885511f3f4d9699d126891891420dd80c2e024a`),
ROS 2 Jazzy, Ubuntu 24.04, planning simulator, local laptop, GPU-accelerated
RViz at 31 fps, planner holding 10.014 Hz.

A secondary build exists — Autoware 0.52.0, source-compiled on AWS EC2
`m7i.4xlarge`, CUDA packages absent. §4.1–§4.3 and §4.9 are now measured on the
reference build; §4.6 lists the rows still carried over from 0.52.0 and marks
them. Results from the two builds are not interchangeable: one finding has
already failed to reproduce across them (§8.4).

**Reading order.** This document is a working record and its section numbers
are stable so that cross-references do not break silently. The strongest
result is §8, differential verification, not §1. In the paper the order
inverts: §8 leads, §1–§3 support it, and §4 becomes background.

---

## 1. Attribute coverage in the external witness

*Derived from map and OSM files only. Build-independent.*

Tier 3 cross-verification assumes OpenStreetMap can corroborate the HD map.
We measured how often it actually carries the attributes an attacker would
target.

| Attribute | Nishi-Shinjuku | Munich centre |
|---|---|---|
| `highway` class | 100.0% | 100.0% |
| `oneway` | 57.7% | 38.0% |
| `lanes` | 32.2% | 34.6% |
| `maxspeed` | 13.2% | 54.5% |
| `tunnel` | 8.1% | 9.1% |
| `bridge` | 3.8% | 0.4% |
| **`width`** | **0.3%** | **5.9%** |

*(782 and 2496 drivable ways respectively; `osm_adapter.py --density`.)*

Two findings:

**Speed-limit coverage is regional.** 13.2% in Tokyo against 54.5% in Munich.
Cross-verification of speed attacks is viable in well-mapped European cities
and largely unavailable in Tokyo.

**Width coverage is structurally absent everywhere.** 0.3% and 5.9%. Even in
the better-mapped city, 94% of roads carry no width tag. Lane width — the
attribute Sato et al. showed can steer a vehicle — cannot be cross-verified
against OSM in any region we measured. Any defence against width tampering
must therefore rely on the map's internal consistency alone, or on the map's
own history (§8).

---

## 2. Honest geometric variation sets a noise floor

*Derived from map files only. Build-independent.*

Width discontinuity between connected road lanelets, on the untampered map
(973 connected pairs, `analyze_widths.py`):

| Statistic | Absolute [m] | Relative | Gradient [m/m] |
|---|---|---|---|
| median | 0.139 | 0.044 | 0.0039 |
| p90 | **0.775** | 0.219 | 0.0412 |
| p99 | 2.194 | 0.711 | 0.3309 |

Sato et al.'s smallest *effective* attack expands a lanelet by 0.5 m. That
value sits **below the 90th percentile of honest variation** on all three
formulations. Filtering to `subtype=road` changes almost nothing (975 → 973
pairs): the variation is intrinsic to intersections, merges and lane flares,
not to non-drivable subtypes.

A global threshold on width discontinuity therefore cannot separate tampering
from ordinary map geometry.

*The canonical p90 is 0.775 m, from `analyze_widths.py` on the clean map. An
earlier draft quoted 0.826 m, which was a per-lanelet severity score for
lanelet 566 copied out of a `tier1_rules.py` run — not a distribution
statistic. Corrected throughout.*

---

## 3. Detection by attack class

*Derived from map files only. Build-independent.*

30 independent tampering campaigns at a 3% budget, pooled: 29,370
(instance, lanelet) examples, 955 positives, train/test split by run.
Recall measured at a fixed 5% flag budget (`detectability.py`).

| Attack | Best detector | Recall |
|---|---|---|
| `speed_spoof` | speed vs neighbour median | **0.846** |
| `width_step` > 1 m | width vs neighbour median | **1.000** |
| `width_ramp` 2–5 m | width vs neighbour median | 0.640 |
| `width_ramp` 1–2 m | width vs neighbour median | 0.201 |
| `width_ramp` < 1 m | any | ≈0.02 |
| `connectivity_break` | isolation forest | 0.120 |
| `oneway_flip` | isolation forest | 0.021 |

**GeoShield detects speed spoofing and abrupt width changes reliably.** Speed
attacks are caught at 84.6% recall; a single-lanelet width step above 1 m is
caught at 100%.

**Gradual ramps defeat it.** The same detector that achieves 1.000 on a single
1.6 m step achieves 0.201 on a ramp of comparable total magnitude spread across
consecutive lanelets. That contrast isolates the mechanism precisely: the
attack is not hidden by its size but by its *distribution*.

### Ablation

Isolation Forest over all 31 features was outperformed by a single
hand-computed feature (`width_vs_nbr_median`): 0.133 against 0.420 recall on
large ramps. Diluting three informative axes across 31 degraded detection, so
the unsupervised layer does not earn its place on this problem and is reported
rather than retained. Separability analysis of the chain-trend statistic gives
a **maximum achievable F1 of 0.220** across chain lengths 3–7 — a ceiling no
threshold choice can exceed.

This ceiling is the reason the defence in §8 does not analyse a single map
snapshot at all.

---

## 4. End-to-end impact: measured, and far weaker than the map change

> **BUILD: Autoware 0.50.0, official Docker image** (digest `405225ed`),
> local laptop, GPU-accelerated RViz. §4.1–§4.3 and §4.9 are measured on this
> build. §4.6 lists rows still carried over from Autoware 0.52.0 (EC2,
> source-compiled) and marks them.

### 4.1 The endpoint artefact, and the corrected figure

`frechet_analysis.py` originally reported `d_Fe` = **1.414 m** for the 3.0 m
width ramp. That number was an artefact of the recording window, not a
deviation. Both runs recorded for a fixed 90 s and the tampered run travelled
further before recording stopped; beyond the clean path's final sample there is
nothing to match against, so every trailing tampered sample couples to that
same point and the offset climbs to exactly the endpoint gap.

Confirmed three independent ways on the same pair of bags:

| Diagnostic | Value |
|---|---|
| `d_Fe` untruncated | **1.4139 m** |
| endpoint gap between the two runs | **1.4139 m** |
| difference in driven length | **1.42 m** (338.62 vs 340.04) |
| max offset excluding the final 20 points | 0.051 m |

The reported deviation, the endpoint gap and the length difference are the same
quantity measured three ways. `truncate_common()` cuts both paths to the
shorter one's arc length; `--no-truncate` reproduces the fault on demand, so it
can be demonstrated rather than asserted.

A second instance of the same artefact, at 50× the magnitude, appeared when a
90 s clean bag was compared against a 150 s tampered bag: untruncated `d_Fe`
72.5920 m, endpoint gap 72.5920 m, truncated 0.0707 m. The mechanism is
unambiguous.

**Two metrics, both reported.** `route_overlap.py` prints the peak lateral
offset computed on the raw points and on the same paths resampled to 400
points. Resampling relocates sample positions and can step over the true
maximum, so the raw peak is the conservative figure and is the one we report.

| Run pair | Raw peak | Resampled | Lanelets driven |
|---|---|---|---|
| `clean_0.50.0` vs `g3_0.50.0_v2` | **0.0402 m** | 0.0334 m | 8 / 8 |
| `restore_test` vs `restore_tampered` | **0.0441 m** | 0.0410 m | 8 / 8 |

The second pair was recorded after `scenario_runner.py` was reverted to commit
`39e0009`, so the result is reproduced across a code change rather than by
re-running identical code. The 4 mm difference is run-to-run variation in the
simulator. **We report 0.0402 m.**

Two implementations agree: `compare_runs.py` gives 0.051 m on the original 90 s
bags and `frechet_analysis.py` gives 0.0509 m raw on the same pair. The scripts
were not comparable before this. `compare_runs.py` reads `.mcap` directly,
decoding schemas embedded in the file; `frechet_analysis.py` used `rosbag2_py`,
which resolves types from the environment and parses `metadata.yaml`. A bag
written by 0.52.0 carries `version: 9` metadata whose QoS block writes
`history: unknown`, which the 0.50.0 reader cannot parse (`yaml-cpp: bad
conversion`) — it aborts before reaching any message. That is why the corrected
value could only ever be computed on one machine and the disagreement went
unnoticed for several sessions. Both scripts now prefer `.mcap`, with a
`rosbag2` fallback for sqlite3 bags.

### 4.2 Full attack exposure, at `--duration 150`

An earlier measurement recorded for a fixed 90 s and ended at 338.62 m of a
393.2 m route. The ramp grows monotonically along a straight successor chain
(3013032 → 3002007 → 3002013, no fork), so its two largest steps lay in the
54.6 m the vehicle never covered: only 6 of 8 tampered lanelets were driven,
and the deviation was the response to a ramp at 67.8% of its peak.

Raising the recording window to 150 s resolves this. Both runs now complete:

| | Clean | Tampered |
|---|---|---|
| driven length | 411.25 m | 411.23 m |
| driven points | 4,209 | 4,206 |
| start | (82259.8, 50468.7) | (82259.8, 50468.7) |
| end | (81857.5, 50439.2) | (81857.5, 50439.2) |

Every tampered lanelet is now on the driven path. Distances from each lanelet's
centreline to the driven trajectory, with widths measured from the written
`.osm` rather than the label file (see §4.8):

| Lanelet | Width, clean → written | Centreline shift | Distance to path | Driven |
|---|---|---|---|---|
| 3012234 | 3.400 → 3.800 (+11.8%) | 0.560 m | 0.19 m | yes |
| 3013054 | 2.574 → 3.510 (+36.4%) | 0.925 m | 0.36 m | yes |
| 3013093 | 2.641 → 3.744 (+41.8%) | 1.290 m | 0.36 m | yes |
| 3012977 | 2.603 → 5.333 (+104.9%) | 1.684 m | 1.24 m | yes |
| 3002017 | 2.679 → 3.802 (+41.9%) | 1.684 m | 1.16 m | yes |
| 3013032 | 3.101 → 5.202 (+67.8%) | 2.437 m | 1.16 m | yes |
| **3002007** | **3.058 → 6.639 (+117.1%)** | **2.790 m** | **1.91 m** | **yes** |
| 3002013 | 3.057 → 6.358 (+108.0%) | 2.790 m | 2.83 m | yes |

Frame check: the nearest map lanelet to the driven path is 0.19 m away,
confirming the bag's `map` frame and the adapter's `local_x`/`local_y` frame
share an origin. **8 of 8 on route.**

### 4.3 A large map change produced a small behavioural change

The deviation can now be paired with the largest change in the entire ramp,
because the vehicle drove all of it:

| Quantity | Value |
|---|---|
| lanelet 3002007 width | 3.058 → 6.639 m (**+117.1%**) |
| its centreline displacement | **2.790 m** |
| resulting driven deviation | **0.0402 m** |

A 2.79 m displacement of the geometric centreline moved the driven path by four
centimetres — **a factor of 70** — with the vehicle exposed to the full ramp,
not a truncated portion of it.

**Autoware's planner optimises within the drivable area rather than tracking
the geometric centre.** Widening a lane does not move the vehicle; it enlarges
the space the planner is free to optimise inside, and the planner's existing
objective keeps it near its previous line. This is why boundary displacement
alone is a weak steering attack, and it predicts why Sato et al. needed an
explicit centreline rather than relying on width expansion.

The consequence for the defence is direct: **this attack cannot be detected by
observing vehicle behaviour.** Four centimetres over a 411 m route is not a
signal a monitor could act on, at any magnitude this attack class reaches. The
tampering is visible only in the map, which is where §8 looks for it.

The finding does not depend on either artefact in §4.1 or §4.4, because it
compares the *map* against the *behaviour* rather than two trajectories against
each other. It is not a headline: it is a negative result about attack efficacy,
on one route, on one planner build.

### 4.4 `d_Fp` is not measurable by this method

`planned_path()` uses only the last `Trajectory` message of each bag. Two runs
that stopped at different points produce last-plans beginning at different ego
positions — an analogous artefact by a different mechanism, which arc-length
truncation does not remove because the plans never shared an origin.

| Quantity | Value |
|---|---|
| `d_Fp` | 2.0266 m |
| gap between the two plans' first points | **1.9975 m** |

The origin gap is **98.6% of the reported deviation**. The previously published
1.998 m was almost entirely the two plans starting from different places.
`compare_runs.py` does not compute `d_Fp`, so it had never been checked against
a second implementation. **Withdrawn.** Recovering it needs a plan selected at a
common route position, not a common wall-clock instant. `plan_origin_gap_m` is
now emitted on every run so the artefact is visible rather than latent.

### 4.5 The route-overlap claim is withdrawn

An earlier draft reported that identical injector, magnitude and seed gave
0.083 m off-route against 1.414 m on-route — "a 17× difference from targeting
alone." The on-route figure is the artefact. The off-route figure came from the
same untruncated script and carries the same fault class. The comparison is
**uninterpretable until both are re-derived**, not inverted: we do not know
which is larger, and asserting that on-route deviation is lower would replace
one unsupported claim with another.

### 4.6 Remaining rows, not yet re-derived

*All produced by the untruncated script on 0.52.0. Retained as a record of what
was run, not as results.*

| Condition | `d_Fp` [m] | `d_Fe` [m] | Status |
|---|---|---|---|
| tamper off-route | 0.020 | 0.083 | artefact-bearing |
| ramp, 2.0 m total | 0.312 | 0.204 | artefact-bearing |
| ramp, 3.0 m total | 1.998 → withdrawn | 1.414 → **0.0402** | **re-derived on 0.50.0** |
| ramp, 4.5 m total | — | — | ego failed to localise |
| ramp, 6.0 m total | — | — | ego failed to localise |

Sato et al. report `d_Fe` of 0.6049, 0.8419 and 1.0965 m for lane widths of
3.5, 4.0 and 4.5 m on this map, and no completed route at 5.0 m. Safety
threshold `th` = 0.5 m (3.0 m lane, 1.895 m vehicle). Our corrected figure is
an order of magnitude below both their measurements and the threshold.

### 4.7 Large displacements fail by localisation, not planning

At 4.5 m and above the ego could not initialise on the tampered map: the start
pose, computed from the clean centreline, no longer fell inside the displaced
drivable area. The vehicle never moves. Sato et al. observed a related failure
at 5.0 m, attributed to infeasible planning; our mechanism differs
(localisation rather than planning) and we do not claim to reproduce theirs.

This observation does not depend on Fréchet distance and is unaffected by
either artefact. It should still be re-checked on 0.50.0.

### 4.8 The injector does not write the widths it records

Comparing intended width (label file) against realised width (measured from the
written XML), for `route_g3.0`:

| Lanelet | Labels say | Map has | Error |
|---|---|---|---|
| 3012977 | 4.103 | 5.333 | **+1.230** |
| 3002007 | 5.683 | 6.639 | +0.956 |
| 3002017 | 4.554 | 3.802 | **−0.752** |
| 3002013 | 6.057 | 6.358 | +0.301 |
| 3013054 | 3.324 | 3.510 | +0.186 |
| 3013032 | 5.351 | 5.202 | −0.149 |
| 3012234 | 3.775 | 3.800 | +0.025 |
| 3013093 | 3.766 | 3.744 | −0.022 |

Worst error 1.23 m — more than three times the ramp's 0.375 m per-step. Errors
take both signs, so it is not a scale factor.

**Cause.** `write_tampered_map()` displaces every node of a lanelet's left
boundary by a uniform `shift = tampered − original`, once per labelled lanelet.
Adjacent lanelets share boundary nodes, so a node belonging to both lanelet *k*
and *k+1* is displaced **twice, cumulatively**, by two different shifts. Its
final position depends on how many labelled lanelets claim it and on each one's
local normal. Measured on `route_g3.0`: of 178 left-boundary nodes touched,
**7 are touched more than once** — one per join in an eight-lanelet chain. The
contamination is at the seams.

This is the third appearance of the shared-node mechanism, after the repair
overshoot (factor 1.88) and the differential-verification false positives.
Repair compounded on read; the injector compounds on write. A regression test
pins the count at exactly 7 of 178 so the geometry cannot drift unnoticed.

**Scope.** `build_dataset.py` never calls `write_tampered_map()` —
`write_tampered_map` appears in one file only, `attack_injector.py`, at its
definition and one call site in `main()`. Track A generates campaigns, extracts
features and evaluates entirely in memory, so features and magnitudes come from
the same `RoadSegment` objects and are mutually consistent. **§3, §5 and the
29,370-example dataset are unaffected.** The fault reaches written `.osm` maps
only: §4's simulation inputs and §8's differential-verification inputs.
Differential verification compares two maps and therefore sees realised
geometry regardless; only its *reported* magnitudes were wrong.

Attack magnitudes for written maps must be recomputed by differencing clean
against tampered, as done in §4.2, rather than read from the label file.

### 4.9 Success flags did not verify where the vehicle was

A run reported `trajectory: yes`, `engage: ok` and `stopped recording` while
the vehicle started at the **goal** rather than the start pose and drove
814.61 m on a 393.2 m one-way route. Measured from the bag: start
(81857.5, 50439.2), end (81857.5, 50439.2) — the goal coordinates, identical.
It had driven a loop. The bag itself looked healthy: 15,080 messages, 167 s,
6,701 odometry points, the correct topic set.

The cause is stack state, not code. The previous run had left the vehicle
engaged and arrived; re-publishing an initial pose does not reset an
already-arrived stack. Neither existing check could catch it: `trajectory` asks
whether a plan exists, and `engage` asks whether the service accepted. Neither
asks where the vehicle is.

`verify_run()` now reads the recorded bag after every drive and compares the
first odometry point against the scenario start pose, the last against the
goal, and driven length against route length. `main()` exits 3 on failure, so
`demo.sh` cannot proceed on a bad bag. Validated against three bags:

| Bag | Driven | Start error | Verdict |
|---|---|---|---|
| `restore_test` | 411.23 m | 0.0 m | PASS |
| `restore_tampered` | 411.24 m | 0.0 m | PASS |
| `g3_repeat` | 814.61 m | **403.35 m** | **FAIL** — start-pose and loop checks |

Two operational rules follow. **Relaunch Autoware between every run**, so each
drive starts from a clean stack. And **a run is not valid because the script
said so** — it is valid because the recorded trajectory matches the scenario it
was supposed to execute. Both are now enforced rather than remembered.

This is the third distinct measurement error caught by checking recorded data
against intent, after the endpoint artefact (§4.1) and the planning-origin
artefact (§4.4). In each case the pipeline reported success and produced a
number that was wrong for a reason invisible in its own output.

---

## 5. Repair is not achievable by geometric means

*Derived from map files only. Build-independent, except where noted.*

Two repair strategies were implemented and evaluated against the 3.0 m ramp —
the largest ramp on which the ego still completed the route.

**Per-lanelet correction** (clamp a lanelet toward its neighbours' median
width) achieved **TP 0, FP 9, recall 0.000**. Calibrated on the map's own
distribution, the flag threshold lands at 2.5 × p90 = 2.065 m, while each ramp
step is 3.0 / 8 = 0.375 m. The nine lanelets it did flag were legitimately
wide intersection segments; "repairing" them displaced geometry far enough
that the ego could no longer localise (observed on 0.52.0). **A defence that
flags the wrong lanelets is worse than no defence.**

**Chain-based correction** using the monotonic trend statistic that achieves
recall 1.000 on isolated ramps:

| Threshold | Flagged | Precision | Recall |
|---|---|---|---|
| p95 | 180 | 0.028 | 0.625 |
| p99 | 55 | 0.055 | 0.375 |
| p99.5 | 25 | 0.000 | 0.000 |
| p99.9 | 8 | 0.000 | 0.000 |

Precision never exceeds 0.055. Above p99 recall collapses entirely: the
tampered chains rank below the 25 highest-scoring clean chains.

**False-positive baseline.** On the untampered map at p99, the same detector
flags **49 of 979 lanelets (5.0%)**. The 3.0 m attack adds roughly six flags
to that background. The signal is inside the false-positive floor, not merely
close to it.

Geometry correction itself works once the right lanelets are known: iterative
damped correction (0.6, ≤6 passes) reduces mean width error from 3.157 m to
0.025 m, a 99.2% reduction. Single-pass correction overshoots by a mean factor
of 1.88 because adjacent lanelets share boundary nodes and corrections compound
along a ramp. **The failure is in detection, not in correction.** Repair is
reported as a finding and is not shipped as a feature.

---

## 6. Summary

GeoShield reliably detects speed-limit spoofing (recall 0.846) and abrupt
single-lanelet width tampering (recall 1.000 above 1 m) from a single map
snapshot. Both are catchable because they present a large deviation at one
location.

Gradual width ramps are not detectable that way. Their per-step magnitude sits
below the honest variation of a real urban map, so per-lanelet thresholds
cannot separate them; their sequence signature ranks below ordinary
intersection geometry, so chain statistics cannot either — the measured ceiling
is F1 0.220. The external witness that would resolve the ambiguity does not
carry the attribute: OSM tags road width on 0.3–5.9% of ways. Repair inherits
the same failure, because it cannot repair what it cannot locate.

**The principal contribution is that map integrity verification must be
differential rather than absolute.** Absolute verification asks whether a map
is plausible, and §1–§5 measure how far that question can be pushed before it
stops separating tampering from ordinary geometry. Differential verification
asks whether a map is *the same map as before*, and under a supply-chain threat
model — where the adversary controls the delivered map but not its publication
history — that question is answerable. It rejects all six applicable attack
types at recall 1.000, including the two that single-snapshot detection handles
worst and one that changes no geometric value at all (§8).

A second finding points the same way from the opposite direction. The attack
that defeats single-snapshot detection also barely moves the vehicle: 2.79 m of
centreline displacement produced 0.0402 m of driven deviation across a full
411 m traversal (§4.3). Behavioural monitoring cannot catch this attack either.
The tampering exists only in the map, so the map is where it must be caught.

### Withdrawn claim

An earlier draft stated:

> There exists a band — roughly 2.0 m to 4.5 m of cumulative displacement
> spread across a route — in which the attack exceeds the safety threshold
> (`d_Fe` = 1.414 m against 0.5 m) while remaining below the detection floor
> of every method evaluated. Characterising that band … is this work's
> principal contribution.

**This is withdrawn.** The 1.414 m figure was an endpoint artefact (§4.1). The
corrected value for that configuration is 0.0402 m, measured at full attack
exposure on the reference build, and no measured condition exceeds the 0.5 m
threshold: 2.0 m gives 0.204 m (itself artefact-bearing), 3.0 m gives 0.0402 m,
and 4.5 m and above fail to localise rather than deviating. The band as
described has no measured support.

The *detection* half of the claim stands and is unaffected — §3 and §5 are
derived from map files, not from trajectories. What does not stand is the
assertion that an undetectable attack was simultaneously shown to be unsafe on
this route. §4.3 gives a mechanism suggesting no such band exists for
width-widening attacks on this planner: the planner optimises within the
drivable area, so widening a lane grants it freedom it does not use.

Recording this withdrawal rather than quietly deleting the claim is deliberate:
the artefact was found by our own diagnostic, and the diagnostic is now in the
tool.

---

## 7. Limitations

- Single map, single route. Generalisation to other maps is untested.
- The planning simulator uses a kinematic vehicle model and ideal
  localisation; AWSIM with full sensor simulation would add physics and NDT
  localisation, and is left as future work.
- **Five of the seven attack types in §8.2 were evaluated with a single
  tampered lanelet (N=1).** Recall on one positive is binary, not a rate: we
  know the defence caught that instance, not that it catches 100% of such
  attacks. Only `width_ramp` (N=8) supports a meaningful recall figure.
  Multiple independent campaigns per attack type would fix this.
- The tampering tool rewrites the XML through ElementTree, so clean and
  tampered files differ in formatting as well as coordinates. Autoware parses
  both identically, but a byte-preserving text edit would be cleaner.
- "Clean" means "not injected by us." The base map may contain genuine survey
  or authoring errors, which would appear as honest variation in our
  calibration.
- Detection thresholds were calibrated on the same map they were evaluated on.
  Cross-map calibration is untested.
- §4.6 rows and §5's localisation observation still come from Autoware 0.52.0
  while the reference build is 0.50.0. One finding has already failed to
  reproduce across the two (§8.4).
- Attack magnitudes written to `.osm` differ from those recorded in the label
  files by up to 1.23 m (§4.8). Magnitudes for written maps must be measured,
  not read.
- Differential verification assumes an authentic prior version is available.
  §8.5 sets out what that assumption costs.

---

## 8. Differential verification

*Derived from map files only. Build-independent — it inspects the map, not the
planner's response to it.*

### 8.1 Why the reference is the previous version

Single-snapshot verification asks whether a map is plausible. §2 measures why
that question is hard to answer: a 0.5 m width change sits below the 90th
percentile of honest variation on a real urban map, and §3's separability
analysis puts a ceiling of F1 0.220 on any chain-trend method. The evidence is
not there to be extracted; more estimator capacity does not help, and §3's
ablation shows it actively hurts.

Every method in the HD map change-detection literature compares the map
against something external. §1 rules out OSM as that reference for width: 0.3%
coverage in Tokyo. Under a supply-chain threat model, though, the adversary
controls the map that is delivered but not the map that was already accepted
and running. **The previous trusted version is the external reference.**

This reframes the question from "is this geometry plausible?" — which §2 shows
is unanswerable at the relevant magnitude — to "did this geometry change, and
is the change coherent?" A 0.375 m per-step ramp is invisible against honest
variation but perfectly visible against the same lanelet's previous width,
because the honest variation cancels.

`differential_verify.py` compares width, computed centreline position, speed
limit, direction, and connectivity field-by-field, adds a structural diff over
relation member roles (§8.3), then groups flagged lanelets into connected runs.
Exit code 2 on REJECT.

### 8.2 Results — all seven attack types

*`run_all_dv.py` against the clean map, seed 42. Reproduced field-for-field
against the stored summary.*

| Attack | Field | N_gt | Changes | FP | Precision | Recall | F1 | Verdict |
|---|---|---|---|---|---|---|---|---|
| `width_ramp` | `width_m` | 8 | 33 | 10 | 0.444 | **1.000** | 0.615 | **REJECT** |
| `width_step` | `width_m` | 1 | 10 | 6 | 0.143 | **1.000** | 0.250 | **REJECT** |
| `speed_spoof` | `speed_limit` | 1 | 1 | 0 | **1.000** | **1.000** | **1.000** | **REJECT** |
| `oneway_flip` | `one_way` | 1 | 1 | 0 | **1.000** | **1.000** | **1.000** | **REJECT** |
| `connectivity_break` | `successors` | 1 | 2 | 1 | 0.500 | **1.000** | 0.667 | **REJECT** |
| `centerline_injection` | structure | 1 | 1 | 0 | **1.000** | **1.000** | **1.000** | **REJECT** |
| `tunnel_bridge_flip` | `tunnel`/`bridge` | 0 | — | — | — | — | — | **N/A** |
| **clean vs clean (control)** | — | 0 | **0** | **0** | — | — | — | **ACCEPT** |

**Recall is 1.000 on every applicable attack type, with zero changes flagged on
the clean control.** Read the N_gt column alongside it: five of the six rows
have a single ground-truth positive, so recall 1.000 there means one lanelet
found out of one, not a rate (see §7). `width_ramp` at N=8 is the only row that
supports a recall figure in the usual sense.

**Precision is the honest weakness.** `width_step` flags 10 changes for 1
targeted lanelet. As with the ramp, the extra flags are lanelets sharing
boundary nodes with the tampered one, whose geometry genuinely moved (§4.8) —
but we report strict precision and do not adjust it. An argument that the false
positives are really true positives would improve the number without improving
the method, and the shared-node coupling is a property of Lanelet2 that any
deployment would face.

**`tunnel_bridge_flip` is structurally inapplicable**, not undetected. The
Nishi-Shinjuku Lanelet2 map carries no tunnel or bridge tags — those are OSM
concepts, and the injector found zero candidate lanelets. The attack cannot be
evaluated on this map, which is the correct result rather than a gap.

**`connectivity_break` required a new writer, and revealed something about the
format.** The original `write_tampered_map()` handled `width_m`,
`speed_limit_kph` and `oneway` only. `connectivity_break` modified successors
in memory, but **Lanelet2 encodes connectivity implicitly, through shared
boundary endpoints** — there is no successor tag to tamper with. The written
`.osm` was byte-identical to the clean map, which is why differential
verification returned ACCEPT with 0 changes: nothing had changed. Severing
connectivity requires duplicating the shared end nodes so the endpoint match
breaks. With that writer in place the attack is caught at recall 1.000, and the
resulting map still loads (`Succeeded to load lanelet2_map`, `/map/vector_map`
publishing to 17 subscribers).

The structural point is worth more than the table row: **an attacker cannot
break connectivity by editing attributes at all.** They must edit geometry,
which moves nodes, which differential verification sees regardless. An earlier
draft claimed differential verification catches `connectivity_break` at REJECT
severity 1.0 without this writer; that claim was unfounded and is corrected
here.

Compare against single-snapshot detection on the same attacks: `width_ramp` at
this magnitude is caught at 0.201–0.640 recall, `oneway_flip` at 0.021,
`connectivity_break` at 0.120 (§3), and centreline injection at 0.000 by field
comparison alone (§8.3). Differential verification catches all of them.

### 8.3 `structural_diff()` — the change that mattered

The first version compared field values only and missed centreline injection
**entirely, at recall 0.000**. The attack adds an explicit `role="centerline"`
member to the relation. The boundaries are untouched, so every geometric field
compared — width, computed centre, speed, direction, connectivity — is
byte-identical between versions. Nothing in a field diff can see it.

Comparing *which member roles exist* catches it at precision 1.000. This is
the general lesson: an attack can be invisible in every value you compare while
being trivially visible in the structure you did not think to compare. It
generalises beyond centrelines to any Lanelet2 regulatory element that can be
added, removed or re-roled without moving a coordinate.

### 8.4 A finding that did not reproduce

Injecting an explicit centreline — even one geometrically identical to
Autoware's own computed centreline, zero displacement — prevented the planning
stack from producing any trajectory on **Autoware 0.52.0** (EC2,
source-compiled, CUDA packages absent). The vehicle never moved. Across five
variants (0.0 m, 0.8 m, 1.0 m × 6, 2.0 m × 8) the result was identical: zero
trajectory messages, one unique position, with `Not found safe pull out path,
publish stop path` in the log. `Using waypoint centerline` appeared repeatedly,
confirming Autoware was reading the injections. Since zero displacement also
failed, presence alone appeared sufficient.

**The same map drives normally on Autoware 0.50.0** (official Docker image):
`Routing: Set`, `Motion: Moving`, vehicle completes the route.

We do not claim this as a denial-of-service finding. At least three variables
differ between the two observations — Autoware version, build configuration
(source vs prebuilt, CUDA absent vs present), and host environment — and no
controlled experiment has isolated which. The correct statement is that
centreline injection produced no trajectory on one build and no effect on
another, cause unresolved. Resolving it means running the identical map on both
builds with the readiness gate applied, varying only the version. Both builds
remain available; it is a side quest, not a priority.

**Detection is unaffected.** Differential verification catches centreline
injection at precision 1.000, recall 1.000, on both builds, because it inspects
the map file rather than the planner's response to it. That the downstream
effect is version-dependent is precisely the argument for detecting tampering
at the ingest boundary instead of waiting to observe its consequences.

### 8.5 What differential verification does not do

- **It requires an authentic prior version.** First-time provisioning has no
  reference; a compromised initial delivery is undetectable by this method.
  A Merkle tree over map tiles with a publisher-signed root would supply the
  missing provenance layer, and would prove origin rather than plausibility —
  a signed-but-tampered tile would pass signature verification and still fail
  the differential check. Left as future work.
- **It flags change, not malice.** A legitimate map update also changes
  geometry. The severity grouping and coordinated-run analysis separate
  scattered edits from coordinated ones, but the final accept/reject decision
  on a genuine update needs an authenticated publisher, which is the same
  orthogonal mechanism.
- **It does not measure impact.** A REJECT verdict says the map differs
  coherently from its predecessor, not that driving it would be unsafe. §4.3
  shows those are genuinely different questions: a 2.79 m centreline
  displacement moved the vehicle 0.0402 m.
- **Precision is modest where the attack touches a chain.** 0.444 on the ramp,
  0.143 on the single step. Shared boundary nodes make neighbouring lanelets
  move too, so the flag set is wider than the target set. In deployment this
  costs review effort on a rejected update, not missed detections.
- **Five of seven rows rest on one tampered lanelet each** (§7). The mechanism
  is demonstrated; the rates are not established.

Note that `oneway_flip` (0.021) and `connectivity_break` (0.120) are the two
attacks single-snapshot detection handles worst, because they are topological
attacks being hunted with geometric features. Differential verification catches
both at recall 1.000 — `oneway_flip` because a direction reversal is an
exact-value change between versions, and `connectivity_break` because severing
implicit connectivity requires moving nodes that the diff then sees.