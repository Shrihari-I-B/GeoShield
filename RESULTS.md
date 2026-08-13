# GeoShield — Results

Draft results section. Every number below is measured, with the producing
command noted. Nothing here is projected or expected.

Map: Nishi-Shinjuku Lanelet2 vector map (TIER IV / AWSIM release), 979
lanelets, 884 of subtype `road`. Same map used by Sato et al., VehicleSec 2025,
so figures are directly comparable.

Stack: Autoware Universe, ROS 2 Jazzy, Ubuntu 24.04, planning simulator,
headless on AWS EC2 `m7i.4xlarge` (CPU only).

---

## 1. Attribute coverage in the external witness

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
must therefore rely on the map's internal consistency alone.

---

## 2. Honest geometric variation sets a noise floor

Width discontinuity between connected road lanelets, on the untampered map
(973 connected pairs, `analyze_widths.py`):

| Statistic | Absolute [m] | Relative | Gradient [m/m] |
|---|---|---|---|
| median | 0.139 | 0.044 | 0.0039 |
| p90 | 0.775 | 0.219 | 0.0412 |
| p99 | 2.194 | 0.711 | 0.3309 |

Sato et al.'s smallest *effective* attack expands a lanelet by 0.5 m. That
value sits **below the 90th percentile of honest variation** on all three
formulations. Filtering to `subtype=road` changes almost nothing (975 → 973
pairs): the variation is intrinsic to intersections, merges and lane flares,
not to non-drivable subtypes.

A global threshold on width discontinuity therefore cannot separate tampering
from ordinary map geometry.

---

## 3. Detection by attack class

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

---

## 4. End-to-end impact in simulation

Fréchet distance between clean and tampered trajectories, identical scripted
start and goal poses, 8-lanelet 393.2 m route (`frechet_analysis.py`).
Threshold `th` = 0.5 m from a 3.0 m lane and a 1.895 m vehicle.

| Condition | `d_Fp` [m] | `d_Fe` [m] | Outcome |
|---|---|---|---|
| tamper off-route | 0.020 | 0.083 | noise floor |
| ramp, 2.0 m total | 0.312 | 0.204 | under threshold |
| **ramp, 3.0 m total** | **1.998** | **1.414** | **exceeds threshold** |
| ramp, 4.5 m total | — | — | ego failed to localise |
| ramp, 6.0 m total | — | — | ego failed to localise |

For reference, Sato et al. report `d_Fe` of 0.6049, 0.8419 and 1.0965 m for
lane widths of 3.5, 4.0 and 4.5 m on this map.

**Route overlap governs effectiveness.** The identical injector, magnitude and
seed produced `d_Fe` = 0.083 m when the tampered lanelets lay off the driven
route and 1.414 m when they lay on it — a 17× difference from targeting alone.
Randomly-placed tampering is largely inert; an adversary who knows the victim's
route is not. This dependence is not reported in prior work.

**Large displacements are self-defeating but not harmless.** At 4.5 m and
above, the ego vehicle could not initialise on the tampered map: the start
pose, computed from the clean centreline, no longer fell inside the displaced
drivable area. The vehicle never moves. Sato et al. observed a related failure
at 5.0 m, attributed to infeasible planning; our mechanism differs
(localisation rather than planning) and we do not claim to reproduce theirs.

---

## 5. Repair is not achievable by geometric means

Two repair strategies were implemented and evaluated against the 3.0 m ramp,
the configuration that exceeds the safety threshold.

**Per-lanelet correction** (clamp a lanelet toward its neighbours' median
width) achieved **TP 0, FP 9, recall 0.000**. Calibrated on the map's own
distribution, the flag threshold lands at 2.5 × p90 = 2.065 m, while each ramp
step is 3.0 / 8 = 0.375 m. The nine lanelets it did flag were legitimately
wide intersection segments; "repairing" them displaced geometry far enough
that the ego could no longer localise. **A defence that flags the wrong
lanelets is worse than no defence.**

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

---

## 6. Summary

GeoShield reliably detects speed-limit spoofing (recall 0.846) and abrupt
single-lanelet width tampering (recall 1.000 above 1 m). Both are catchable
because they present a large deviation at a single location.

Gradual width ramps are not detectable by geometric analysis of the map. Their
per-step magnitude sits below the honest variation of a real urban map, so
per-lanelet thresholds cannot separate them; their sequence signature ranks
below ordinary intersection geometry, so chain statistics cannot either. The
external witness that would resolve the ambiguity does not carry the attribute:
OSM tags road width on 0.3–5.9% of ways.

**There exists a band — from roughly 2.0 m to 4.5 m of cumulative displacement
spread across a route — in which the attack exceeds the safety threshold
(`d_Fe` = 1.414 m against 0.5 m) while remaining below the detection floor of
every method evaluated.** Characterising that band, and showing why it resists
both internal and external verification, is this work's principal contribution.

---

## 7. Limitations

- Single map, single route. Generalisation to other maps is untested.
- The planning simulator uses a kinematic vehicle model and ideal
  localisation; AWSIM with full sensor simulation would add physics and NDT
  localisation, and is left as future work.
- The tampering tool rewrites the XML through ElementTree, so clean and
  tampered files differ in formatting as well as coordinates. Autoware parses
  both identically, but a byte-preserving text edit would be cleaner.
- "Clean" means "not injected by us." The base map may contain genuine survey
  or authoring errors, which would appear as honest variation in our
  calibration.
- Detection thresholds were calibrated on the same map they were evaluated on.
  Cross-map calibration is untested.
