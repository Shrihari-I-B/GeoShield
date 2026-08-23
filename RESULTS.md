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
RViz, planner holding 10.014 Hz.

A secondary build exists — Autoware 0.52.0, source-compiled on AWS EC2
`m7i.4xlarge`, CUDA packages absent — and **every simulation figure currently
in this document came from that build, not the reference one.** Each table
below carries its own build label. Results from the two builds are not
interchangeable: at least one finding has already failed to reproduce across
them (§8.4).

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

## 4. End-to-end impact: inconclusive on this route

> **STATUS: PENDING RE-MEASUREMENT.** Every figure in this section was
> produced by `frechet_analysis.py` on Autoware 0.52.0 (EC2), by a revision of
> that script that did not truncate to common arc length. All of them carry the
> endpoint artefact described below. They are retained here as a record of what
> was run, not as results. Re-derivation on the reference build (0.50.0,
> Docker) is Priority 2.

### 4.1 The endpoint artefact

`frechet_analysis.py` originally reported `d_Fe` = **1.414 m** for the 3.0 m
width ramp. That number was an artefact of the recording window, not a
deviation.

Both runs recorded for a fixed 90 s. The tampered run was 1.414 m further
along the route when recording stopped. Beyond the clean path's final sample
there is nothing to match against, so every trailing tampered sample couples
to that same final clean point and the offset grows monotonically to exactly
the endpoint gap:

| Diagnostic | Value |
|---|---|
| last 10 lateral offsets | 0.474 → 1.414, monotonic, no plateau |
| max offset excluding final 20 points | 0.051 |
| endpoint gap between the two runs | 1.414 |
| reported `d_Fe` | 1.414 |

The reported deviation and the endpoint gap agree to four decimal places
because they are the same quantity.

Cutting both paths to the shorter one's arc length removes it.
`compare_runs.py` has done this since the artefact was found; re-running the
same two bags (`clean_run_good` vs `route_g3.0`) through it gives **0.051 m**.
Verified like-for-like: same bags, same pair, two independent implementations.
`frechet_analysis.py` now carries `truncate_common()` and reproduces the fault
on demand via `--no-truncate`, so it can be demonstrated rather than asserted.

### 4.2 What was measured, and why none of it can be reported

*Autoware 0.52.0 (EC2, source build). 393.2 m route, 8 lanelets, identical
scripted start and goal poses. Threshold `th` = 0.5 m from a 3.0 m lane and a
1.895 m vehicle.*

| Condition | `d_Fp` [m] | `d_Fe` [m] | Status |
|---|---|---|---|
| tamper off-route | 1.998 → unverified | 0.083 → artefact-bearing | re-derive |
| ramp, 2.0 m total | — | 0.204 → artefact-bearing | re-derive |
| ramp, 3.0 m total | 1.998 → unverified | 1.414 → **corrected 0.051** | re-derive on 0.50.0 |
| ramp, 4.5 m total | — | — | ego failed to localise |
| ramp, 6.0 m total | — | — | ego failed to localise |

Sato et al. report `d_Fe` of 0.6049, 0.8419 and 1.0965 m for lane widths of
3.5, 4.0 and 4.5 m on this map, and no completed route at 5.0 m.

**The route-overlap claim is withdrawn.** An earlier draft of this section
reported that identical injector, magnitude and seed produced 0.083 m
off-route against 1.414 m on-route — "a 17× difference from targeting alone."
The on-route figure is an artefact. The off-route figure came from the same
untruncated script and carries the same fault class. The comparison is
therefore **uninterpretable until both are re-derived**, not inverted: we do
not currently know which is larger, and asserting that on-route deviation is
lower would replace one unsupported claim with another.

**`d_Fp` is unverified.** `planned_path()` uses only the last `Trajectory`
message of each bag. Two runs that stopped at different points produce
last-plans beginning at different ego positions — an analogous artefact by a
different mechanism, which arc-length truncation does not fully remove because
the plans never shared an origin. `compare_runs.py` does not compute `d_Fp`,
so 1.998 m has never been checked against a second implementation. The patched
script now emits `plan_origin_gap_m` to quantify the suspicion.

### 4.3 What survives, and is a genuine finding

One result does not depend on the artefact, because it is a comparison between
the *map* and the *behaviour*, not between two trajectories:

| Quantity | Value |
|---|---|
| lanelet 3002013 width | 3.058 → 6.639 m (**+117.1%**) |
| its centreline displacement | **2.79 m** |
| resulting driven deviation | **0.051 m** |

A 2.79 m displacement of the geometric centreline moved the driven path by
five centimetres. **Autoware's planner optimises within the drivable area
rather than tracking the geometric centre**, so widening a lane does not, by
itself, move the vehicle — it enlarges the space the planner is free to
optimise inside, and the planner's existing objective keeps it near its
previous line.

This is why widening attacks are weak against this planner, and it explains
the shape of §3's detection results from the other direction: the attacks
easiest to *detect* geometrically are not the ones with the largest
behavioural effect, and vice versa. It also predicts why Sato et al. needed an
explicit centreline to steer the vehicle rather than relying on boundary
displacement alone.

The finding is real. It is not a headline, because it is a negative result
about attack efficacy on one route on one planner build.

### 4.4 Large displacements fail by localisation, not planning

At 4.5 m and above the ego could not initialise on the tampered map: the start
pose, computed from the clean centreline, no longer fell inside the displaced
drivable area. The vehicle never moves. Sato et al. observed a related failure
at 5.0 m, attributed to infeasible planning; our mechanism differs
(localisation rather than planning) and we do not claim to reproduce theirs.

This observation does not depend on Fréchet distance and is unaffected by the
artefact. It should still be re-checked on 0.50.0.

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
single-lanelet width tampering (recall 1.000 above 1 m). Both are catchable
because they present a large deviation at a single location.

Gradual width ramps are not detectable by geometric analysis of a single map
snapshot. Their per-step magnitude sits below the honest variation of a real
urban map, so per-lanelet thresholds cannot separate them; their sequence
signature ranks below ordinary intersection geometry, so chain statistics
cannot either — the measured ceiling is F1 0.220. The external witness that
would resolve the ambiguity does not carry the attribute: OSM tags road width
on 0.3–5.9% of ways. Repair inherits the same failure, because it cannot
repair what it cannot locate.

**The principal contribution is that map integrity verification must be
differential rather than absolute.** Absolute verification asks whether a map
is plausible, and §1–§5 measure how far that question can be pushed before it
stops separating tampering from ordinary geometry. Differential verification
asks whether a map is *the same map as before*, and under a supply-chain
threat model — where the adversary controls the delivered map but not its
publication history — that question is answerable. It detects every attack
class we can generate, including the two that defeat single-snapshot detection
entirely, and including one that changes no geometric value at all (§8).

### Withdrawn claim

An earlier draft stated:

> There exists a band — roughly 2.0 m to 4.5 m of cumulative displacement
> spread across a route — in which the attack exceeds the safety threshold
> (`d_Fe` = 1.414 m against 0.5 m) while remaining below the detection floor
> of every method evaluated. Characterising that band … is this work's
> principal contribution.

**This is withdrawn.** The 1.414 m figure was an endpoint artefact (§4.1). The
corrected value for that configuration is 0.051 m, and no measured condition
exceeds the 0.5 m threshold: 2.0 m gives 0.204 m (itself artefact-bearing),
3.0 m gives 0.051 m, and 4.5 m and above fail to localise rather than
deviating. The band as described has no measured support.

The *detection* half of the claim stands and is unaffected — §3 and §5 are
derived from map files, not from trajectories. What does not stand is the
assertion that an undetectable attack was simultaneously shown to be unsafe on
this route. Whether such a band exists on this planner is an open question, and
§4.3 gives a mechanism suggesting it may not for width-widening attacks
specifically.

Recording this withdrawal rather than quietly deleting the claim is
deliberate: the artefact was found by our own diagnostic, and the diagnostic is
now in the tool.

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
- All simulation figures come from Autoware 0.52.0 while the reference build is
  0.50.0. One finding has already failed to reproduce across the two (§8.4).
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
limit, direction, and connectivity field-by-field, then groups flagged
lanelets into connected runs. Exit code 2 on REJECT.

### 8.2 Results

| Attack map | Changes | TP | FP | FN | Precision | Recall | F1 | Verdict |
|---|---|---|---|---|---|---|---|---|
| `route_g3.0.osm` (width ramp) | 33 | 8 | 10 | 0 | 0.444 | **1.000** | 0.615 | **REJECT** |
| `centerline_attack.osm` | 8 | 8 | 0 | 0 | **1.000** | **1.000** | **1.000** | **REJECT** |
| clean vs clean (control) | 0 | — | — | — | — | — | — | ACCEPT |

Lanelet counts were 979 → 979 in both attack cases (+0 / −0): the attacks
change geometry and membership, not the lanelet inventory. The width-ramp case
grouped its flags into 7 coordinated runs, and classified 33 changes as 0
accepted / 14 suspect / 19 rejected.

Compare against the same attacks under single-snapshot detection: `width_ramp`
at this magnitude is caught at 0.201–0.640 recall (§3), and the centreline
injection is caught at 0.000 by field comparison alone (§8.3). Differential
verification catches both at recall 1.000.

**On the 10 false positives.** Those lanelets share boundary nodes with
tampered ones, so displacing a boundary genuinely moved their geometry too.
The label file records only what the injector explicitly targeted. We report
strict precision 0.444 and do not adjust it: an argument that the false
positives are really true positives would improve the number without improving
the method, and the shared-node coupling is a property of Lanelet2 that any
deployment would face.

### 8.3 `structural_diff()` — the change that mattered

The first version compared field values only and missed centreline injection
**entirely, at recall 0.000**. The attack adds an explicit `role="centerline"`
member to the relation. The boundaries are untouched, so every geometric field
compared — width, computed centre, speed, direction, connectivity — is
byte-identical between versions. Nothing in a field diff can see it.

Comparing *which member roles exist* catches it at precision 1.000. This is
the general lesson: an attack can be invisible in every value you compare
while being trivially visible in the structure you did not think to compare.
It generalises beyond centrelines to any Lanelet2 regulatory element that can
be added, removed or re-roled without moving a coordinate.

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
another, cause unresolved.

**Detection is unaffected.** Differential verification catches centreline
injection at precision 1.000, recall 1.000, on both builds, because it
inspects the map file rather than the planner's response to it. That the
downstream effect is version-dependent is precisely the argument for detecting
tampering at the ingest boundary instead of waiting to observe its
consequences.

### 8.5 What differential verification does not do

- **It requires an authentic prior version.** First-time provisioning has no
  reference; a compromised initial delivery is undetectable by this method.
  Bootstrapping is out of scope here.
- **It flags change, not malice.** A legitimate map update also changes
  geometry. The severity grouping and coordinated-run analysis separate
  scattered edits from coordinated ones, but the final accept/reject decision
  on a genuine update needs an authenticated publisher, which is an
  orthogonal mechanism.
- **It does not measure impact.** A REJECT verdict says the map differs
  coherently from its predecessor, not that driving it would be unsafe. §4.3
  shows those are genuinely different questions: a 2.79 m centreline
  displacement moved the vehicle 0.051 m.
- **Five attack types remain unexercised.** `speed_spoof`, `oneway_flip`,
  `connectivity_break`, `tunnel_bridge_flip` and `width_step` have code paths
  that would fire but have never been run. Completing that table is Priority 3.
  Until it is done, "detects every attack class we can generate" is supported
  by two of seven.

Note that `oneway_flip` (0.021) and `connectivity_break` (0.120) are the two
attacks single-snapshot detection handles worst, because they are topological
attacks being hunted with geometric features. Both are exact-value changes
between versions, so differential verification is expected to catch them at
REJECT severity — but expected is not measured, and §8.2 has two rows, not
seven.