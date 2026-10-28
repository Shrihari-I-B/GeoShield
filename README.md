# GeoShield

A map-integrity gate for autonomous navigation.

GeoShield sits at the ingest boundary between an HD map file and Autoware's
planning stack. It inspects a candidate map before the planner sees it, refuses
maps that fail integrity checks, and writes a repaired map the planner can use
instead. It never touches sensor data.

Extends Sato et al., USENIX VehicleSec 2025, which demonstrated HD map
tampering attacks and proposed no defence.

## What is demonstrated

Two attack families injected into the Nishi-Shinjuku Lanelet2 map and driven
end-to-end in the Autoware planning simulator, each in four conditions, one
system configuration throughout.

### Safety impact (Track B)

Discrete Frechet distance between the recorded trajectory and the clean-map
reference, metres, arc-length truncated to strip start/stop artefacts.
Route: lanelet 3012234 to 3002007, 393.2 m.

| condition | d_Fe (m) | outcome |
|---|---|---|
| clean vs clean (repeat) | 0.0314 | completes |
| **width ramp** | **1.4454** | completes, `boundary_departure_critical` |
| width ramp + GeoShield | 0.0516 | completes |
| **centreline injection** | **0.5500** | MRM `emergency_stop`, 6.8 m short of goal |
| centreline injection + GeoShield | 0.0504 | completes, no MRM |
| clean + GeoShield (false-positive control) | 0.0492 | completes |

Run-to-run noise over six non-attack runs: 0.0302 to 0.0516 m, mean 0.0408,
sd 0.0098. The weaker attack is 11x the worst non-attack run, the stronger 28x.
The width-ramp figure reproduced to 2.1 mm across a one-week gap
(1.4475 m, then 1.4454 m).

### Detection

`differential_verify.py`, candidate against trusted prior.

| map | flagged | TP | FP | recall | precision |
|---|---|---|---|---|---|
| width ramp, 8 tampered lanelets | 17 | 8 | 9 | 1.000 | 0.471 |
| centreline injection, 8 tampered lanelets | 8 | 8 | 0 | 1.000 | 1.000 |
| clean control | 0 | - | 0 | - | ACCEPT |

The rejected alternative, for the ablation: gating on the `chain_trend_R1`
feature at p95 gives recall 0.875, precision 0.039, and clamps 163 lanelets on
an untampered map. No percentile fixes it; recall reaches 0 at p99.5 while 25
clean lanelets are still flagged. A percentile of a per-chain statistic flags
a fixed fraction of chains by construction.

### Repair

Exact restoration of flagged geometry from the trusted prior.

- Width ramp: 176 boundary nodes restored, 404 matched unchanged. The repaired
  map's node coordinates are identical to the prior, 0 of 0 differing.
- Centreline injection: 8 injected relation members removed, then 8 orphaned
  ways and 89 orphaned nodes. Way and node counts match the prior exactly.

Restoring unmodified geometry is a no-op, so repair is idempotent and repair
precision is irrelevant. Only recall matters.

Orphan removal is not cosmetic. Leaving the 8 orphaned ways and 89 nodes in
place produced a repeating `mrm_comfortable_stop_operator` that the clean
control did not show; they were the only difference between the two maps.

## Two findings

**The reference frame is the attack surface.** The width ramp is the more
dangerous family despite being the cruder one. Moving lane boundaries moves the
lane, so lane-keeping does not object: the vehicle tracks a corrupted centre
faithfully and reports no error. An attack that corrupts the reference frame
defeats every check defined in that frame. The centreline injection, which
leaves boundaries intact, trips Autoware's own boundary-departure monitor and
an MRM, and is therefore the safer attack from the victim's point of view.

**Body departure precedes path departure.** At the centreline stop the
commanded lateral offset was 0.864 m, inside the 1.529 m half-lane. Adding the
0.948 m vehicle half-width puts the body 0.282 m past the boundary. Body
departure begins at 0.581 m of commanded offset, 2.6x more sensitive than path
departure. A defence evaluated on path error alone understates the hazard.

## What is not demonstrated

Read this before quoting any number above.

- **The Track B gate is differential, not self-consistent.** Every figure in
  the tables comes from comparison against a trusted prior map. The project's
  thesis claim, self-consistency needing no external witness, lives in
  `tier1_rules.py` and `tier2_iforest.py` and is evaluated on Track A. It is
  not what gated these drives. The two modes rest on different assumptions and
  are reported separately. A trusted prior is unavailable on first map load and
  in a new region, which is where self-consistency is the only option.
- **No ROS 2 node.** The gate is command-line scripts run against files by
  hand. The ingest-boundary claim is not yet demonstrated by a process.
- **No no-prior repair policy.** Repair restores from the prior.
- **Tier 3 Overpass cross-verification is not in the Track B path.**
- **One map, one route.** Track B is Nishi-Shinjuku, one 393.2 m corridor.
- **Coherence gap.** `build_dataset.py` models a uniform width shift in memory
  while the written maps are tapered at the joins with untampered road. Track A
  features and Track B maps therefore do not describe the same attack.
- **Centreline recall is structural, not geometric.** The arc-length ramp
  concentrates displacement: five of the eight tampered lanelets shift by
  0.111 m or less, and the last three carry 89% of the 1.0 m total. Those five
  are caught because a `centerline` member exists where the prior had none, not
  because an 11 cm anomaly was resolved. Recall 1.000 on this family is a
  structural result. The smallest geometrically detectable shift is unmeasured.
  A constant-shift variant with the ramp removed would measure it and belongs
  in the ablation.

## Superseded results

`RESULTS.md` and `RESULTS_UPDATE.md` contain Track B trajectory figures
recorded before 2026-10-26 that are invalid.

`write_tampered_map()` displaced nodes in the Lanelet2 `local_x` and `local_y`
tags only. Autoware, finding no `map_projector_info.yaml`, defaults to MGRS and
projects from the `lat` and `lon` attributes, which were never written. A
node-level diff settled it: 36936 nodes compared, 178 differing in
`local_x`/`local_y`, **0 differing in lat/lon**. Every tampered drive before
that date was a clean drive. The 0.0402 m quoted in those files is the
run-to-run noise floor, not an attack effect.

The same defect was present in `repair.py`, where it would have shown GeoShield
failing to repair a map it had repaired correctly, and twice in
`centerline_attack.py` as `lat="0" lon="0"` and `ele="0"` on created nodes. All
coordinate writes now go through `osm_geom.py`.

The bug was findable only because a run-to-run noise floor was measured first.
It had never been measured. `demo.sh` was deleted rather than updated because
it asserted the 0.0402 m figure as an attack effect.

Figures in this README supersede both files.

## Reproducing

    bash reproduce.sh detect     # no ROS, no Docker, no simulator
    bash reproduce.sh drive      # operator checklist for the six Autoware runs

The split is deliberate. `detect` builds both attack maps, runs verification
and repair, and checks the repaired maps against the prior, on any machine with
Python. The detection pipeline does not depend on the simulator working.
`drive` cannot be fully automated: Autoware permits one recording per launch,
and the map argument changes between conditions.

`detect` compares its regenerated maps against the archived ones geometrically
rather than by hash. lat/lon is written to twelve decimals while
`local_x`/`local_y` carry 0.1 mm, so the last digits are invented precision: the
archived `route_cl1.0.osm` differs from a fresh run by one unit in the last
place of longitude on three of its 89 created nodes, about 1e-7 m, with local
coordinates bit-identical.

## Layout

| path | role |
|---|---|
| `osm_geom.py` | coordinate calibration and writes, single source of truth |
| `attack_injector.py` | width-ramp family |
| `centerline_attack.py` | centreline-injection family |
| `differential_verify.py` | detection against a trusted prior |
| `repair_differential.py` | restoration from a trusted prior |
| `tier1_rules.py`, `tier2_iforest.py` | no-prior self-consistency checks |
| `osm_adapter.py`, `lanelet2_adapter.py` | normalise to the `RoadSegment` schema |
| `scenario_runner.py` | drives a route in Autoware and records a verified bag |
| `frechet_analysis.py` | trajectory comparison |
| `publish_flags.py` | RViz MarkerArray for flagged lanelets |
| `ARCHITECTURE_BLUEPRINT.md` | design intent, including unmet acceptance criteria |

`scenario_runner.py` must run inside the Autoware container; it refuses to run
on the host, where `autoware_adapi_v1_msgs` is absent and engage fails with
"The passed service type is invalid".

## Reference

Sato et al., *HD map tampering attacks on autonomous driving*, USENIX
VehicleSec 2025. The paper is not redistributed in this repository.
