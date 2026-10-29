# GeoShield

A map-integrity gate for autonomous navigation.

GeoShield sits at the ingest boundary between an HD map file and Autoware's
planning stack. It inspects a candidate map before the planner sees it, refuses
or repairs what fails, and admits the result. It never touches sensor data.

Extends Sato et al., USENIX VehicleSec 2025, which demonstrated HD map
tampering attacks and proposed no defence.

## What it claims

Two claims, both measured, neither assumed:

1. **Lane width is witnessed by neither OpenStreetMap nor within-map
   self-consistency.** Cross-verification's recall ceiling on width is
   **0.000** — not one of 844 geometrically matched lanelets carries a width
   tag in OSM — and eight independent detection approaches, rule-based and
   learned, fail to separate a width ramp from clean geometry.
2. **It is therefore recoverable only against a trusted prior**, which
   GeoShield does with exact repair and a measured safety impact.

The reason underneath both: a detector with a non-zero clean-map rate emits
counts that cannot be interpreted without a baseline. **The prior is that
baseline.**

## Reproducing this

Regenerates both attack families from the clean map, asserts every detection
and repair figure below, checks eight gate decisions by exit code, and
re-derives the Track B safety figures from the bags that produced them. No
ROS, no simulator, no network, no GPU.

| command | what it proves |
|---|---|
| `bash reproduce.sh detect` | detection and repair; every figure asserted, not printed |
| `bash reproduce.sh gate` | eight gate decisions by exit code, with and without a trusted prior |
| `bash reproduce.sh archive` | the nine Track B `d_Fe` figures, re-derived from their own bags |
| `bash reproduce.sh drive` | checklist for recording the bags from scratch, seven launches |
| `bash reproduce.sh all` | the first three, then the drive if its recordings exist |

A moved figure exits non-zero and names itself. Every `results/fr_*.json`
records the two bags it read, so `archive` needs no hardcoded list and
re-derives all nine to 0.0000 m. The drive is a checklist rather than a
pipeline because Autoware permits one recording per launch and the map
argument changes between conditions.

## Safety impact (Track B)

Discrete Fréchet distance between the recorded trajectory and the clean-map
reference, arc-length truncated. Autoware 0.50.0-docker, route 3012234 →
3002007, 393.2 m.

| condition | d_Fe (m) | outcome |
|---|---|---|
| clean vs clean (repeat) | 0.0314 | completes |
| **width ramp** | **1.4454** | completes, `boundary_departure_critical` |
| width ramp + GeoShield | 0.0516 | completes |
| **centreline injection** | **0.5500** | MRM `emergency_stop`, 6.8 m short of goal |
| centreline injection + GeoShield | 0.0504 | completes, no MRM |
| clean + GeoShield (false-positive control) | 0.0492 | completes |

Seven non-attack comparisons span **0.0302–0.0516 m**; the weaker attack is
**11×** the worst of them. The width-ramp figure reproduced to 2.1 mm across a
one-week gap.

## Detection, against a trusted prior

| attack | N | verdict | precision | recall |
|---|---|---|---|---|
| clean control | 0 | ACCEPT | — | 0 false alarms |
| oneway_flip | 8 | REJECT | **1.000** | **1.000** |
| speed_spoof | 8 | REJECT | **1.000** | 0.875 |
| centerline_injection | 1 | REJECT | 1.000 | 1.000 |
| width_step | 8 | REVIEW | 0.714 | 0.625 |
| connectivity_break | 8 | REJECT | 0.571 | 1.000 |
| width_ramp | 8 | REJECT | 0.471 | 1.000 |

### Detection floor

| intended gain | realised | verdict | recall |
|---|---|---|---|
| +0.25 m | 0.218 | REVIEW | 0.375 |
| **+0.50 m** — Sato's smallest effective attack | 0.436 | **REVIEW** | **0.750** |
| +0.75 m | 0.655 | **REJECT** | 0.875 |
| +3.00 m | 2.623 | REJECT | 1.000 |

**GeoShield refuses width tampering from +0.75 m, and escalates Sato's +0.5 m
attack for review at recall 0.750.** Realisation holds at 0.874 across every
magnitude, so the floor is the detector's, not the injector's.

Precision *falls* as the attack grows (1.000 → 0.471) because larger
displacements propagate through shared boundary nodes into lanelets the label
set does not name. Those extra flags are real modifications, so 0.471 is
pessimistic rather than accurate.

## The blind spot, measured

OSM width coverage across four cities:

| region | drivable ways | with `width` |
|---|---|---|
| berlin-mitte | 1817 | 27.4% |
| munich-centre | 2496 | 5.9% |
| **nishi-shinjuku** | 782 | **0.3%** |
| sf-soma | 1552 | **0.0%** |

Cross-verification against OSM on the evaluation region, 844 of 979 lanelets
matched geometrically at 2.9 m median:

| attribute | ceiling | clean-map disagreement |
|---|---|---|
| oneway | 0.718 | 8.8% |
| lanes | 0.640 | 98.1% (semantic mismatch: lane-level vs carriageway) |
| speed_limit | 0.246 | 31.5% |
| **width** | **0.000** | — |

Cross-verification compares **two different artefacts**, so a disagreement is
ambiguous between tampering, HD-map error and stale OSM. Differential
verification compares the same artefact against its own earlier self, so a
disagreement is unambiguous. That, more than coverage, is why the prior is
required.

## Architecture

Two modes, selected by whether a trusted prior exists.

**With a prior** — differential verification alone, then exact restoration.
Optimal on every attack family tested; each added tier costs precision and
gains no recall. One exception: on `speed_spoof`, Tier 1 raises average
precision 0.879 → 0.925 by ranking a lanelet differential scores low.

**Without a prior** — Tier 1 minimal, two rules, clean-map FPR **0.0153** over all 979 lanelets, or **0.0196** over the 764 a neighbour rule can evaluate (see the 215-lanelet limitation below), no
calibration. Gates on `R5_attribute_neighbour` (clean-map FPR 0.0000) alone,
because only a rule that never fires on clean data can gate without a baseline.
Consensus repair where that rule licenses it; speed corrections proposed for
review, never written. Covers 1 of 4 families. Geometry uncovered.

**Tier 3** (Overpass cross-verification) is built and measured, and
deliberately not in the inference path — it is the baseline the design is
justified against. API-call reduction is therefore 100%.

**Tier 2** (Isolation Forest, 27 features) reaches PR-AUC 0.018 on width
against a base rate of 0.0082 and is the second most expensive stage. Retained
and measured; it earns no place in the gate.

### Latency

Once at map ingest, not per planning cycle.

| stage | |
|---|---|
| Tier 1 minimal (2 rules) | 0.00 s |
| Tier 3 match + coverage | 0.04 s |
| Tier 2 Isolation Forest | 0.91 s |
| repair_differential | 1.21 s |
| differential_verify | 1.66 s |
| **gate end to end** | **4.57 s** |

## The shared-node constraint

Consecutive Lanelet2 lanelets **share their boundary nodes**. Three findings
follow:

- A detector reading discontinuity *at a join* measures zero on any well-formed
  map, tampered or not. Three Tier 2 features were built that way and are
  identically zero on all 979 lanelets.
- An abrupt width step **cannot be written**. Isolated edits realise ~0.23 of
  their intended magnitude; on a 9.96 m lanelet, nothing at all.
- An attacker cannot make an isolated geometric edit stick — width tampering
  must be contiguous, so it necessarily has spatial extent.

## What this does not do

- **Width is invisible to every no-prior check tested** — eight approaches,
  rule-based and learned. The prior is not an optimisation.
- **Without a prior the gate refuses on 1 of 6 attack types tested.** Only
  `oneway_flip` trips a gating rule. `speed_spoof` is detected at the same
  conditional recall but through `R4_speed_neighbour`, which is advisory, so
  it never refuses a map. Geometry and topology are neither detected nor
  repaired.
- **Tier 1's conditional recall of 1.000 holds on two attack types, not six.**
  On `oneway_flip` and `speed_spoof` it flags all 6 of the tampered lanelets
  it can examine, missing exactly the 2 that fall below two neighbours. On
  `centerline_injection`, `connectivity_break`, `width_ramp` and `width_step`
  conditional recall is **0.000** — the two shipped rules read neither
  geometry nor width, so the coverage ceiling is not what limits them there.
- **Tier 1 cannot examine 215 of 979 lanelets** — those with fewer than two
  connected neighbours; coverage ceiling 0.780. An attacker targeting stubs
  and dead ends evades it entirely.
- **Tier 1's output is invariant under width tampering.** Thirteen maps with
  width-ramp total gains from 0.25 m to 6.0 m produce byte-identical findings
  — 15, the clean map's own. No choice of threshold can recover a signal that
  never enters the computation.
- **`R5_topology` was cut**, so connectivity breaks have no no-prior detection.
- **One map**, by decision. Generality is untested.
- The no-prior branch has not been exercised inside the Autoware container.

## Superseded results

`RESULTS.md` and `RESULTS_UPDATE.md` contain Track B figures recorded before
2026-10-19 that are invalid: `write_tampered_map` displaced nodes in
`local_x`/`local_y` only, while Autoware projects from `lat`/`lon`. 36 936
nodes compared, 178 differing in local coordinates, **0 in lat/lon** — every
"tampered" drive before that date was a clean drive. The 0.0402 m quoted there
is the run-to-run noise floor.

`results/superseded/` holds pre-fix evaluation artefacts and three Fréchet runs
excluded from the demo, each with its reason.

## Layout

| path | role |
|---|---|
| `geoshield_verifier.py` | the gate: verify, repair, admit or refuse, both modes |
| `launch/geoshield_gate.launch.py` | starts the planner only on an admitted map |
| `differential_verify.py` / `repair_differential.py` | the prior-based path |
| `repair_noprior.py` | consensus repair where rule precision licenses it |
| `tier1_rules.py` | two surviving rules; six cut, with reasons |
| `tier2_iforest.py` / `features.py` | learned anomaly detection |
| `tier3_osm_verify.py` | OSM cross-verification, matching and coverage |
| `fusion.py` | P_attack, and the ablation that superseded it |
| `attack_injector.py` / `centerline_attack.py` | the two attack families |
| `osm_geom.py` | coordinate writes, single source of truth |
| `magnitude_sweep.py` / `latency.py` | detection floor and stage timing |
| `demo.sh` / `reproduce.sh` | presentation and reproduction |

## Reference

Sato et al., *HD map tampering attacks on autonomous driving*, USENIX
VehicleSec 2025. The paper is not redistributed in this repository.
