# Architecture Evolution Blueprint: GeoShield

*HD map tampering attack-and-defence pipeline for autonomous navigation.
Final-year major project, KSIT Bengaluru (BCS786), extending Sato et al.,
USENIX VehicleSec 2025.*

**Scope note before anything else.** This is a research artefact, not a product.
Every "weakness" below is weighed against whether fixing it improves the
*claims* the project makes. Several standard engineering upgrades — CI/CD,
orchestration, service decomposition — would look impressive and change nothing
about whether the results are true. Those are flagged as such rather than
recommended by default.

---

## Current Foundation

### Runtime

| Layer | Value |
|---|---|
| Host | Ubuntu 24.04, Intel Core Ultra 5 125H (18 threads), Intel Arc iGPU, 16 GB RAM, Wayland |
| Simulator | Autoware **0.50.0**, `ghcr.io/autowarefoundation/autoware:universe-devel`, digest `sha256:405225eda6c05161bfde39cc7885511f3f4d9699d126891891420dd80c2e024a` |
| Image | 11.1 GB on disk, 2.62 GB content |
| ROS | ROS 2 Jazzy inside the container; host has Jazzy but **no Autoware message packages** |
| Analysis | Python 3.12, scikit-learn 1.9.0, matplotlib, mcap 1.4.0, mcap-ros2-support 0.5.7 |
| VCS | `github.com/Shrihari-I-B/GeoShield`, `main` |

### Container invocation, verified working

```bash
xhost +local:docker

docker run -it --rm --name autoware \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --group-add video \
  -v ~/autoware_map:/autoware_map \
  -v ~/Development/projects/geoshield:/geoshield \
  --net=host \
  ghcr.io/autowarefoundation/autoware:universe-devel bash
```

`--device=/dev/dri` plus `--group-add video` is load-bearing: without them RViz
falls back to llvmpipe (OpenGL 4.5, `failed to load driver: iris`); with them it
reaches the Arc GPU at **OpenGL 4.6**.

### Measured runtime characteristics

| Metric | Value |
|---|---|
| Node count | 135 |
| `/planning/trajectory` rate | **10.014 Hz**, σ 0.004 s |
| RViz | 26–31 fps |
| RAM in use | ~5 GB used, 9 GB free |
| Readiness gate | `until [ "$(ros2 node list \| wc -l)" -gt 100 ]; do sleep 10; done` |

The 10.014 Hz figure is why EC2 was retired: the laptop holds the planner's
target rate as well as an m7i.4xlarge did.

### Codebase

23 Python modules, all parsing, ~6,000 lines. Two tracks over one detection
core:

```
OSM (Overpass) ──┐
                 ├─► RoadSegment ─┬─► Tier 1 rules ─┐
Lanelet2 .osm ───┘                ├─► Tier 2 IForest ┼─► fusion ─► Tier 3 (gated by γ)
                                  └─► Differential ──┘
                                          │
                       tampered .osm ─► Autoware ─► rosbag ─► Fréchet
```

### What is working and verified

| Capability | Evidence |
|---|---|
| Attack injection, 8 types | seeded, reproducible, writes valid Lanelet2 XML |
| **Differential verification** | precision/recall **1.000/1.000** on speed_spoof, oneway_flip, centerline_injection; 1.000 recall on both width attacks; **0 false positives** on the clean map |
| End-to-end simulation | 8/8 tampered lanelets driven, both runs 411 m, `d_Fe` **0.0402 m** vs 0.5 m threshold |
| Detection evaluation | 29,370 examples, 955 positives, split by run |
| Local live visualisation | RViz, GPU-accelerated |

### Measured negative results — deliberate, not gaps

- Rule-based ceiling **F1 0.220** (separability analysis, chain lengths 3–7)
- Isolation Forest **0.133** vs a single hand-crafted feature at **0.420**
- Repair infeasible: precision ≤0.055 at every threshold; a mis-targeted repair
  broke ego localisation
- OSM width coverage 0.3% / 5.9% — no external witness for the key attribute

---

## Technical Debt & Weaknesses

### Severity 1 — affects the validity of claims

**No test suite whatsoever.** `tests/` exists and is empty. Every correctness
check in this project's history was a manual `grep -c` or a one-off diagnostic
script. Four measurement errors reached committed results before being caught:

| Error | Impact | Caught by |
|---|---|---|
| Endpoint artefact | `d_Fe` reported 1.414 m; true 0.040 m | inspecting the deviation curve |
| Incomplete exposure | 6/8 lanelets driven, reported as full | `route_overlap.py`, written after the fact |
| Centreline DoS | claimed as a finding, does not reproduce on 0.50.0 | running it on a second build |
| Label vs realised width | up to **1.23 m** divergence | differencing the maps by hand |

A regression test asserting `truncate_common()` on two collinear paths of
differing length returns 0.0 would have caught the first in seconds. It does
not exist.

**`write_tampered_map()` double-displaces shared boundary nodes.** Consecutive
lanelets share boundary nodes; the writer displaces each node once per claiming
lanelet. On an 8-lanelet chain, **7 of 178 nodes** are touched twice, and
realised width diverges from the label by up to 1.23 m in both directions.

This is the same mechanism as two earlier findings — repair overshoot by a
factor of 1.88, and differential-verification false positives on shared-node
neighbours. Three appearances, one root cause, never centralised.

**Two attack types cannot be evaluated end-to-end.** `write_tampered_map()`
handles `width_m`, `speed_limit_kph` and `oneway` only. `connectivity_break`
modifies `successors` in memory and the change never reaches the XML — the
written map is materially identical to the clean one, so ACCEPT was correct on
a map that was never tampered. `tunnel_bridge_flip` is inapplicable: the map
carries no tunnel or bridge tags.

**Cross-version topic drift is unmodelled.** 0.52.0 publishes
`/planning/trajectory`; 0.50.0 publishes `/planning/scenario_planning/trajectory`.
This has been patched by `sed` in both directions at least three times.
`d_Fp` is currently unmeasurable on the reference build for exactly this reason.

### Severity 2 — operational friction

**Manual file transfer as the deployment mechanism.** Corrected files were
delivered by browser download and hand-placed. Verified failures: four
occasions where a file silently did not land, producing byte-identical output
across three consecutive "fixed" runs. The `grep -c` habit exists because of
this and is compensating for a missing mechanism.

**Fixed-duration recording.** `scenario_runner.py` records for a wall-clock
window. At 90 s the vehicle covered 338.62 m of a 393.2 m route — the source of
both the endpoint artefact and the exposure gap. 150 s works for *this* route
and is not a general fix. `/api/routing/state` reports ARRIVED and is not polled.

**Environment-dependent bag format.** 0.52.0 recorded `.mcap`; 0.50.0 records
`.db3`. `frechet_analysis.py` now dispatches between backends, but `.db3` bags
are unreadable on the host, so analysis of new runs is container-bound while
analysis of old runs is not.

**No experiment provenance.** Results are `results/*.json` with filenames as
the only index. Reconstructing which build, duration, map and seed produced
`results_g3.0.json` requires reading the conversation. `--build` was added late
and is not retrofitted.

### Severity 3 — cosmetic, listed for completeness

- Diagnostics (`topo_diag.py`, `verify_run.py`, `separability.py`,
  `route_overlap.py`) sit beside pipeline code with no separation
- `make_demo_figure.py` is superseded and still present, and still produces the
  uncorrected 1.414 m figure
- `data/bags/clean_run_good/` contains `tampered_run1_0.mcap` — directory
  renamed, file not
- Two dead features retained in `features.py` (`cl_jump_succ`,
  `boundary_asym_succ`), structurally always zero

### Explicitly NOT weaknesses

Listed so a refactor does not "fix" them:

| Apparent problem | Why it stays |
|---|---|
| Isolation Forest underperforms a single feature | Measured ablation result, reported deliberately |
| Repair module doesn't work | Evaluated and found infeasible; the finding is the output |
| Only one map, one route | Nishi-Shinjuku is Sato's map — comparability is the point |
| No GraphSAGE, no HMM | Both cut with evidence; do not reintroduce |
| Low precision on width attacks (0.143, 0.444) | Flagged neighbours genuinely changed geometry; incomplete labels, not false alarms |

---

## The 'Next Level' Upgrade Plan

Ordered by effect on the project's claims, not by engineering appeal.

### Upgrade 1 — Property-based regression suite

**Problem it solves:** four measurement errors reached committed results. All
four are expressible as invariants.

**Design.** `pytest`, synthetic fixtures, no simulator, target under 30 s total.

```python
# tests/test_metrics.py
def test_truncation_removes_length_artefact():
    """Two collinear paths differing only in length must give d_Fe == 0."""
    a = [(i * 1.0, 0.0) for i in range(101)]
    b = [(i * 1.0, 0.0) for i in range(120)]
    ta, tb, _ = truncate_common(a, b)
    assert frechet(resample(ta, 100), resample(tb, 100)) < 1e-6

def test_parallel_offset_recovered_exactly():
    """A pure lateral offset of d must measure d."""
    a = [(i * 1.0, 0.0) for i in range(100)]
    b = [(i * 1.0, 0.5) for i in range(100)]
    assert abs(frechet(a, b) - 0.5) < 1e-9

# tests/test_injector.py
def test_realised_width_matches_label():
    """Written XML must contain the widths the label records.
    Currently FAILS: shared seam nodes are displaced twice (up to 1.23 m)."""
    ...

def test_no_shared_nodes_silently_double_displaced():
    """Injecting a chain must warn when target lanelets share boundary nodes."""
    ...

# tests/test_differential.py
def test_clean_vs_clean_is_accept():
    """Identical maps must produce zero changes and ACCEPT."""
    ...

def test_structural_addition_detected():
    """Adding a centerline member with zero geometric change must REJECT."""
    ...
```

The injector tests **fail today**. That is the point — they encode known bugs
as executable specifications rather than prose in a handoff document.

**Effort:** 1 day. **Payoff:** every future refactor is guarded.

---

### Upgrade 2 — Fix the seam bug at its root, and make it visible

**Problem it solves:** the same shared-node coupling has produced three
separate defects. Fix once, centrally.

**Design.** Replace per-lanelet displacement in `write_tampered_map()` with a
two-pass node-level solve:

```python
# Pass 1: accumulate the requested displacement per NODE, not per lanelet.
#         A node claimed by two lanelets gets one entry, not two applications.
node_shift: dict[int, tuple[float, float]] = {}
for lid, target_width in plan.items():
    for nid in left_boundary(lid):
        dx, dy = normal_at(nid) * required_shift(lid)
        prev = node_shift.get(nid)
        # Shared seam: average the two claims rather than applying both.
        node_shift[nid] = (dx, dy) if prev is None else midpoint(prev, (dx, dy))

# Pass 2: apply once per node, then MEASURE and report realised vs intended.
apply(node_shift)
realised = measure_widths(tree)
report_divergence(plan, realised)   # never silent again
```

Extend the same function with a `successors` branch so `connectivity_break`
becomes evaluable — detaching a boundary way rather than deleting a member
reference, since Lanelet2 encodes connectivity through shared endpoints.

**Effort:** 1 day. **Payoff:** label files become ground truth; one more attack
type becomes measurable; three known defects collapse into one fixed mechanism.

---

### Upgrade 3 — Experiment harness with provenance

**Problem it solves:** results are unindexed files; build, duration, map and
seed are recoverable only from conversation history. Reproducing a number
requires archaeology.

**Design.** A declarative experiment spec plus a runner that captures context.

```yaml
# experiments/g3_reference.yaml
name: width_ramp_3m_reference
autoware_build: "0.50.0-docker"
image_digest: "sha256:405225eda6c05161bfde39cc7885511f3f4d9699d126891891420dd80c2e024a"
map:
  base: nishishinjuku_autoware_map
  attack: {type: width_ramp, total_gain: 3.0, seed: 7,
           targets: [3012234, 3013054, 3013093, 3012977,
                     3002017, 3013032, 3002007, 3002013]}
scenario: data/scenario_clean.json
recording: {mode: until_arrived, timeout_s: 300}
baseline: clean_0.50.0
metrics: [d_Fe, d_Fp, exposure_fraction, per_lanelet_realised_width]
```

The runner emits a manifest containing the image digest, git SHA, topic names
resolved at runtime, realised widths measured from the written map, and the
metrics. **Every result becomes self-describing.**

Two mechanisms fold in here:

- **Arrival polling** replaces fixed duration. Poll `/api/routing/state` for
  ARRIVED with a timeout fallback. Removes the endpoint artefact and the
  exposure gap *at source* rather than post hoc.
- **Topic resolution at runtime** replaces `sed`. Query `ros2 topic list`,
  match against a known-alias table, record which was used.

**Effort:** 2 days. **Payoff:** experiments become re-runnable by specification;
cross-build comparison becomes structurally impossible to do by accident.

---

### Upgrade 4 — Complete the defence architecture with Merkle tile signing

**Problem it solves:** differential verification proves a change is *plausible*.
It cannot prove the previous version was *authentic*. That gap is currently
stated as a limitation in §8.5 with no mitigation.

**Design.** Tile the map, hash each tile, build a Merkle tree, sign the root.
The vehicle verifies a tile with an O(log n) proof rather than re-hashing the
map. This is the established approach for vehicular integrity precisely because
it allows local verification with minimal server interaction — and unlike
blockchain-based auditing, it does not carry ≥300 ms per-transaction latency,
which rules blockchain out for a latency-sensitive AV path.

```
Layer                What it proves              What it misses
─────────────────────────────────────────────────────────────────
Merkle + signature   provenance — who sent it    a compromised supplier
                                                  signs tampered data
Differential         plausibility — is this      needs an authentic
                     change justified?            prior version
Tier 1 + Tier 2      absolute anomaly            F1 ceiling 0.220
                     (no prior version)
```

Each layer covers the one above's blind spot. Currently the project ships the
bottom two.

**Measurable outputs:** proof size vs tile count, verification latency per
tile, and the demonstration that a signed-but-tampered tile passes signature
verification and is caught by differential — which is the argument for needing
both.

**Effort:** 1 day for a `hashlib`-only demonstrator. **Payoff:** the defence
architecture becomes complete rather than partial, and it closes a limitation
the project currently states about itself.

---

### Considered and NOT recommended

| Upgrade | Why not |
|---|---|
| CI/CD pipeline | Nothing to deploy. Value is running the tests from Upgrade 1, which `pytest` on a pre-commit hook achieves without infrastructure. |
| Kubernetes / Compose orchestration | One container on one laptop. Orchestration solves a problem that does not exist here. |
| Microservice decomposition | The pipeline is a batch DAG, not a request path. Decomposition adds IPC failure modes to code that currently cannot fail that way. |
| Rewriting analysis in C++/Rust | Fréchet on 400 resampled points is milliseconds. Nothing is compute-bound except chain enumeration, which is capped at 20,000 and takes seconds. |
| More ML | Ceiling measured at F1 0.220 across seven feature formulations. The problem is the evidence, not the estimator. |

---

## Agentic Execution Tasks

Tasks are grouped by wave. Within a wave they touch disjoint files and can run
in parallel. Each carries an explicit acceptance test.

### Wave 1 — independent, parallelisable now

**A1 · Metric regression tests**
Files: `tests/test_metrics.py` (new). Reads: `frechet_analysis.py`,
`compare_runs.py`.
Write property tests for `truncate_common`, `frechet`, `resample`,
`arc_lengths`. Must include: collinear paths of differing length → 0.0;
parallel offset d → d; identical paths → 0.0; and a regression asserting the
two implementations agree within 7 mm on the same input (measured: 0.0402 raw
vs 0.0334 resampled).
**Accept:** `pytest tests/test_metrics.py` green, under 5 s, no bag files.

**A2 · Injector seam detector**
Files: `attack_injector.py` (append only, no behaviour change).
Add `detect_shared_seams(map_path, target_ids) -> dict[node_id, count]` and
call it at the end of `write_tampered_map()`, printing a warning when any count
exceeds 1.
**Accept:** on `route_g3.0` targets, reports exactly **7** doubly-claimed nodes
out of 178. Existing outputs byte-identical.

**A3 · Differential verification test suite**
Files: `tests/test_differential.py` (new). Reads: `differential_verify.py`.
Cover: clean-vs-clean → ACCEPT with zero changes; structural addition with zero
geometric change → REJECT; each of the five working attack types → recall 1.000.
**Accept:** green against existing `data/dv_*.osm` fixtures.

**A4 · Repository hygiene**
Files: move `topo_diag.py`, `verify_run.py`, `separability.py`,
`route_overlap.py` → `diagnostics/`. Delete `make_demo_figure.py` (superseded,
emits the uncorrected 1.414 m). Rename the stale `.mcap` inside
`clean_run_good/`. Remove the two dead features from `features.py` with a
comment recording why they are structurally zero.
**Accept:** all imports resolve; `differential_verify.py` and
`frechet_analysis.py` reproduce their current outputs exactly.

**A5 · Topic alias resolution**
Files: `scenario_runner.py`, `frechet_analysis.py`.
Replace hardcoded topic constants with runtime resolution against an alias
table: `{trajectory: ["/planning/trajectory",
"/planning/scenario_planning/trajectory"]}`. Record the resolved name in the
run manifest.
**Accept:** runs unmodified on both 0.50.0 and 0.52.0 bags; no `sed` required
to switch builds.

### Wave 2 — depends on Wave 1

**B1 · Node-level displacement solve** *(needs A1, A2)*
Files: `write_tampered_map()` in `attack_injector.py`.
Replace per-lanelet application with the two-pass node accumulation above.
Report realised vs intended width for every target.
**Accept:** for `route_g3.0`, realised width matches label within **±0.05 m**
for all 8 lanelets, against a current worst case of 1.23 m. The A1 test
`test_realised_width_matches_label` flips from fail to pass.

**B2 · Connectivity writer** *(needs B1)*
Files: `attack_injector.py`.
Add a `successors` branch that detaches a boundary way to sever a connection.
**Accept:** `differential_verify.py --candidate dv_connectivity_break.osm`
reports a non-zero edge delta and REJECT, replacing today's ACCEPT on an
effectively unmodified map.

**B3 · Arrival polling** *(needs A5)*
Files: `scenario_runner.py`.
Replace `time.sleep(duration)` with a poll of `/api/routing/state` for ARRIVED,
timeout fallback, and record which terminated the run.
**Accept:** on the reference scenario, terminates on arrival at roughly 110 s
rather than a fixed 150 s, and reports exposure 8/8.

### Wave 3 — depends on Wave 2

**C1 · Experiment harness** *(needs A5, B3)*
Files: `experiments/*.yaml`, `run_experiment.py` (new).
Declarative spec → run → manifest capturing image digest, git SHA, resolved
topics, realised widths, metrics.
**Accept:** `run_experiment.py experiments/g3_reference.yaml` reproduces
`d_Fe = 0.0402 ± 0.005` from a clean checkout with no manual steps.

**C2 · Merkle tile signing** *(independent of B, needs A1 for test scaffolding)*
Files: `tile_integrity.py` (new), `tests/test_tile_integrity.py`.
Tile by bounding box, hash tiles, build the tree, sign the root, verify with an
inclusion proof. `hashlib` only.
**Accept:** modifying one lanelet invalidates exactly one tile's proof; report
proof size and verification latency against tile count; demonstrate that a
signed-but-tampered tile passes signature verification and is caught by
`differential_verify.py`.

**C3 · Re-derive the full `d_Fe` table on the reference build** *(needs C1)*
Re-run the off-route, 2.0 m, 4.5 m and 6.0 m conditions on 0.50.0 with arrival
polling, replacing the artefact-bearing 0.52.0 rows in §4.6.
**Accept:** every row in RESULTS §4.6 carries a build label and a manifest
reference; no row remains marked artefact-bearing.

### Wave 4 — presentation

**D1 · `geoshield_verifier` ROS 2 node**
Files: `ros2_ws/src/geoshield_verifier/`.
Subscribe `/map/vector_map` and `/localization/kinematic_state`; publish
`/geoshield/integrity_status` and `/geoshield/flagged_markers` (red overlays on
rejected lanelets). Thin wrapper — detection logic already exists.
**Accept:** loading a tampered map turns the status TAMPERED and renders red
markers in RViz within 2 s.

**D2 · Demo automation**
Files: `demo/run_demo.sh`.
Scripted sequence: clean launch → drive → tampered launch → drive →
differential REJECT. Records video throughout.
**Accept:** single command produces the full sequence plus a fallback recording.

---

## Dependency graph

```
Wave 1   A1   A2   A3   A4   A5        ← all parallel, start immediately
              │              │
Wave 2        B1 ──► B2      B3        ← B1 needs A1+A2; B3 needs A5
              │      │       │
Wave 3        └──────┴──► C1 ◄┘   C2   ← C2 parallel with C1
                          │
                          C3
Wave 4                              D1   D2   ← parallel, after C1
```

**Critical path:** A2 → B1 → B2 → C1 → C3. Roughly 5 days sequential; Wave 1's
five tasks compress to one day in parallel.

---

## Migration guardrails

Constraints an autonomous agent must not violate. Each is a decision made with
evidence in this project's history.

1. **Autoware 0.50.0 Docker is the reference build.** Pinned by digest. EC2
   (0.52.0) results are secondary and must carry build labels.
2. **Do not reintroduce AWSIM, GraphSAGE, or HMM.** All three were rejected
   with measurements. AWSIM needs NVIDIA/Vulkan and measures perception, which
   this study does not examine.
3. **Do not "fix" the negative results.** Isolation Forest underperforming a
   single feature, repair infeasibility, and the F1 0.220 ceiling are findings.
4. **Tiers 1 and 2 run in parallel, never cascaded.** A cascade caps total
   recall at Tier 1's recall.
5. **Isolation Forest fits on clean maps only.** Novelty detection. Fitting on
   the mixed set collapses detection.
6. **Train/test splits are by run, never by segment.**
7. **Report PR-AUC, never accuracy.** Positives are ~3%.
8. **The HD map is the artefact under test; OSM is the independent witness.**
   Never both.
9. **Any change to metric code must keep `tests/test_metrics.py` green.** Those
   tests encode four measurement errors that reached committed results.
10. **Every result carries its Autoware build.** Cross-build comparison
    produced a 2× error (0.0707 vs 0.0402) and must be structurally prevented.

---

## Success criteria for the migrated system

| Criterion | Current | Target |
|---|---|---|
| Regression tests | 0 | ≥25, under 30 s |
| Realised vs intended width error | up to 1.23 m | ≤0.05 m |
| Attack types evaluable end-to-end | 5 of 7 | 7 of 7 |
| Experiment reproducibility | manual, multi-terminal | one command from spec |
| Provenance in results | filename only | full manifest per run |
| Defence layers | 2 of 3 | 3 of 3 with signing |
| Artefact-bearing rows in §4.6 | 4 | 0 |

**What must not change:** differential verification at recall 1.000 with zero
false positives on the clean map, and the measured negative results that make
the project's argument.
