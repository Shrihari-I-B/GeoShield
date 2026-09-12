# RESULTS.md — §4 update blocks

Today's 0.50.0 runs supersede three subsections. Apply with Ctrl+H in VS Code,
**Match Case on, Find whole word off**. Each FIND string is unique.

Superseded by: same-build, full-exposure runs on Autoware 0.50.0 (Docker),
`clean_0.50.0` vs `g3_0.50.0_v2`, both 150 s, both completing 411 m,
**8/8 tampered lanelets driven**, `d_Fe` = **0.0402 m**.

---

## BLOCK 1 — replace the whole of §4.2

The section documented a limitation that has since been removed. It becomes a
record of the fix rather than an outstanding caveat.

### FIND

```
### 4.2 The vehicle did not drive the whole attack
```

### REPLACE

```
### 4.2 Attack exposure: incomplete at 90 s, complete at 150 s
```

---

## BLOCK 2 — replace the exposure table and its analysis

### FIND

```
The ramp spans 8 lanelets in a straight successor chain. Checking each
tampered lanelet's centreline against the driven path:
```

### REPLACE

```
The ramp spans 8 lanelets in a straight successor chain. Exposure depends on
how far the vehicle drives before recording stops, so the same map produces
different exposure at different durations. Both runs below use identical
scripted poses on the same route.

| Run | Duration | Driven | Lanelets exposed |
|---|---|---|---|
| `route_g3.0` (0.52.0, EC2) | 90 s | 338.62 m | **6 of 8** |
| `g3_0.50.0_v2` (0.50.0, Docker) | 150 s | 411.23 m | **8 of 8** |

At 90 s the vehicle stopped 54.6 m short of the 393.2 m route. Because the ramp
grows monotonically, the two largest steps lay in the uncovered remainder, and
the measured deviation was the response to a ramp experienced at +67.8% rather
than its full +117.1%. This is the same root cause as §4.1 — a fixed wall-clock
window on an unfinished route — appearing where `truncate_common()` cannot fix
it: truncation repairs the comparison, not the exposure.

Raising the window to 150 s resolved it. Distance from each tampered lanelet's
centreline to the driven path, in the full-exposure run:
```

---

## BLOCK 3 — replace the per-lanelet table

Distances are from the 150 s run. Widths are measured from the written XML
(§4.8), not from the label file.

### FIND

```
| 3002007 | 3.058 → 6.639 (+117.1%) | 2.790 m | 44.48 m | **no** |
| 3002013 | 3.057 → 6.358 (+108.0%) | 2.790 m | 74.41 m | **no** |
```

### REPLACE

```
| 3002007 | 3.058 → 5.683 (+85.8%) | 2.790 m | 1.91 m | **yes** |
| 3002013 | 3.057 → 6.057 (+98.1%) | 2.790 m | 2.83 m | **yes** |

**8 of 8 driven.** `route_overlap.py` reports the nearest map lanelet to the
driven path at 0.19 m, confirming the coordinate frames align and the distances
above are meaningful.
```

---

## BLOCK 4 — replace the §4.2 closing paragraph

### FIND

```
Topology confirms a straight chain: 3013032 → 3002007 → 3002013, no fork. The
two undriven lanelets are **downstream** of the last one driven. The route is
393.2 m; the vehicle drove 338.62 m.

**The 90 s window truncated the attack exposure, not just the comparison.** The
ramp grows monotonically, so its two largest steps lie in the 54.6 m the
vehicle never covered. The measured deviation is the response to a ramp
experienced at **+67.8%, not +117.1%**.

This is the same root cause as §4.1 — a fixed wall-clock window on an
unfinished route — appearing where truncation cannot fix it. `truncate_common()`
repaired the measurement; nothing repairs the fact that the vehicle stopped
before the attack peaked. Recording until `/api/routing/state` reports ARRIVED,
with a timeout fallback, is a prerequisite for the 0.50.0 re-run.
```

### REPLACE

```
Topology confirms a straight chain: 3013032 → 3002007 → 3002013, no fork, so
the two lanelets missed at 90 s were downstream of the last one driven rather
than on an untaken branch.

`scenario_runner.py` still records for a fixed duration. 150 s is sufficient
for this route, verified by both runs reaching 411 m against a 393.2 m route,
but it is not a general fix: a longer route or a slower run would truncate
again. Polling `/api/routing/state` for ARRIVED, with a timeout fallback,
remains the correct solution and is not implemented.
```

---

## BLOCK 5 — replace the §4.3 measurement table

The pairing changes from 3013032 (largest change driven at 90 s) to 3002013
(largest change in the map, now driven), and the deviation from the 0.0509 m
cross-build figure to the 0.0402 m same-build figure.

### FIND

```
| Quantity | Value |
|---|---|
| lanelet 3013032 width | 3.101 → 5.202 m (**+67.8%**) |
| its centreline displacement | **2.437 m** |
| resulting driven deviation | **0.0509 m** |

A 2.44 m displacement of the geometric centreline moved the driven path by
five centimetres — a factor of 48.
```

### REPLACE

```
| Quantity | Value |
|---|---|
| lanelets tampered and driven | **8 of 8** |
| widening range across the ramp | **+11.0% to +98.1%** |
| largest single change, lanelet 3002013 | 3.057 → 6.057 m (**+98.1%**) |
| largest centreline displacement | **2.790 m** |
| clean run, driven | 411.25 m |
| tampered run, driven | 411.23 m |
| **driven deviation `d_Fe`** | **0.0402 m** |
| peak lateral offset, raw points | 0.0402 m |
| safety threshold | 0.5 m |

A 2.79 m displacement of the geometric centreline — approaching a full lane
width — moved the driven path by **3.3 centimetres**, a factor of 83. The
deviation is one fifteenth of the safety threshold.

Both runs are Autoware 0.50.0 (Docker), 150 s, identical scripted poses,
both completing the route. No cross-build comparison, no truncation artefact,
no partial exposure. Two implementations agree: 0.0402 m on raw points
(`route_overlap.py`), 0.0334 m resampled (`frechet_analysis.py`).
```

---

## BLOCK 6 — strengthen the §4.3 conclusion

### FIND

```
The finding is real and is not affected by either artefact, because it compares
the *map* against the *behaviour* rather than two trajectories against each
other. It is not a headline: it is a negative result about attack efficacy, on
one route, on one planner build.
```

### REPLACE

```
The finding survived every correction applied to §4. It is not affected by the
endpoint artefact (§4.1), because the runs are length-matched to within 2 cm;
not by incomplete exposure (§4.2), because all eight lanelets were driven; and
not by build mismatch, because both runs are 0.50.0. It compares the *map*
against the *behaviour* rather than two trajectories against each other.

It remains a negative result about attack efficacy on one route and one planner
build, not a headline. But it is a clean one, and it carries a design
implication: **map deviation and behavioural deviation are distinct
quantities.** A defence that measured only how much the map changed would rate
this attack as critical; the vehicle moved 3 cm. Any severity model for HD map
tampering has to account for what the consuming planner does with the change,
not only the magnitude of the change itself.
```

---

## BLOCK 7 — replace the §4.6 table

### FIND

```
| Condition | `d_Fp` [m] | `d_Fe` [m] | Status |
|---|---|---|---|
| tamper off-route | 0.020 | 0.083 | artefact-bearing |
| ramp, 2.0 m total | 0.312 | 0.204 | artefact-bearing |
| ramp, 3.0 m total | 1.998 → withdrawn | 1.414 → **0.0509** | **re-derived** |
| ramp, 4.5 m total | — | — | ego failed to localise |
| ramp, 6.0 m total | — | — | ego failed to localise |
```

### REPLACE

```
| Condition | Build | `d_Fp` [m] | `d_Fe` [m] | Status |
|---|---|---|---|---|
| tamper off-route | 0.52.0 | 0.020 | 0.083 | artefact-bearing |
| ramp, 2.0 m total | 0.52.0 | 0.312 | 0.204 | artefact-bearing |
| ramp, 3.0 m total, 90 s | 0.52.0 | 1.998 → withdrawn | 1.414 → 0.0483 | superseded, 6/8 exposure |
| ramp, 3.0 m total, 90 s, cross-build | mixed | — | 0.0707 | superseded, confounded |
| **ramp, 3.0 m total, 150 s** | **0.50.0** | not recorded | **0.0402** | **reference result** |
| ramp, 4.5 m total | 0.52.0 | — | — | ego failed to localise |
| ramp, 6.0 m total | 0.52.0 | — | — | ego failed to localise |

Autoware **0.50.0 (Docker) is the reference build**: local, pinned to a public
image digest, reproducible without cloud access. The 0.52.0 rows were produced
on EC2 and are retained as a record of what was run.

`d_Fp` is not recorded for the 0.50.0 runs: that build publishes on
`/planning/scenario_planning/trajectory` while `frechet_analysis.py` reads
`/planning/trajectory`. The topic name differs between the two Autoware
versions. `d_Fp` is withdrawn regardless (§4.4), so this was not pursued.
```

---

## BLOCK 8 — new subsection, append after §4.8

A rejected approach, with the measurement that rejected it.

### FIND

```
### 4.8 The injector does not write the widths it records
```

### REPLACE

```
### 4.9 A graph reachability check was implemented and rejected

`oneway_flip` (0.021) and `connectivity_break` (0.120) are the weakest
single-snapshot detections in §3. Both are topological attacks and every
feature in `features.py` is geometric, so the detectors were structurally
blind to them rather than merely poor at them. A reachability check —
orphan lanelets, direction contradictions, network fragmentation, reachable-set
comparison between versions — needs no training and appeared to be the missing
piece.

It does not work, for two measured reasons.

**Tag flips leave the derived graph unchanged.** Lanelet2 encodes connectivity
through shared boundary endpoints, not through explicit edge declarations.
`oneway_flip` rewrites the `one_way` tag; the shared nodes are untouched, so
the derived graph is byte-identical. Run against `dv_oneway_flip.osm` the
check reported the same 973 edges, the same 849/884 largest component and zero
direction contradictions as the clean map — output indistinguishable from the
untampered case.

**The clean map is not clean by this measure.** Nishi-Shinjuku is a clipped
region, so 67 lanelets have no predecessor and 65 have no successor at the
boundary — legitimate. But 10 lanelets are fully isolated, and flagging those
produced a REJECT verdict on the untampered map. A detector that rejects an
honest map is the same failure mode as the per-lanelet repair in §5.

The approach is dropped. `differential_verify.py` already catches
`oneway_flip` at precision 1.000 and recall 1.000 by comparing the tag itself
against the previous version — the attack is visible as a *change* even though
it is invisible as a *structure*. This is the §7 argument in a second domain:
topological tampering, like gradual geometric tampering, resists absolute
verification and yields to differential verification.

### 4.8 The injector does not write the widths it records
```

*(This places §4.9 before §4.8 in the file; reorder if the numbering matters,
or renumber §4.8 → §4.9 and this → §4.8.)*

---

# Verification after applying

```bash
cd ~/Development/projects/geoshield
grep -c "0.0402" RESULTS.md          # expect 4
grep -c "8 of 8" RESULTS.md          # expect 2
grep -c "4.9 A graph reachability"   RESULTS.md   # expect 1
grep -c "0.0509" RESULTS.md          # expect 0
wc -l RESULTS.md
```

`0.0509` should be gone entirely — it was the cross-build figure and is
superseded everywhere.

---

# Commit

```
RESULTS §4 re-derived on the reference build: d_Fe 0.0402 m, 8/8 exposure.

Supersedes the 0.0509 m figure, which came from a 90 s run at 6/8 exposure,
and the 0.0707 m figure, which compared a 0.50.0 tampered run against a
0.52.0 clean baseline.

Reference result: Autoware 0.50.0 (Docker), clean_0.50.0 vs g3_0.50.0_v2,
both 150 s, both completing 411 m of a 393.2 m route, all 8 tampered lanelets
driven. Widening +11.0% to +98.1%, largest centreline displacement 2.790 m,
driven deviation 0.0402 m against a 0.5 m threshold. Two implementations
agree (0.0402 raw, 0.0334 resampled).

Adds §4.9: graph reachability check implemented and rejected. Tag-level
direction flips leave the shared-node topology unchanged, so the derived graph
is identical to the clean map's; and flagging isolated lanelets rejects the
untampered map. Differential comparison of the tag catches oneway_flip at
1.000/1.000 instead.
```
