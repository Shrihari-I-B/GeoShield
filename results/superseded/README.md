Results kept for provenance and excluded from demo.sh.

fr_clean_vs_cl_multi_shield_2026-10-26.json   d_Fe 0.0523
    Superseded by fr_clean_vs_cl_multi_shield2. This run used a repaired map
    that still contained 8 orphaned ways and 89 orphaned nodes, which raised
    mrm_comfortable_stop_operator repeatedly where the clean control raised
    nothing. Unreferenced geometry is not inert. See
    claude/2026-10-26-centreline-attack-family.md.

fr_clean_vs_cl_single_2026-10-26.json         d_Fe 0.4717
fr_clean_vs_cl_single2_2026-10-26.json        d_Fe 0.4405
    Intermediate single-lanelet centreline experiments, used to isolate the
    junction-continuity defect. cl_single2 is an INCOMPLETE run: the vehicle
    stopped 72.7 m short of the goal, so its d_Fe covers 338.5 m of a 411 m
    route and is not comparable with the full-route conditions.

--- Pre-fix evaluation artefacts, moved 2026-10-30 ---

ablation.json / ablation.txt / separability.txt        dated 2026-10-07
detectability{,_v2,_v3,_by_type}.json                  dated 2026-08-12/13
tier2_ramp.json                                        dated 2026-08-12

All predate the lat/lon fix of 2026-10-19, which showed that
write_tampered_map displaced nodes in local_x/local_y only while Autoware
projects from lat/lon -- so every "tampered" result before that date was
measured on a map that reached the simulator unmodified. They also predate
the Tier 2 feature cleanup (three identically-zero columns and the `length`
leak removed), the speed-unit correction (201 of 979 lanelets mis-parsed),
and the regeneration of the attack suite with real truth sets.

Superseded by: results/tier1_matrix*.json, results/fusion_*.json,
results/tier2_width_clean.json, results/dv_summary.json.

NOT superseded in substance: detectability.py performs a magnitude-stratified
analysis -- how large must an attack be before it is detected -- which nothing
current replaces, and which directly addresses the open question of the
smallest detectable width shift. Worth regenerating as its own piece of work.
