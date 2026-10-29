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
