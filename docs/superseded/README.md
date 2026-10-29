# Superseded documents

Nothing in this directory is current. Each file carries a banner saying what
replaced it and why. They are kept because the project's argument depends on
showing which claims were revised by measurement and when.

| File | Written | Superseded by | Why |
|---|---|---|---|
| `RESULTS.md` | 2026-09-15 | `results/*.json`, README detection table | Driven-deviation figures are clean-vs-clean; `d_Fe` 0.0402 m is the noise floor, not an attack effect |
| `RESULTS_UPDATE.md` | 2026-09-12 | nothing — **do not apply** | A find-and-replace sheet that injects 0.0402 m into `RESULTS.md` and verifies the corruption succeeded |
| `HANDOFF_1.md` | 2026-09-05 | `README.md`, `ARCHITECTURE_BLUEPRINT.md` | Three-tier cascade architecture, superseded by the two-mode design |
| `HANDOFF_2.md` | 2026-09-05 | `README.md`, `ARCHITECTURE_BLUEPRINT.md` | Same, plus decisions justified by the stale F1 0.220 ceiling |
| `HANDOFF_ADDENDUM.md` | 2026-09-05 | `README.md` | Live content folded into README |
| `COMMIT_MESSAGES.txt` | 2026-08-23 | git history | Drafts for commits already made |

The single root cause for the results files: a latitude/longitude transpose,
fixed 2026-10-19. Until then the tampered map was never loaded, so every
"attack" run was a clean drive and every deviation figure measured
reproducibility rather than impact.
