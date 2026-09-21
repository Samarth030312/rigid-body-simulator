# Project Status

Last updated: 2026-09-21

## Current Phase

PHASE 10 — COMPLETE (local release candidate). Awaiting owner review; GitHub push not authorised.

## Phase gates

| Phase | Scope | Status |
|---|---|---|
| 0 | control plane, sources, MATLAB | COMPLETE — MATLAB R2026a Update 5 verified |
| 1 | assignment + lecture analysis, concept map | COMPLETE — `docs/concepts/LECTURE_CONCEPT_MAP.md`, `SOURCE_DEFECTS.md` |
| 2 | conventions, architecture freeze | COMPLETE — `CONVENTIONS.md`, `architecture.md` (ADR-0006…0013) |
| 3 | rotation primitives, transforms, kinematics | COMPLETE — 36 tests PASS |
| 4 | inertia, gravity, Newton–Euler, integration | COMPLETE — 55 tests PASS total |
| 5 | dice | COMPLETE — physics + outcome; rendering in Phase 8 |
| 6 | 3R manipulator | COMPLETE — 81 tests PASS total |
| 7 | four-bar | COMPLETE — 95 tests PASS total |
| 8 | visualization | COMPLETE — 101 tests PASS total |
| 9 | GUI | COMPLETE — 110 tests PASS total |
| 10 | end-to-end validation, docs, release prep | COMPLETE — 113 tests PASS |

## Verified facts

- Base MATLAB only (no toolboxes installed); `matlab.unittest` available.
- Lecture equations are images; evidence taken from rendered slides (ADR-0007).
- Supplied lectures: L1–L21. L22–L26 (energy methods, numerical solution, visualisation)
  are listed in L1/s7 but not supplied.

## Release readiness (definition of done, mission §23)

| Criterion | Status |
|---|---|
| Assignment requirements traceable | yes — TRACEABILITY_MATRIX all VALIDATED |
| Lecture mechanics traceable | yes — LECTURE_CONCEPT_MAP, SOURCE_DEFECTS |
| Conventions frozen | yes — CONVENTIONS.md, ADR-0006 |
| Core, dynamics, dice, 3R, four-bar, visualization, GUI validated | yes — VALIDATION_MATRIX levels 0–8 |
| End-to-end demonstrations | yes — examples/runExamples, test_end_to_end |
| Documentation current, reproducibility documented | yes — README, RUNBOOK, VALIDATION_REPORT |
| Git state coherent | yes — local commits only; not pushed (no authorisation) |
| MATLAB Online / browser deployment | NOT TESTED / NOT IMPLEMENTED |

## Known limitations / open items

- Four-bar dynamics and dice ground contact are engineering extensions (ADR-0011, 0012).
- SD-04, SD-10, SD-11 closed by Phase 6 tests.

- MATLAB Online and browser deployment not attempted.
- Only MATLAB R2026a tested.

## How to verify

```
./scripts/check_environment.sh
./scripts/run_tests.sh
```
