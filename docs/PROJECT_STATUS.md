# Project Status

Last updated: 2026-09-21

## Current Phase

PHASE 4 — Newton–Euler dynamics and numerical integration (next).

## Phase gates

| Phase | Scope | Status |
|---|---|---|
| 0 | control plane, sources, MATLAB | COMPLETE — MATLAB R2026a Update 5 verified |
| 1 | assignment + lecture analysis, concept map | COMPLETE — `docs/concepts/LECTURE_CONCEPT_MAP.md`, `SOURCE_DEFECTS.md` |
| 2 | conventions, architecture freeze | COMPLETE — `CONVENTIONS.md`, `architecture.md` (ADR-0006…0013) |
| 3 | rotation primitives, transforms, kinematics | COMPLETE — 36 tests PASS |
| 4 | inertia, gravity, Newton–Euler, integration | NEXT |
| 5 | dice | NOT STARTED |
| 6 | 3R manipulator | NOT STARTED |
| 7 | four-bar | NOT STARTED |
| 8 | visualization | NOT STARTED |
| 9 | GUI | NOT STARTED |
| 10 | end-to-end validation, docs, release prep | NOT STARTED |

## Verified facts

- Base MATLAB only (no toolboxes installed); `matlab.unittest` available.
- Lecture equations are images; evidence taken from rendered slides (ADR-0007).
- Supplied lectures: L1–L21. L22–L26 (energy methods, numerical solution, visualisation)
  are listed in L1/s7 but not supplied.

## Known limitations / open items

- Four-bar dynamics and dice ground contact are engineering extensions (ADR-0011, 0012).
- Source defects SD-04, SD-10, SD-11 to be closed by Phase 6 tests.

## How to verify

```
./scripts/check_environment.sh
./scripts/run_tests.sh
```
