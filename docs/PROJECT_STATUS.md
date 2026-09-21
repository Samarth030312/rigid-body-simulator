# Project Status

Last updated: 2026-09-21

## Current Phase

PHASE 10 — End-to-end validation, documentation, release preparation (next).

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
| 9 | GUI | COMPLETE — 116 tests PASS total |
| 10 | end-to-end validation, docs, release prep | NEXT |

## Verified facts

- Base MATLAB only (no toolboxes installed); `matlab.unittest` available.
- Lecture equations are images; evidence taken from rendered slides (ADR-0007).
- Supplied lectures: L1–L21. L22–L26 (energy methods, numerical solution, visualisation)
  are listed in L1/s7 but not supplied.

## Known limitations / open items

- Four-bar dynamics and dice ground contact are engineering extensions (ADR-0011, 0012).
- SD-04, SD-10, SD-11 closed by Phase 6 tests.

## How to verify

```
./scripts/check_environment.sh
./scripts/run_tests.sh
```
