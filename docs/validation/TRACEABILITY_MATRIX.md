# Requirements Traceability Matrix

Requirement source: `Project_ENR209_August2026.docx` (text in PROJECT_REQUIREMENTS.md).
Lecture sources: `docs/concepts/LECTURE_CONCEPT_MAP.md`. Status values: NOT STARTED,
IN PROGRESS, IMPLEMENTED (code exists, tests pending), VALIDATED (tests pass, evidence in
VALIDATION_MATRIX.md).

| ID | Requirement (assignment wording) | Lecture basis | Implementation | Validation | Status |
|---|---|---|---|---|---|
| R-D1 | Dice translation | L7-9/s4, L17-19/s6 | — | — | NOT STARTED |
| R-D2 | Dice rotation | L2-4, L7-9/s5–10 | — | — | NOT STARTED |
| R-D3 | Kinematics: orientation | L2-4/s7–13 | `core/math/rot*` | `test_rotations` | VALIDATED (primitives) |
| R-D4 | Kinematics: space-fixed rotations | L2-4/s32–36, s44 | `rotSpaceFixed` | `test_rotations` (s44 value, derivation replay) | VALIDATED |
| R-D5 | Kinematics: body-fixed rotations | L2-4/s21–31, s50, s64 | `rotBodyFixed`, `rotToEuler*` | `test_rotations` (s50, s64) | VALIDATED |
| R-D6 | Kinematics: rotation matrices | L2-4/s8–20 | `core/math/*` | `test_rotations` | VALIDATED |
| R-D7 | Angular velocity (kinematics) | L7-9/s5–16 | `core/kinematics/*`, `eulerRates*` | `test_angular_velocity` (s11–13) | VALIDATED |
| R-D8 | Dynamics: Newton equation | L17-19/s6 | — | — | NOT STARTED |
| R-D9 | Dynamics: Euler equation | L16/s6 | — | — | NOT STARTED |
| R-D10 | EOM "fed to the kinematics routines" | L7-9/s14–16 | — | — | NOT STARTED |
| R-D11 | Mass | L14-15/s4 | — | — | NOT STARTED |
| R-D12 | Inertia of a cube | L14-15/s9 | — | — | NOT STARTED |
| R-D13 | Gravity | L10-13/s8 | — | — | NOT STARTED |
| R-D14 | Initial position + orientation | L2-4, L7-9/s20 | — | — | NOT STARTED |
| R-D15 | Initial linear + angular velocity | L7-9/s4–10 | — | — | NOT STARTED |
| R-D16 | Dice dots graphics | L2-4/s13 face layout | — | — | NOT STARTED |
| R-R1 | 3R kinematics | L5-6/s13, L7-9/s44–53 | — | — | NOT STARTED |
| R-R2 | 3R dynamics | L20-21/s5–19 | — | — | NOT STARTED |
| R-F1 | Four-bar kinematics | L5-6/s20–22 | — | — | NOT STARTED |
| R-F2 | Four-bar dynamics | EXT (ADR-0011) using L17-19, L10-13/s20 | — | — | NOT STARTED |
| R-G1 | Interactive GUI | — | — | — | NOT STARTED |
| R-G2 | Buttons, dropdown menus, boxes | — | — | — | NOT STARTED |
| R-G3 | Dashboards of simulated motions and outcomes | — | — | — | NOT STARTED |
