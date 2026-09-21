# Requirements Traceability Matrix

Requirement source: `Project_ENR209_August2026.docx` (text in PROJECT_REQUIREMENTS.md).
Lecture sources: `docs/concepts/LECTURE_CONCEPT_MAP.md`. Status values: NOT STARTED,
IN PROGRESS, IMPLEMENTED (code exists, tests pending), VALIDATED (tests pass, evidence in
VALIDATION_MATRIX.md).

| ID | Requirement (assignment wording) | Lecture basis | Implementation | Validation | Status |
|---|---|---|---|---|---|
| R-D1 | Dice translation | L7-9/s4, L17-19/s6 | `dice/simulateDice` | `testFreeFlightAnalytic` | VALIDATED |
| R-D2 | Dice rotation | L2-4, L7-9/s5–10, L16/s6 | `dice/simulateDice` | `testFreeFlightAnalytic`, `testOutcomeEmergesFromDynamics` | VALIDATED |
| R-D3 | Kinematics: orientation | L2-4/s7–13 | `core/math/rot*` | `test_rotations` | VALIDATED (primitives) |
| R-D4 | Kinematics: space-fixed rotations | L2-4/s32–36, s44 | `rotSpaceFixed` | `test_rotations` (s44 value, derivation replay) | VALIDATED |
| R-D5 | Kinematics: body-fixed rotations | L2-4/s21–31, s50, s64 | `rotBodyFixed`, `rotToEuler*` | `test_rotations` (s50, s64) | VALIDATED |
| R-D6 | Kinematics: rotation matrices | L2-4/s8–20 | `core/math/*` | `test_rotations` | VALIDATED |
| R-D7 | Angular velocity (kinematics) | L7-9/s5–16 | `core/kinematics/*`, `eulerRates*` | `test_angular_velocity` (s11–13) | VALIDATED |
| R-D8 | Dynamics: Newton equation | L17-19/s6 | `rigidBodyDerivative` | projectile analytic test | VALIDATED (core) |
| R-D9 | Dynamics: Euler equation | L16/s6 | `rigidBodyDerivative` | symmetric top, conservation, dH/dt = M | VALIDATED (core) |
| R-D10 | EOM "fed to the kinematics routines" | L7-9/s14–16 | `integrateRK4` + `Ṙ = R[ω^b×]` | Level 4 tests | VALIDATED (core) |
| R-D11 | Mass | L14-15/s4 | `diceDefaultParams.mass` | static equilibrium test | VALIDATED |
| R-D12 | Inertia of a cube | L14-15/s9 | `inertiaCube` | `test_inertia` | VALIDATED |
| R-D13 | Gravity | L10-13/s8 | force `m g` | projectile, energy tests | VALIDATED (core) |
| R-D14 | Initial position + orientation | L2-4, L7-9/s20 | `diceInitialState` (body/space-fixed, any sequence) | `testInitialOrientationBodyAndSpace` | VALIDATED |
| R-D15 | Initial linear + angular velocity | L7-9/s4–10 | `diceInitialState` (ω in body or space components) | `testInitialAngularVelocityFrames` | VALIDATED |
| R-D16 | Dice dots graphics | L2-4/s13 face layout | `diceGeometry`, `vizDiceScene` (pips on faces, state-driven) | `test_dice` layout tests, `test_visualization` | VALIDATED |
| R-R1 | 3R kinematics | L5-6/s13, L7-9/s44–53 | `robotForwardKinematics`, `robotToolKinematics` | `test_robot` kinematics tests | VALIDATED |
| R-R2 | 3R dynamics | L20-21/s5–19 | `robotNewtonEuler`, `robotMassMatrix`, `robotDynamics`, `simulateRobot` | 2R reduction, virtual work, energy tests | VALIDATED |
| R-F1 | Four-bar kinematics | L5-6/s20–22 | `fourbarPosition`, `fourbarKinematics`, `fourbarGrashof` | `test_fourbar` position/velocity tests | VALIDATED |
| R-F2 | Four-bar dynamics | EXT (ADR-0011) using L17-19, L10-13/s20 | `fourbarNewtonEuler`, `fourbarEnergy`, `simulateFourbar` | energy-equation cross-check, virtual work, energy balance | VALIDATED |
| R-G1 | Interactive GUI | — | `app/RigidBodySimulatorApp.m`, `launchRigidBodySimulator` | `test_app` | VALIDATED |
| R-G2 | Buttons, dropdown menus, boxes | — | Run/Reset/Random throw/Play buttons, rotation type/sequence/ω-frame/control/drive/branch dropdowns, numeric boxes, check boxes, time slider | `test_app` | VALIDATED |
| R-G3 | Dashboards of simulated motions and outcomes | — | `vizDashboard` in each tab + summary table (dice outcome, residuals, energy balance) + Validation tab | `test_visualization`, `test_app` | VALIDATED |
