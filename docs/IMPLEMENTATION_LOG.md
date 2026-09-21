# Implementation Log

## 2026-09-21 — Project Control Plane

Created:

- CLAUDE.md
- .claude/rules/
- .claude/commands/
- project requirements document
- lecture index
- project status

No MATLAB implementation has been committed.

Next:

1. inspect lecture material;
2. inspect MATLAB environment;
3. establish toolchain baseline;
4. derive architecture and validation matrix.

## 2026-09-21 — Audit, source analysis, convention freeze, Phase 3 core

Audit findings:
- MATLAB R2026a Update 5 verified; base MATLAB only. PROJECT_STATUS previously (wrongly)
  said MATLAB was missing — superseded.
- `scripts/extract_source.py` defects fixed (empty assignment text; wrong slide numbers).
- Lecture equations are images → slides rendered and inspected (ADR-0007). Three bounded
  read-only subagents transcribed L5–L9 (mechanisms), L10–L15 (forces, inertia) and
  L16–L21 (dynamics); the principal agent verified every convention-critical slide
  (L2-4/s7–69, L7-9/s2–24, s49, L5-6/s20–22, L16/s6, L20-21/s10–11, s19) directly.

Created: `docs/concepts/LECTURE_CONCEPT_MAP.md`, `docs/concepts/SOURCE_DEFECTS.md`,
frozen `CONVENTIONS.md` and `architecture.md`, ADR-0006…0013, traceability and validation
matrices.

Code: `core/math/` (17 functions), `core/kinematics/` (5 functions), `tests/runAllTests.m`,
`scripts/run_tests.sh`, `tests/core/test_rotations.m` (27), `test_angular_velocity.m` (9).

Tests: `./scripts/run_tests.sh core` → 36/36 PASS.

Next: Phase 4 dynamics + integration.

## 2026-09-21 — Phase 4: dynamics and integration

Code: `core/dynamics/` (inertiaBox, inertiaCube, inertiaRotate, inertiaParallelAxis,
rbPack/rbUnpack, rigidBodyDerivative, rigidBodyEnergy, rbProjectState),
`core/integration/` (integrateRK4 with PostStep/OutputEvery/StopFcn, integrateODE45).
Tests: `test_inertia` (6), `test_rigid_body_dynamics` (11). Full suite 55/55 PASS.

Findings during testing:
- First cube torque-free tolerance (1e-9) was tighter than the RK4 truncation bound
  (~5e-9 for the chosen dt); tolerance now derived from the bound.
- L7-9/s16 plot is not reproducible from the slide's equations (SD-14); implementation
  proven against the exact constant-ω rotation instead.

## 2026-09-21 — Phase 5: dice

Code: `dice/` — diceDefaultParams, diceValidateParams (incl. RK4 stability limit for the
contact stiffness), diceInitialState (body-/space-fixed orientation, ω in body or space
components), diceGeometry (COM-centred cube, lecture face layout, pip positions),
diceTopFace, diceContactLoads (EXT: vertex spring-damper + regularised Coulomb friction),
diceDynamics, DiceRestDetector (early stop once at rest 0.1 s), simulateDice.

Tests: `tests/dice/test_dice.m` (12). Full suite 67/67 PASS.

Physical checks: free flight equals the analytic projectile + fixed-axis rotation
(isotropic cube inertia ⇒ ω×Iω = 0); total energy including contact springs never
increases (max per-sample increase ~1e-18 J = round-off); resting COM height equals
a/2 − mg/(4k).

Performance: default throw ~1 s (was 7.3 s): inertia precomputed, argument validation
removed from inner-loop functions only, diagnostics computed lazily, early stop at rest.

Test-authoring errors caught by the suite (fixed in the tests, not the code): swapped
expected faces for R(X,±90°); an RK4 tolerance tighter than the truncation bound.

## 2026-09-21 — Phase 6: planar 3R manipulator

Code: `robot/` — robotDefaultParams, robotValidateParams, robotForwardKinematics
(lecture composition), robotToolKinematics (lecture propagation), robotNewtonEuler (RNE
exactly as L20-21/s10–11, optional tool force), robotMassMatrix (RNE columns),
robotJointTorque (free / constant / PD ± gravity compensation / computed-torque
trajectory), robotReference (quintic, EXT), robotDynamics, robotEnergy (independent,
link-by-link), simulateRobot (actuator + damper work integrated as an internal 7th state).

Tests: `tests/robot/test_robot.m` (14). Full suite 81/81 PASS.

Findings: energy balance with trapezoid-integrated power was limited by output sampling
(error ∝ h²); fixed by integrating work with the state — residual now falls ~20× per dt
halving (RK4 order), proving dynamics/work consistency.
