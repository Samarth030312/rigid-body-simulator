# ENR209 Rigid Body Simulator

A MATLAB simulation framework for **ENR209 Mechanics of Rigid Bodies** (Monsoon 2026):
a thrown **dice** (rigid-body kinematics + Newton–Euler dynamics), a **planar 3R robotic
manipulator** and a **four-bar mechanism** (kinematics + dynamics), with an interactive
GUI and diagnostic dashboards.

Every equation is traced to the course lectures or labelled as an engineering extension,
and every subsystem has automated validation (110+ tests against analytic solutions,
lecture numerical examples and conservation laws).

![Dice tab](docs/figures/gui_dice.png)

## Requirements

- MATLAB **R2026a** (developed and tested on R2026a Update 5, Ubuntu 22.04).
- **Base MATLAB only** — no toolboxes. (`uifigure`, `matlab.unittest`, `ode45` are base.)
- Older releases are untested; the GUI uses `theme()` guarded by `try` so it should still
  open on releases that lack it.

## Quick start

```matlab
cd RigidBodySimulator
addpath app
launchRigidBodySimulator          % opens the GUI
```

Use the GUI tabs **Dice**, **3R Manipulator**, **Four-Bar Mechanism**: edit parameters,
press **Run**, then **Play** or drag the time slider. **Validation** runs the full test
suite inside the app; **Conventions** summarises the frames and equations.

Without the GUI (the mechanics never depend on it):

```matlab
addpath config; addProjectPaths();
res = simulateDice(diceDefaultParams());      res.summary
res = simulateRobot(robotDefaultParams());    res.summary
res = simulateFourbar(fourbarDefaultParams()); res.summary
addpath examples; runExamples                  % reproducible reference numbers
```

## Verify

```bash
./scripts/check_environment.sh      # MATLAB release
./scripts/run_tests.sh              # full suite (about 2 minutes)
./scripts/run_tests.sh dice         # one area: core | dice | robot | fourbar | visualization | app | e2e
matlab -batch "addpath('scripts'); make_figures"   # regenerate docs/figures
```

## What is simulated

| Application | Kinematics | Dynamics | Lecture basis |
|---|---|---|---|
| Dice | orientation as `^A_B[R]`; body-/space-fixed rotations in any sequence; `ω^s`, `ω^b` | Newton `m a_C = F`, Euler `[I]ω̇ + ω×[I]ω = M` in the body frame, cube inertia `m a²/6`, gravity | L2–L4, L7–L9, L14–L16 |
| Planar 3R | frame composition, velocity/acceleration propagation | recursive Newton–Euler (inverse), mass-matrix forward dynamics; free / constant torque / PD / computed-torque control | L5–L9, L20–L21 |
| Four-bar | Freudenstein loop closure with explicit assembly branch, velocity/acceleration, Grashof | per-link Newton–Euler free-body diagrams with joint reactions; free / torque / constant-speed drive | L5–L6, L10–L13, L17–L19 |

Engineering extensions (not in the supplied lectures, clearly labelled): dice floor contact
(vertex spring–damper–Coulomb friction), dice outcome rule, four-bar dynamics formulation,
controllers, Grashof classification, RK4 integrator choice. See `docs/DECISIONS.md`.

## Repository layout

```
core/math          rotation matrices, Euler angles, axis-angle, skew matrices
core/kinematics    point position/velocity/acceleration, angular velocity
core/dynamics      inertia, Newton-Euler rigid-body derivative, energy/momentum
core/integration   RK4 (with SO(3) projection), ode45 reference
dice/ robot/ fourbar/   application models and simulate* drivers
visualization/     state-driven scenes and dashboards
app/               GUI (RigidBodySimulatorApp) and launcher
examples/          GUI-free demonstrations
tests/             matlab.unittest suites (levels 1-8)
docs/              requirements, concept map, conventions, decisions, validation
```

## Documentation

| Topic | File |
|---|---|
| **Which file does what / where to tinker** | `docs/FILE_GUIDE.md`, `examples/playground.m` |
| Requirement-by-requirement check | `docs/validation/REQUIREMENTS_CHECK.md` |
| Frame/rotation/unit conventions (frozen) | `docs/architecture/CONVENTIONS.md` |
| Architecture | `docs/architecture/architecture.md` |
| Lecture concept map + slide defects | `docs/concepts/LECTURE_CONCEPT_MAP.md`, `docs/concepts/SOURCE_DEFECTS.md` |
| Decisions (ADRs) | `docs/DECISIONS.md` |
| Validation evidence | `docs/validation/VALIDATION_REPORT.md`, `VALIDATION_MATRIX.md`, `TRACEABILITY_MATRIX.md` |
| Status / history | `docs/PROJECT_STATUS.md`, `docs/IMPLEMENTATION_LOG.md` |

Course lecture slides and the assignment are **not** redistributed in this repository.

## Deployment status

| Target | Status |
|---|---|
| Local MATLAB R2026a (Linux) | tested |
| MATLAB Online | not tested |
| Browser deployment | not implemented |
