# File Guide — where to play and tinker

Start MATLAB in the repository folder.

| I want to… | Open / run |
|---|---|
| Use the GUI | `addpath app; launchRigidBodySimulator` |
| Change numbers in code and see the result animated | `examples/playground.m` (edit a section, **Ctrl+Enter**) |
| See reference numbers without graphics | `addpath examples; runExamples` |
| Run all checks | `./scripts/run_tests.sh` (or in MATLAB: `addpath tests; runAllTests`) |
| Regenerate the figures in `docs/figures/` | `matlab -batch "addpath('scripts'); make_figures"` |

## Safe knobs (change freely)

| File | What to change |
|---|---|
| `dice/diceDefaultParams.m` | edge, mass, initial position/orientation/velocity/spin, contact stiffness, damping, friction, duration, time step |
| `robot/robotDefaultParams.m` | link lengths, masses, COM positions, inertias, initial angles, control mode, gains, target, damping |
| `fourbar/fourbarDefaultParams.m` | link lengths (try non-Grashof ones!), masses, branch, initial crank angle/rate, drive mode, torque, speed, load, damping |
| `visualization/vizTheme.m` | colours |
| `examples/playground.m` | everything above, without editing the defaults |

After changing defaults, run `./scripts/run_tests.sh` — some tests assume the shipped
defaults and will tell you which ones changed.

## Every file

### Mechanics core (`core/`) — lecture mathematics
| File | Purpose | Lecture |
|---|---|---|
| `core/math/rotX.m`, `rotY.m`, `rotZ.m` | simple rotations | L2-4/s19–20 |
| `core/math/rotAxisAngle.m`, `rotToAxisAngle.m` | Rodrigues / axis–angle and inverse | L2-4/s14–18 |
| `core/math/rotBodyFixed.m` | body-fixed rotations (post-multiply) | L2-4/s21–31 |
| `core/math/rotSpaceFixed.m` | space-fixed rotations (pre-multiply) | L2-4/s32–36 |
| `core/math/rotToEulerXYZ.m`, `rotToEulerZYZ.m` | rotation matrix → Euler angles | L2-4/s26–27, s31 |
| `core/math/eulerRatesXYZ.m`, `eulerRatesZYZ.m`, `eulerRatesFromOmegaXYZ.m` | Euler rates ↔ angular velocity | L7-9/s7–16 |
| `core/math/skew.m`, `unskew.m` | skew-symmetric matrix `[Ω]` | L7-9/s6 |
| `core/math/isRotationMatrix.m`, `rotationError.m`, `orthonormalize.m` | det = 1, RᵀR = I checks / repair | L2-4/s9 |
| `core/kinematics/pointPosition.m`, `pointVelocity.m`, `pointAcceleration.m` | point motion on a rigid body | L7-9/s20–24 |
| `core/kinematics/angularVelocityFromRdot.m`, `rotationRate.m` | ω from Ṙ, Ṙ from ω | L7-9/s5–10 |
| `core/dynamics/inertiaBox.m`, `inertiaCube.m`, `inertiaRotate.m`, `inertiaParallelAxis.m` | mass properties | L14-15/s6–9 |
| `core/dynamics/rigidBodyDerivative.m` | **Newton + Euler equations** | L16/s6, L17-19/s6 |
| `core/dynamics/rigidBodyEnergy.m` | energy and momenta | L17-19/s27 |
| `core/dynamics/rbPack.m`, `rbUnpack.m`, `rbProjectState.m` | 18×1 state layout, SO(3) projection | CONVENTIONS §6.2 |
| `core/integration/integrateRK4.m`, `integrateODE45.m` | time integration | ADR-0009 |

### Applications
| File | Purpose |
|---|---|
| `dice/simulateDice.m` | run a throw → result (trajectory, energies, top face) |
| `dice/diceInitialState.m` | position + body/space-fixed orientation + velocities → state |
| `dice/diceDynamics.m` | Newton–Euler with gravity and optional floor |
| `dice/diceContactLoads.m` | floor contact forces (extension) |
| `dice/diceGeometry.m`, `diceTopFace.m` | cube, dots, face numbering, outcome |
| `dice/DiceRestDetector.m`, `diceValidateParams.m`, `diceDefaultParams.m` | stop at rest, input checks, defaults |
| `robot/simulateRobot.m` | run the 3R |
| `robot/robotForwardKinematics.m`, `robotToolKinematics.m` | position / velocity / acceleration |
| `robot/robotNewtonEuler.m`, `robotMassMatrix.m`, `robotDynamics.m` | recursive Newton–Euler dynamics |
| `robot/robotJointTorque.m`, `robotReference.m` | control modes, trajectories |
| `robot/robotEnergy.m`, `robotValidateParams.m`, `robotDefaultParams.m` | energy check, input checks, defaults |
| `fourbar/simulateFourbar.m` | run the four-bar |
| `fourbar/fourbarPosition.m`, `fourbarKinematics.m`, `fourbarGrashof.m` | loop closure, velocity/acceleration, mechanism type |
| `fourbar/fourbarNewtonEuler.m`, `fourbarEnergy.m` | dynamics and joint reactions; independent energy check |
| `fourbar/fourbarValidateParams.m`, `fourbarDefaultParams.m` | input checks, defaults |

### Visualization and GUI
| File | Purpose |
|---|---|
| `visualization/vizDiceScene.m`, `vizDiceUpdate.m` | 3D dice with dots, frames, trajectory, arrows |
| `visualization/vizRobotScene.m`, `vizRobotUpdate.m` | 3R drawing |
| `visualization/vizFourbarScene.m`, `vizFourbarUpdate.m` | four-bar drawing |
| `visualization/vizDashboard.m`, `vizSetCursor.m` | 4-panel dashboards |
| `visualization/vizTheme.m`, `vizStyleAxes.m`, `vizArrow3.m`, `vizSetArrow3.m` | styling helpers |
| `app/RigidBodySimulatorApp.m` | the GUI |
| `app/launchRigidBodySimulator.m` | entry point |

### Examples, tests, scripts
| File | Purpose |
|---|---|
| `examples/playground.m` | editable sections to tinker with |
| `examples/runExamples.m` | reference results, no graphics |
| `tests/runAllTests.m` | test runner |
| `tests/core/*`, `tests/dice/*`, `tests/robot/*`, `tests/fourbar/*`, `tests/visualization/*`, `tests/app/*`, `tests/e2e/*` | 114 tests, levels 1–8 |
| `scripts/run_tests.sh`, `check_environment.sh`, `make_figures.m` | run tests, check MATLAB, render figures |
| `config/addProjectPaths.m`, `projectRoot.m` | path setup |

### Documentation
`README.md`, `docs/architecture/CONVENTIONS.md`, `docs/concepts/LECTURE_CONCEPT_MAP.md`,
`docs/concepts/SOURCE_DEFECTS.md`, `docs/DECISIONS.md`,
`docs/validation/REQUIREMENTS_CHECK.md`, `docs/validation/VALIDATION_REPORT.md`.
