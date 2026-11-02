# Capability map

What the simulator does, where it is implemented and what proves it. Every row is covered
by the suite described in [`VALIDATION.md`](VALIDATION.md).

## The die

| Capability | Implementation | Evidence |
|---|---|---|
| Translation and rotation together | `rbsim.dice.simulate` (18-state body) | `test_dice/testFreeFlightAnalytic` |
| Orientation as a rotation matrix | `rbsim.math`, `rbsim.body.pack` | `test_rotations` |
| Rotations about body-fixed axes | `rbsim.math.rotBodyFixed` | closed-form matrices, `test_rotations` |
| Rotations about space-fixed axes | `rbsim.math.rotSpaceFixed` | closed-form matrices and a second construction |
| Angular velocity, both frames | `rbsim.body.angularVelocityFromRdot` | `test_angular_velocity` |
| Newton's equation | `rbsim.body.derivative` | exact projectile |
| Euler's equation | `rbsim.body.derivative` | symmetric top, `d(H)/dt = M` |
| Equations of motion driving the kinematics | `rbsim.solver.rk4` with `Ṙ = R[ω^b×]` | convergence and `ode45` agreement |
| Mass and cube inertia | `rbsim.body.inertiaCube` | discretised definition |
| Gravity | `rbsim.dice.dynamics` | projectile and energy tests |
| Initial position and orientation | `rbsim.dice.initialState` | `test_dice` |
| Initial linear and angular velocity | `rbsim.dice.initialState` | both ω frames checked |
| Faces and pips | `rbsim.dice.geometry`, `rbsim.viz.diceScene` | layout, winding and pip-count tests |
| Outcome read from the simulated state | `rbsim.dice.topFace` | eight throws, ≥3 distinct faces |
| Landing on a floor (extension) | `rbsim.dice.contactLoads` | passivity and resting height |

## The 3R arm

| Capability | Implementation | Evidence |
|---|---|---|
| Forward kinematics | `rbsim.robot.forwardKinematics` | closed form and rigid-link distances |
| Velocity and acceleration propagation | `rbsim.robot.toolKinematics` | finite differences |
| Inverse dynamics | `rbsim.robot.newtonEuler` | exact reduction to the planar 2R equations |
| Mass matrix and forward dynamics | `rbsim.robot.massMatrix`, `rbsim.robot.dynamics` | kinetic energy, round trip |
| Gravity torques | same | gradient of the potential |
| Joint damping and actuator work | `rbsim.robot.simulate` | energy balance |
| Control: free, constant, PD, computed torque | `rbsim.robot.jointTorque` | target reached, tracking error |

## The four-bar linkage

| Capability | Implementation | Evidence |
|---|---|---|
| Geometry and assembly branch | `rbsim.fourbar.position` | closure residual on both branches |
| Loop closure | same | 10⁻¹⁴ m over a full turn; independent root-find |
| Grashof classification | `rbsim.fourbar.grashof` | the four classes |
| Velocity and acceleration | `rbsim.fourbar.kinematics` | finite differences |
| Dynamics and joint reactions | `rbsim.fourbar.newtonEuler` | independent energy equation |
| Drive by torque or by speed | `rbsim.fourbar.simulate` | power balance |
| Toggle detection | `rbsim.fourbar.kinematics` | explicit error at a change point |

## The application

| Capability | Implementation | Evidence |
|---|---|---|
| Interactive interface | `RigidBodyLabApp` | `test_app` |
| Buttons, dropdowns, input boxes | the parameter forms | `test_app` |
| Dashboards of motion and outcome | `rbsim.viz.dashboard` and the result tables | `test_visualization`, `test_app` |
| Playback and scrubbing | `RigidBodyLabApp` | `test_app` |
| Checks runnable from the app | the Validation tab | the suite itself |
