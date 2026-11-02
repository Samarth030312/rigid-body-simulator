# Changelog

## 1.0.0 (2026-11-02): first public release

Three rigid-body models on one core, an interactive lab app, and a test suite that checks
the physics rather than the plumbing.

- **Mathematical core.** Rotation matrices with the group invariants enforced and tested;
  elementary, axis–angle (Rodrigues) and both body-fixed and space-fixed triple rotations
  for all twelve sequences; the axis–angle inverse taken from `atan2` of the antisymmetric
  part rather than `acos` of the trace, which costs half the significant digits near
  `φ = 0` and `φ = π`; X–Y–Z and Z–Y–Z inverses including their singular branches;
  skew matrices; space- and body-fixed angular velocity from the derivative of the rotation
  matrix; Euler-angle rate maps and the X–Y–Z inverse.
- **Rigid-body dynamics.** Inertia of a box and a cube, change of frame, parallel axis;
  Newton in the world frame and Euler about the centre of mass in the body frame; energy
  and momentum diagnostics; an 18-element state carrying the rotation matrix, with
  projection back onto SO(3) after every step.
- **Integration.** Fixed-step classical Runge–Kutta with a post-step hook, an early-stop
  condition and output decimation; `ode45` kept as an independent reference. The order of
  accuracy is measured, not claimed.
- **The die.** Cube geometry with the standard face layout and pips, initial state from
  body- or space-fixed triples and from angular velocity given in either frame, gravity,
  and an optional penalty floor (corner springs, dampers and regularised Coulomb friction)
  that is labelled an engineering extension wherever it appears. The outcome is read from
  the simulated orientation.
- **The 3R arm.** Forward kinematics and tool velocity and acceleration by frame
  propagation; recursive Newton–Euler inverse dynamics; the joint-space inertia matrix read
  out of the same routine for forward dynamics; free, constant-torque, PD with optional
  gravity compensation, and computed-torque trajectory control.
- **The four-bar linkage.** Loop closure in Freudenstein's form solved in closed form with
  an explicit assembly branch; velocities and accelerations from the differentiated
  constraint; Grashof classification; toggle detection; free-body dynamics giving the crank
  acceleration or the required torque together with every joint reaction.
- **Visualisation.** Scenes driven from the simulated state — the die's transform is the
  rotation matrix of the sample being shown — and a four-panel dashboard per model with a
  shared time cursor.
- **The app.** Parameter forms, dropdowns, Run, Reset, Random throw, playback and
  scrubbing, a results table per model, the test suite runnable from inside the app, and a
  conventions page. The app computes no mechanics; a test enforces that the mechanics
  packages never reach into it.
- **Validation.** 118 tests across ten files: closed-form answers, conservation laws,
  independent second derivations and finite differences. Reproducible reference numbers in
  `rigid_body_examples`.
- **Repository.** One `rbsim` package tree under `matlab/`, `setup.m` as the only entry
  point, CI running repository hygiene plus the full suite on Linux, Windows and macOS.
