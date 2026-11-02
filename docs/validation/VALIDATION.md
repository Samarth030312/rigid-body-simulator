# Validation

Every claim in this repository is backed by a test that runs on demand. The suite is 115
tests; `setup test`, or `scripts/run_tests.sh`, runs all of them and exits non-zero on any
failure. Last full run: **115 passed, 0 failed**, 137 s, MATLAB R2026a Update 5 on Ubuntu
22.04.

## How the checks are built

Four kinds of evidence are used, in this order of preference:

1. **A closed-form answer.** A projectile under gravity, a cube spinning with no moment, a
   symmetric body precessing freely, a rotation matrix for known angles — the test compares
   against the exact result, not against a previous run.
2. **A conservation law.** With no non-conservative force, energy and angular momentum must
   not move; with one, the change must equal the work done. The work is integrated as part
   of the state so the comparison is good to integrator accuracy.
3. **An independent second derivation.** The four-bar's free-body solution is checked
   against a one-degree-of-freedom energy equation; the arm's mass matrix against a
   link-by-link kinetic energy; the inertia formulas against a direct discretisation of the
   defining integral.
4. **Finite differences.** Any velocity or acceleration relation is checked by numerically
   differentiating the quantity it claims to be the derivative of.

Three regression values exist, and are labelled as such where they appear.

## What each area covers

| Area | Tests | What is established |
|---|---|---|
| `test_rotations` | 26 | group properties (`RᵀR = I`, `det = +1`, inverse = transpose); right-hand sense; closed-form matrices for body- and space-fixed triples; the equivalence of a body-fixed X–Y–Z triple and the reversed space-fixed triple; Rodrigues' formula and its inverse including both singular cases; eigenstructure; X–Y–Z and Z–Y–Z inverses including gimbal lock; projection back onto SO(3) |
| `test_angular_velocity` | 10 | `[Ω]p = ω × p`; `ω^s` and `ω^b` recovered from a numerically differentiated `Ṙ` agree with the closed-form rate maps; `ω^s = R ω^b`; `Ṙ = R[ω^b×] = [ω^s×]R`; point velocity and the five-term acceleration against finite differences |
| `test_inertia` | 6 | box, cube, rotated and offset inertias against a direct discretisation of `−Σ m[r][r]`, which also fixes the sign of the products of inertia; the parallel-axis and perpendicular-axis theorems; symmetry and positive definiteness |
| `test_rigid_body_dynamics` | 13 | a body at rest stays at rest; pure translation; the exact projectile; a cube's fixed-axis tumble; the symmetric top's closed-form precession; energy and angular momentum conserved when they should be; `d(H)/dt = M`; measured fourth-order convergence; agreement with `ode45`; the solver's stop and output-decimation behaviour |
| `test_dice` | 12 | the standard face layout and pip placement; the top-face rule; initial orientation from body- and space-fixed triples; angular velocity given in either frame; parameter validation including the stability limit on the step; free flight equal to the analytic result; contact that never adds energy; the resting height `a/2 − mg/4k`; eight different throws that all settle flat on at least three different faces; determinism; the same outcome at half the step |
| `test_robot` | 14 | forward kinematics against its closed form and against rigid-link distances; tool velocity and acceleration against finite differences; the reduction of the three-link recursive solution to the standard planar two-link equations; gravity torques equal to the gradient of the potential; `½q̇ᵀMq̇` equal to the link-by-link kinetic energy; `M` symmetric positive definite; inverse and forward dynamics mutually consistent; `τ = Jᵀ F` for a tool load; energy conserved when undamped; work accounted for when damped; both controllers reaching their target |
| `test_fourbar` | 14 | the loop closes to 10⁻¹⁴ m over a full crank turn on both branches; the closed-form solution against an independent numerical root; Grashof classification; assembly failure reported rather than guessed; velocities and accelerations against finite differences; the free-body dynamics against the one-degree-of-freedom energy equation; static torque equal to the gradient of the potential; ground reactions carrying the full weight; power balance under a constant-speed drive; toggle detection |
| `test_visualization` | 6 | the die's transform equals `[R r_C]` of the sample being shown; the arm's drawn joints equal its forward kinematics; the linkage's drawn joints equal the position solution; dashboards build and their cursors move; the colour order is fixed |
| `test_app` | 9 | the default form reproduces the model defaults exactly; unit conversion at the input boundary; a run through the app is bit-identical to calling the model directly; bad input is reported, not thrown; reset; playback; the scrub bar showing the sample it claims |
| `test_end_to_end` | 5 | `setup_path` works from a default MATLAB path; the worked examples reproduce their reference values; the app opens and runs; the playground script runs as shipped; the mechanics packages never reach into the app or the graphics |

## Reference results

From `matlab/scripts/rigid_body_examples.m`:

```
body-fixed  X-Y-Z (60, 45, -75) deg = [ 0.1830  0.6830  0.7071
                                       -0.3245  0.7209 -0.6124
                                       -0.9280 -0.1174  0.3536]
space-fixed X-Y-Z (60, 45, -75) deg = [ 0.1830  0.6415 -0.7450
                                       -0.6830 -0.4621 -0.5657
                                       -0.7071  0.6124  0.3536]

die, free flight 0.3 s        energy drift 5.9e-17 J, angular-momentum drift 3.8e-20
die, default throw            face 1, at rest after 0.460 s and 7 floor contacts
3R, undamped swing 2 s        max |E - E0| = 9.2e-6 J
3R, computed-torque move      tracking error 4.2e-12 rad
four-bar at 360 deg/s         loop closure 7.7e-16 m, peak torque 1.211 N m,
                              energy balance 1.3e-11 J
```

## Physical sanity checks

- A die resting flat sits at `a/2 − mg/(4k)`: four corner springs carrying the weight.
- Total energy, including the contact springs, never rises — the contact model is passive.
- A cube has isotropic inertia, so `ω × [I]ω = 0` and a free tumble is a fixed-axis rotation.
- The arm's gravity torques are the gradient of its potential energy.
- The linkage's ground reactions carry the weight of all three moving links.
- The crank torque times the crank rate equals the rate of change of the mechanism's energy.
