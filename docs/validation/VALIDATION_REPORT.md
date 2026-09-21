# Validation Report

Date: 2026-09-21 · MATLAB R2026a Update 5 · Ubuntu 22.04.5 · base MATLAB only.
Command: `./scripts/run_tests.sh` — result recorded in `VALIDATION_MATRIX.md`.

## Summary by level

| Level | Scope | Tests | Key evidence | Result |
|---|---|---|---|---|
| 0 | environment | scripts | MATLAB launches, sources verified | PASS |
| 1 | rotation primitives | 27 | all lecture numerical rotation matrices (L2-4/s12, s44, s50, s64) to 4 decimals; closed forms; derivation replay of space-fixed rotations | PASS |
| 2 | kinematics | 9 (+3R, four-bar kinematics) | lecture angular-velocity values (L7-9/s12–13); ω from Ṙ by finite differences; five-term acceleration | PASS |
| 3 | dynamics | 13 | projectile & symmetric top closed forms (L16/s7–8); T and H conserved; d(H)/dt = M; inertia vs direct discretisation | PASS |
| 4 | integration | 4 | measured RK4 order ≈ 4; agreement with ode45 < 1e-7; lecture Euler-angle ODE reproduces exact rotation | PASS |
| 5 | applications | 12 dice + 14 3R + 14 four-bar | dice free flight analytic; contact passive; varied emergent outcomes; **3R RNE = lecture 2R equations exactly**; four-bar closure ≤ 1e-14 m; independent energy-equation check of FBD dynamics | PASS |
| 6 | visualization | 6 + visual review | drawn geometry equals simulated state | PASS |
| 7 | GUI | 9 + visual review | GUI results bit-identical to core; exact default fidelity; error handling; playback | PASS |
| 8 | end-to-end | 3 | examples from clean path; launcher; automated layer-dependency check | PASS |

## Reference results (`examples/runExamples.m`)

```
Body-fixed X-Y-Z (60,45,-75) deg  = [0.1830 0.6830 0.7071; -0.3245 0.7209 -0.6124; -0.9280 -0.1174 0.3536]  (L2-4/s50)
Space-fixed X-Y-Z (60,45,-75) deg = [0.1830 0.6415 -0.7450; -0.6830 -0.4621 -0.5657; -0.7071 0.6124 0.3536] (L2-4/s44)
Dice free flight 0.3 s: energy drift 5.9e-17 J, angular-momentum drift 3.8e-20
Dice throw (default): top face 1, settled at 0.460 s, 7 contact episodes
3R passive swing 2 s: max |E - E0| = 9.2e-6 J
3R computed-torque move: tracking error 4.2e-12 rad
Four-bar crank-rocker at 360 deg/s: closure 7.7e-16 m, peak torque 1.211 N m, energy balance 1.3e-11 J
```

## Physical sanity checks performed

- Dice at rest on a face: COM height = a/2 − mg/(4k) (static equilibrium of four vertex springs).
- Dice total energy including contact springs never increases (passive contact).
- Isotropic cube inertia ⇒ ω×[I]ω = 0 ⇒ torque-free rotation about a fixed axis (verified).
- 3R: gravity torques = gradient of potential energy (virtual work); τ = Jᵀ F for a tool force.
- Four-bar: static crank torque = ∂V/∂θ1; ground reactions carry total weight; τω = dE/dt.

## Known limitations

- Dice contact is a penalty model (stiffness 2e4 N/m default): contact outcome depends on
  contact parameters and is sensitive to initial conditions, as real dice are.
- Four-bar dynamics use θ1 as generalised coordinate: toggle configurations of the input
  (non-Grashof / change-point linkages) stop with a clear `fourbar:singular` error.
- L7-9/s16 plotted curves are not reproducible from the slide's equations (SD-14).
- MATLAB Online and browser deployment are not tested.
