# System Architecture

Status: **FROZEN — 2026-09-21** (layering and folder contract). Conventions:
[`CONVENTIONS.md`](CONVENTIONS.md). Decisions: [`../DECISIONS.md`](../DECISIONS.md).

## Layers and folders

| Layer | Folder | Responsibility | May depend on |
|---|---|---|---|
| 1 Mathematical foundations | `core/math/` | skew matrices, rotation matrices, Euler angles, axis–angle, SO(3) checks | base MATLAB |
| 2 Kinematics | `core/kinematics/` | point position/velocity/acceleration, angular velocity, `Ṙ` | 1 |
| 3 Dynamics | `core/dynamics/` | inertia, gravity, Newton–Euler rigid-body derivative, energy/momentum | 1–2 |
| 4 Integration | `core/integration/` | RK4 fixed step (+ SO(3) projection hook), ode45 wrapper, logging | 1 |
| 5 Applications | `dice/`, `robot/`, `fourbar/` | model parameters, initial states, application physics, simulation drivers | 1–4 |
| 6 Visualization | `visualization/` | drawing of simulated states, frames, trajectories, dashboards (axes handles in, graphics out) | 1–5 (reads results only) |
| 7 GUI | `app/` | parameter entry, validation, run/replay, displays | 1–6 |

Lower layers never call upper layers. No function below layer 7 creates UI controls; no
function below layer 6 creates graphics.

## Simulation data flow

```
params struct  --(app)defaultParams / validateParams-->
x0             --(app)initialState-->
f(t,x)         --(app)dynamics: state derivative, uses core/dynamics-->
integrate      --(core/integration)rk4 with post-step projection-->
result struct  --(app)postprocess: diagnostics (energy, residuals, outcome)-->
visualization  --(visualization)draw*/plot* from result only-->
GUI            --(app) orchestrates, never computes physics
```

## Result struct contract (all applications)

| Field | Meaning |
|---|---|
| `t` | 1×N time samples, s |
| `x` | n×N state history (layout per CONVENTIONS §6) |
| `params` | parameter struct used |
| `solver` | struct: name, dt, wall time, projection statistics |
| `diag` | struct of diagnostic time series (energy, momentum, residuals, orthonormality error) |
| `summary` | struct of scalar outcomes (e.g. final dice face, max closure residual) |

## Testing contract

`tests/` mirrors the source tree. `tests/runAllTests.m` runs every test via
`matlab.unittest` (base MATLAB); `scripts/run_tests.sh` runs it headlessly. Validation
levels are defined in `docs/validation/VALIDATION_PLAN.md`; evidence in
`docs/validation/VALIDATION_MATRIX.md`.
