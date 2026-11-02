# Code layout

```
setup.m                         the only entry point: path, checks, app, tests
matlab/
  setup_path.m                  puts this repository (and nothing else) on the path
  functions/+rbsim/
    doctor.m                    what this computer can and cannot do
    +math/                      rotations, Euler angles, axis-angle, skew matrices
    +body/                      points, angular velocity, inertia, Newton-Euler, energy
    +solver/                    rk4 (fixed step, SO(3) projection hook), ode45ref
    +dice/                      cube geometry, contact, dynamics, simulate
    +robot/                     planar 3R kinematics, recursive Newton-Euler, control
    +fourbar/                   loop closure, kinematics, free-body dynamics
    +viz/                       scenes and dashboards, driven by a result struct
  apps/rigid_body_lab/          rigid_body_lab.m (entry) and RigidBodyLabApp.m (class)
  scripts/                      worked examples, the playground, figure generation
  tests/                        matlab.unittest suite, e2e/ for the whole-workflow tests
docs/                           this documentation
scripts/validation/             repository hygiene check used by CI
```

## Layers

Each package may use the ones below it and never the ones above.

| Layer | Package | Depends on |
|---|---|---|
| 7 app | `matlab/apps` | everything below |
| 6 drawing | `+viz` | the result struct only |
| 5 models | `+dice`, `+robot`, `+fourbar` | 1–4 |
| 4 integration | `+solver` | 1 |
| 3 dynamics | `+body` | 1–2 |
| 2 kinematics | `+body` | 1 |
| 1 mathematics | `+math` | base MATLAB |

`test_end_to_end/testTheModelsDoNotDependOnTheAppOrTheGraphics` enforces the top two rows
by inspection of the source.

## How a simulation flows

```
defaults()            a parameter struct with every knob at a sensible value
   |
validate(p)           readable errors; nothing downstream re-checks
   |
initialState(p)       the state vector the integrator will advance
   |
dynamics(t, x, p)     the state derivative: Newton, Euler, constraints, contact
   |
rbsim.solver.rk4      fixed step, SO(3) projection after each step
   |
simulate(p)           adds diagnostics and the summary, returns the result struct
   |
rbsim.viz.*           scene and dashboard, read-only
```

## The result struct

Every model returns the same shape, which is what lets one app and one dashboard serve all
three.

| Field | Meaning |
|---|---|
| `t` | 1×N sample times, s |
| `x` | n×N state history, layout per `CONVENTIONS.md` §6 |
| `params` | the validated parameter struct the run used |
| `solver` | name, step, wall time, number of steps, size of the largest SO(3) correction |
| `diag` | per-sample diagnostics: energies, momenta, residuals, torques, reactions |
| `summary` | the scalar outcomes of the run, the things a table would show |

## Naming

- Packages and functions are lower camel case: `rbsim.robot.forwardKinematics`.
- A function's help block starts `%NAME  one line`, then usage, inputs, outputs, and the
  equations it implements; the body is indented four spaces below it.
- Error identifiers are `rbsim:<package>:<tag>`.
- Tests are `matlab/tests/test_<area>.m`, one assertion family per function, and read
  `tc.verifyEqual(actual, expected, ..., 'why this must hold')`.
