# How it works

## One shape for every model

The die, the arm and the linkage are different mechanics but the same pipeline, which is
why one app and one dashboard serve all three.

```
rbsim.<model>.defaults()        every knob, at a sensible value
rbsim.<model>.validate(p)       readable errors, once, at the front
rbsim.<model>.initialState(p)   the state the integrator will advance
rbsim.<model>.dynamics(t,x,p)   the state derivative
rbsim.solver.rk4(...)           fixed step; rotation re-projected onto SO(3)
rbsim.<model>.simulate(p)       the above, plus diagnostics and a summary
rbsim.viz.*Scene / dashboard    drawn from the result, never recomputed
```

`simulate` is the only function the app calls. Everything it returns is in the result
struct described in [architecture/MATLAB](architecture/MATLAB.md).

## What the state is, per model

| Model | State | Size |
|---|---|---|
| Die | `[r_C; V_C; R(:); ω^b]` | 18 |
| 3R arm | `[q; q̇]` | 6 |
| Four-bar | `[θ₁; θ̇₁]` | 2 |

The linkage has two numbers because a four-bar has one degree of freedom: the other seven
angles and positions are solved from the loop-closure constraint at every sample rather
than integrated, which is why its closure residual stays at the level of rounding instead
of drifting.

## Why the die carries nine numbers for its orientation

Any three-angle description of orientation has a configuration where two of the angles
describe the same rotation and the rates blow up. A tumbling die reaches that
configuration routinely. Carrying the rotation matrix and integrating `Ṙ = R[ω^b ×]` has no
such point. The price is that numerical error slowly spoils `RᵀR = I`, so after every step
the matrix is replaced by the nearest proper rotation and the size of that correction is
recorded in `solver.maxCorrection` — it is typically 10⁻⁷ over a whole throw.

## Where the work is done

| Question | Answered by |
|---|---|
| What is this body's acceleration? | `rbsim.body.derivative` — Newton in the world frame, Euler about the centre of mass in the body frame |
| What torque does this arm need? | `rbsim.robot.newtonEuler` — outward pass for motion, inward pass for force |
| What does this arm do if I apply that torque? | the same routine, used once per joint to build `M(q)`, then solved |
| Where is this linkage? | `rbsim.fourbar.position` — two circles intersected, branch chosen explicitly |
| How fast is it moving? | `rbsim.fourbar.kinematics` — the differentiated loop closure, a 2×2 solve |
| What holds it together? | `rbsim.fourbar.newtonEuler` — nine equations, eight reaction components and one unknown |

## Diagnostics are part of the result, not an afterthought

Each run carries the quantities that would reveal a mistake: kinetic and potential energy,
the work done by every non-conservative element, angular momentum, orthonormality error,
loop-closure residual, transmission angle, joint reactions. The dashboards plot them and
the tests assert on them. The work term is integrated alongside the state rather than
summed afterwards, so the energy balance is accurate to the integrator rather than to the
output sampling.
