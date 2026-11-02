# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Unrecognized function or variable 'rbsim'` | the path is not set | run `setup` from the repository root |
| `setup` reports **FAIL Repository on the path** | MATLAB was started elsewhere and `matlab/functions` is missing from the path | `cd` to the repository root and run `setup` again |
| The app does not open, but simulations run | no display (a head-less session or a remote shell without X) | use the models and `rigid_body_examples` directly; `rbsim.doctor` reports this as a warning |
| `rbsim:dice:param  Time step too large for the contact stiffness` | the explicit integrator would be unstable at that stiffness | reduce `dt`, or reduce `contact.stiffness`; the message gives the largest step that will work |
| `rbsim:dice:param  Initial height must keep the whole cube above the floor` | the die starts intersecting the floor | raise `r0(3)` to at least `a√3/2` above the floor |
| `rbsim:fourbar:param  The loop cannot be assembled at the initial crank angle` | those four lengths cannot close at that angle | change a link length, or start at a crank angle that does assemble; `rbsim.fourbar.grashof` says whether full rotation is possible at all |
| `rbsim:fourbar:singular  Toggle (singular) configuration` | coupler and rocker are collinear, where the velocity relation has no solution | expected for change-point linkages; start away from the toggle or choose lengths that avoid it |
| `rbsim:math:singular  X-Y-Z Euler angle singularity` | the Euler-angle rate map was asked for at `cos θ₂ = 0` | use the rotation-matrix path, which has no such point; this error exists so the singularity cannot pass unnoticed |
| Tests fail after editing a defaults file | some tests assert the shipped defaults | revert the file, or update the test that names it |
| A simulation is slow | the step is set by the stiffest element, usually contact | raise `outputEvery` to store fewer samples, or lower the contact stiffness |
