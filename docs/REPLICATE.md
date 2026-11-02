# Replicate

Everything below runs on base MATLAB R2021b or newer. No toolbox, no compiler, nothing to
install.

## Run it

```matlab
cd rigid-body-simulator
setup open
```

`setup` puts the repository on the path, checks the computer and opens the app. Pick a tab,
change the numbers, press **Run**, then **Play** or drag the time slider.

Without the app:

```matlab
setup
res = rbsim.dice.simulate(rbsim.dice.defaults());      res.summary
res = rbsim.robot.simulate(rbsim.robot.defaults());    res.summary
res = rbsim.fourbar.simulate(rbsim.fourbar.defaults()); res.summary
```

## Check it

```matlab
setup test
```

or, from a shell:

```bash
./scripts/run_tests.sh
```

115 tests, about two minutes. Any failure exits non-zero and names the test. The same suite
runs from the app's **Validation** tab.

## Reproduce the reference numbers

```matlab
rigid_body_examples
```

Prints the closed-form rotation matrices, the die's energy and angular-momentum drift in
free flight, the outcome of the default throw, the arm's energy conservation and tracking
error, and the linkage's closure residual and peak torque. These are the numbers quoted in
[validation/VALIDATION](validation/VALIDATION.md).

## Change it

Open `matlab/scripts/rigid_body_playground.m`, edit a section, press **Ctrl+Enter**. Three
sections, one per model, each ending in an animated figure and a dashboard.

The defaults live in one file per model and are safe to edit:

| File | What to change |
|---|---|
| `matlab/functions/+rbsim/+dice/defaults.m` | edge, mass, throw, spin, contact stiffness, damping, friction |
| `matlab/functions/+rbsim/+robot/defaults.m` | link lengths, masses, centres of mass, control mode, gains, target |
| `matlab/functions/+rbsim/+fourbar/defaults.m` | the four link lengths, branch, drive mode, torque or speed, load |
| `matlab/functions/+rbsim/+viz/theme.m` | colours |

Some tests assert the shipped defaults; after changing them, `setup test` will say which.

Things worth trying: switch the die's floor contact off and watch the energy become exactly
constant; give the linkage lengths that break Grashof's condition and watch it refuse to
assemble at some crank angles; set the arm's damping to zero and confirm the energy drift
stays at 10⁻⁶ J over two seconds.

## Regenerate the figures

```matlab
setup figures
```

Writes `docs/img/dice.png`, `robot.png` and `fourbar.png` from fresh simulations.
