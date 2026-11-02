# Roadmap

Current state: 1.0.0, three models validated, 115 tests passing. The open limitations
behind these items are listed in
[docs/knowledge-base/OPEN_ITEMS.md](docs/knowledge-base/OPEN_ITEMS.md).

## Next

- **Contact that does not set the step (OI-07).** The explicit integrator must resolve the
  stiffest corner spring, which fixes the step for the whole run. A semi-implicit step for
  the contact forces, or an impulse-based collision response, would let the free-flight
  phase run at a sensible step and make long throws cheap.
- **A measured die (OI-01, OI-05).** Give the faces the mass a hollowed pip actually
  removes, then compare the simulated face distribution over many throws with a recorded
  one. That turns the contact model from plausible into calibrated.
- **Windows and macOS in routine CI (OI-03).** The workflow already fans out across the
  three platforms; the claim of an R2021b floor should be replaced by a run on R2021b.

## Later

- **Through the toggle (OI-02).** Switching the generalised coordinate from the crank to
  the rocker at a change point would let non-Grashof linkages be driven through positions
  that currently stop the solver.
- **A spatial arm (OI-06).** A three-dimensional manipulator needs a frame-assignment
  convention and a second set of kinematics tests. The dynamics routine is already written
  in three dimensions and would not change.
- **Constraint drift in closed chains.** The four-bar solves its constraint at every sample,
  which is exact but specific to one loop. A general constraint solver with Baumgarte
  stabilisation would carry the approach to other linkages.
- **MATLAB Online and a browser build (OI-04).** The core is plain MATLAB and should run on
  MATLAB Online unchanged; that has not been tried, and no claim is made until it is.
