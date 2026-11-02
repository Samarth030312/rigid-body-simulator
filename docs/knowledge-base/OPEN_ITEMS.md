# Open items

Known limitations and the work they imply. Closed items keep their number.

| ID | Item | Status |
|---|---|---|
| OI-01 | Contact is a penalty model. The outcome of a throw depends on the stiffness, damping and friction chosen, and on the initial conditions — as a real die does, but it means the face distribution is not a calibrated prediction of a physical die. | open; documented in ADR-0012 |
| OI-02 | The four-bar generalised coordinate is the crank angle, so a non-Grashof or change-point linkage that reaches a toggle stops with `rbsim:fourbar:singular` rather than continuing through it. Continuing would need a coordinate switch at the toggle. | open |
| OI-03 | Only MATLAB R2026a on Linux is exercised routinely. R2021b is the stated floor on the strength of the language features used, not on a test run. | open; CI covers Linux only |
| OI-04 | MATLAB Online and any browser deployment are untried. | open |
| OI-05 | The die's inertia is uniform. Real dice have hollowed pips and are measurably biased; modelling that would need a per-face mass offset. | open, low priority |
| OI-06 | The arm is planar by decision ADR-0010. A spatial version would need a frame-assignment convention and a second set of kinematics tests. | open, by design |
| OI-07 | The integrator is fixed-step. Stiff contact therefore sets the step for the whole simulation; an adaptive or semi-implicit contact step would be faster. | open, performance only |
