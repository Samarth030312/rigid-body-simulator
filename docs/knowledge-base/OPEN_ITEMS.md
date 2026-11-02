# Open items

Known limitations and the work they imply. Closed items keep their number.

| ID | Item | Status |
|---|---|---|
| OI-01 | Contact is a penalty model. The outcome of a throw depends on the stiffness, damping and friction chosen, and on the initial conditions — as a real die does, but it means the face distribution is not a calibrated prediction of a physical die. | open; documented in ADR-0012 |
| OI-02 | The four-bar generalised coordinate is the crank angle, so a non-Grashof or change-point linkage that reaches a toggle stops with `rbsim:fourbar:singular` rather than continuing through it. Continuing would need a coordinate switch at the toggle. | open |
| OI-03 | Only MATLAB R2026a is exercised. R2021b is the stated floor on the strength of the language features used, not on a test run; CI installs the latest release, not the floor. | open; CI runs the suite on Linux, Windows and macOS |
| OI-04 | MATLAB Online and any browser deployment are untried. | open |
| OI-05 | The die's inertia is uniform. Real dice have hollowed pips and are measurably biased; modelling that would need a per-face mass offset. | open, low priority |
| OI-06 | The arm is planar by decision ADR-0010. A spatial version would need a frame-assignment convention and a second set of kinematics tests. | open, by design |
| OI-07 | The integrator is fixed-step. Stiff contact therefore sets the step for the whole simulation; an adaptive or semi-implicit contact step would be faster. | open, performance only |
| OI-08 | The four-bar solves the angle at `O_R` with `acos` of the law of cosines, which loses half its significant digits as the cosine approaches ±1 — the same conditioning defect that was fixed in the axis–angle inverse. It is not reached by anything shipped: over 50 001 crank angles the default linkage stays at `|cos γ| ≤ 0.753` and the loop closes to 9.9×10⁻¹⁶ m. A change-point linkage (all four links equal) does reach `|cos γ| = 1` at `θ₁ = π`, where the measured angle discrepancy against the stable form is ≤10⁻¹⁰ rad. Neither residual can detect it, because both the loop closure and the coupler length are stationary at a toggle. A stable form (`atan2` of the triangle area, Kahan's ordered Heron) was written and measured, then dropped: it cost 0.46 µs per call against an improvement no test could demonstrate. | open, latent; revisit if a toggle-crossing coordinate switch is added (OI-02) |
