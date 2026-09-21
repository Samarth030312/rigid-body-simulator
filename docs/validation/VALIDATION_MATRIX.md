# Validation Matrix (evidence)

Every row records an executed check. Environment: MATLAB R2026a Update 5
(26.1.0.3346908), Ubuntu 22.04.5, base MATLAB only. Command: `./scripts/run_tests.sh`.

| Level | Test (file/function) | Expected | Observed | Tolerance | Result | Date |
|---|---|---|---|---|---|---|
| 0 | `scripts/check_environment.sh` | MATLAB launches, release recorded | R2026a Update 5 | — | PASS | 2026-09-21 |
| 0 | entry point `launchRigidBodySimulator` | runs without error | prints banner | — | PASS | 2026-09-21 |
| 0 | `scripts/verify_sources.sh` | SHA-256 of 10 sources match manifest | all match | exact | PASS | 2026-09-21 |
| 1 | `test_rotations` (27 tests) | simple rotations proper orthonormal; right-hand rule; L2-4/s12, s44, s50, s64 matrices; closed forms s25, s36; space-fixed derivation replay; body XYZ ≡ space ZYX; Rodrigues; axis–angle round trip & special cases; eigenvalues; Euler XYZ/ZYZ inverse incl. singular branches; SO(3) projection | all as expected | 1e-12 algebraic, 5e-5 vs 4-decimal lecture values | PASS | 2026-09-21 |
| 2 | `test_angular_velocity` (9 tests) | `[Ω]p = ω×p`; X–Y–Z and Z–Y–Z ω formulas equal ω from finite-difference `Ṙ`; L7-9/s12–13 values; Euler-rate inverse; `Ṙ = R[ω^b×] = [ω^s×]R`; L7-9/s11; point velocity & five-term acceleration vs finite differences | all as expected | 1e-8 (FD), 5e-4 vs lecture | PASS | 2026-09-21 |
| 3 | `test_inertia` (6) | cube `m a²/6 [U]`; box, rotated box and parallel-axis results equal a direct discretisation of `−Σ m_k [r_k][r_k]` (sign of products of inertia); corrected parallel-axis (3,3) entry (SD-05); perpendicular axis; SPD | as expected | 1e-3 rel (midpoint rule, n=40), 1e-14 exact | PASS | 2026-09-21 |
| 3 | `test_rigid_body_dynamics` Level-3 part (7) | body at rest stays at rest; pure translation; projectile `r0+v0t+gt²/2`; cube torque-free rotation about fixed axis `R0 R(ŵ,|w|t)`; symmetric top closed form (L16/s7–8); asymmetric torque-free T and `^AH_C` conservation; `d(^AH_C)/dt = R ^BM` (Newton–Euler consistency); constant principal torque; gravity energy conservation | as expected | 1e-12…2e-8 (RK4 bound), conservation 1e-8 rel | PASS | 2026-09-21 |
| 4 | `test_rigid_body_dynamics` Level-4 part (4) | lecture Euler-angle ODE (L7-9/s14–16) reproduces exact constant-ω rotation; RK4 order measured 3.7–4.3; RK4 (dt 1e-3, SO(3) projection) vs ode45 (1e-10); stop function & output decimation | order ≈ 4.0; RK4–ode45 < 1e-7 | as stated | PASS | 2026-09-21 |
| 4 | L7-9/s16 plot | slide plot reproduced | NOT reproducible from the slide's own ODE — recorded as SD-14; the exact-solution check above is authoritative | — | N/A (source defect) | 2026-09-21 |
| 5 | applications | — | — | — | NOT TESTED | — |
| 6 | visualization | — | — | — | NOT TESTED | — |
| 7 | GUI | — | — | — | NOT TESTED | — |
| 8 | end-to-end | — | — | — | NOT TESTED | — |
