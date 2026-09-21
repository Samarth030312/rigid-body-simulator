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
| 3 | dynamics | — | — | — | NOT TESTED | — |
| 4 | integration | — | — | — | NOT TESTED | — |
| 5 | applications | — | — | — | NOT TESTED | — |
| 6 | visualization | — | — | — | NOT TESTED | — |
| 7 | GUI | — | — | — | NOT TESTED | — |
| 8 | end-to-end | — | — | — | NOT TESTED | — |
