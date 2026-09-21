# Persistent Context Ledger

Read this first when resuming work.

## Mission

Rigorous, viva-defensible MATLAB ENR209 rigid-body simulator: dice, planar 3R manipulator,
four-bar mechanism, interactive GUI with dashboards. Base MATLAB R2026a only.

## Where the truth lives

| Topic | File |
|---|---|
| Requirements | `docs/requirements/PROJECT_REQUIREMENTS.md` |
| Lecture evidence | `docs/concepts/LECTURE_CONCEPT_MAP.md`, `docs/concepts/SOURCE_DEFECTS.md` |
| Conventions (FROZEN) | `docs/architecture/CONVENTIONS.md` |
| Architecture (FROZEN) | `docs/architecture/architecture.md` |
| Decisions | `docs/DECISIONS.md` (ADR-0001…0013) |
| Validation evidence | `docs/validation/VALIDATION_MATRIX.md`, `TRACEABILITY_MATRIX.md` |
| Progress | `docs/PROJECT_STATUS.md`, `docs/IMPLEMENTATION_LOG.md` |

## Key conventions (summary — CONVENTIONS.md is authoritative)

- `^A_B[R]` columns = body axes in `{A}`; `^A r = R ^B r`; body-fixed post-multiply,
  space-fixed pre-multiply.
- `[Ω]_R = ṘRᵀ → ω^s`, `[Ω]_L = RᵀṘ → ω^b`, `Ṙ = R[ω^b×]`.
- Euler's equation in `{B}` at C; dice state `[r_C; V_C; R(:); ω^b]` (18×1).
- 3D gravity −Ẑ_A; planar gravity −Ŷ_0. SI, radians.
- Dice faces: +X 4, −X 3, +Y 2, −Y 5, +Z 1, −Z 6.

## Slide renders

Evidence renders are regenerated on demand (not committed):
`soffice --headless --convert-to pdf …; pdftoppm -r 80 -png …`.

## Immediate next action

Phase 7: four-bar (`fourbar/`): Freudenstein position solution (L5-6/s22) with explicit assembly branch, full loop-closure residual (L5-6/s21), velocity/acceleration from the differentiated constraint, Grashof classification, dynamics by per-link Newton–Euler + FBD (ADR-0011), energy/power validation.
rigidBodyDerivative, energy/momentum) and `core/integration` (RK4 + projection), with
Level 3–4 tests (gravity analytic, torque-free top L16/s7–8, Euler-angle vs rotation-matrix
integration of L7-9/s16, RK4 convergence order).
