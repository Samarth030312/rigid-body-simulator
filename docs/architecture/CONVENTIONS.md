# Mechanics Conventions

## Status

**FROZEN — 2026-09-21** (ADR-0006). Changing any item below requires a new ADR in
`docs/DECISIONS.md`, updated tests, and an entry in `docs/IMPLEMENTATION_LOG.md`.

Every item is tagged:

- **[LECTURE]** — taken directly from the supplied ENR209 slides (citation given as
  `deck/slide`, e.g. `L2-4/s9` = deck `L2,L3,L4.pptx`, slide 9).
- **[EXTENSION]** — an engineering choice where the lecture does not fix a unique
  answer. Justification is given.

---

## 1. Frames

| Item | Convention | Tag |
|---|---|---|
| Coordinate systems | Cartesian, right-handed, unit vectors X̂, Ŷ, Ẑ, origin O | [LECTURE] L2-4/s4, L7-9/s3 |
| Frame notation | `{A}`, `{B}`, `{0}`, `{i}` | [LECTURE] L2-4/s7 |
| World / fixed frame | `{A}` for a free rigid body (dice); `{0}` for mechanisms | [LECTURE] L2-4/s7, L7-9/s4 |
| World vertical | `+Ẑ_A` is "up" for 3D scenes | [EXTENSION] matches every 3D lecture figure (L2-4/s13, s44) |
| Gravity (3D) | `g = [0; 0; -9.81]` m/s² in `{A}` | g value [LECTURE] L10-13/s8; direction [EXTENSION] |
| Planar mechanisms | motion in the X–Y plane of `{0}`, joint axes along `+Ẑ_0`, gravity along `-Ŷ_0` | [LECTURE] L17-19/s15 (2R: "gravity in the –ve Y direction") |
| Body frame | `{B}` rigidly attached to the body | [LECTURE] L2-4/s7 |
| Dice body-frame origin | at the centre of mass C, axes along the cube edges | [EXTENSION] lecture dice figures put the origin at a corner (L2-4/s13); the Newton–Euler equations are simplest about C, and the corner offset is a pure geometric shift applied only for display if needed |

## 2. Rotation matrix

| Item | Convention | Tag |
|---|---|---|
| Symbol | `^A_B[R]` — orientation of `{B}` with respect to `{A}` | [LECTURE] L2-4/s8 |
| Columns | `^A_B[R] = [ ^A X̂_B | ^A Ŷ_B | ^A Ẑ_B ]` (axes of `{B}` described in `{A}`) | [LECTURE] L2-4/s8 |
| Mapping direction | `^A r = ^A_B[R] ^B r` (body components → world components) | [LECTURE] L2-4/s9 |
| Properties | `det = +1`, `R^T R = [U]`, `^B_A[R] = ^A_B[R]^{-1} = ^A_B[R]^T` | [LECTURE] L2-4/s9 |
| Interpretation | `^A_B[R]` both (i) describes the body's orientation and (ii) transforms components from `{B}` to `{A}`. Rodrigues' formula (L2-4/s16) also uses `[R]` actively to rotate a vector within one frame (`^A Q' = [R] ^A Q`). Both uses give the same matrix. | [LECTURE] L2-4/s9, s16 |
| MATLAB variable | `R_AB` (3×3 double) means `^A_B[R]` | [EXTENSION] naming |

### 2.1 Simple (elementary) rotations — [LECTURE] L2-4/s19–20

```
R(X,φ) = [1 0 0; 0 cφ -sφ; 0 sφ cφ]
R(Y,φ) = [cφ 0 sφ; 0 1 0; -sφ 0 cφ]
R(Z,φ) = [cφ -sφ 0; sφ cφ 0; 0 0 1]
```

Positive angle = counter-clockwise rotation about the axis when viewed with the axis
pointing at the viewer (right-hand rule). Units: radians internally.

### 2.2 Axis–angle — [LECTURE] L2-4/s14–18

`[R(k̂,φ)]` elements exactly as L2-4/s18. Inverse: `φ = acos((r11+r22+r33−1)/2)`,
`k̂ = (1/(2 sinφ)) [r32−r23, r13−r31, r21−r12]^T` for `φ ∉ {0, π}` (L2-4/s14–15).
The φ = π case is a special case (L2-4/s15); its handling is an [EXTENSION].

### 2.3 Successive rotations and multiplication order

| Rotation type | Composition rule | Tag |
|---|---|---|
| Body-fixed (about moving axes) | **post-multiply**: `^A_B[R] = ^A_{B1}[R] ^{B1}_{B2}[R] … ^{Bn-1}_B[R]` | [LECTURE] L2-4/s21 |
| Body-fixed X–Y–Z Euler angles | `^A_B[R] = R(X,θ1) R(Y,θ2) R(Z,θ3)` | [LECTURE] L2-4/s24–25 |
| Body-fixed Z–Y–Z Euler angles | `^A_B[R] = R(Z,θ1) R(Y,θ2) R(Z,θ3)` | [LECTURE] L2-4/s30 |
| Space-fixed (about fixed axes) | **pre-multiply**: space-fixed X–Y–Z gives `^A_B[R] = R(Z,θ3) R(Y,θ2) R(X,θ1)` | [LECTURE] L2-4/s32–36 (derived there with the axis–angle method; final matrix s36) |
| Equivalence | body-fixed X–Y–Z ≡ space-fixed Z–Y–X with the same angles | [LECTURE] L2-4/s59 |

"Body-fixed" = intrinsic, "space-fixed" = extrinsic (external terminology given only as a
cross-reference; project code and docs use the lecture terms).

Generalisation: any sequence of 3 (or 2 distinct) axes (L2-4/s22) is supported by
`rotBodyFixed(seq, angles)` / `rotSpaceFixed(seq, angles)`. [LECTURE] for the rule,
[EXTENSION] for the general function.

### 2.4 Inverse (rotation matrix → angles)

X–Y–Z: L2-4/s26–27 (atan2 formulas, `r13 = ±1` singular branch).
Z–Y–Z: L2-4/s31 (atan2 formulas, `r33 = ±1` singular branch, choose θ1 = 0).
Principal branch: the `+sqrt` branch. [LECTURE] formulas; branch choice [EXTENSION].

### 2.5 Lecture numerical reference values (used as regression tests)

| Case | Angles (deg) | `^A_B[R]` | Source |
|---|---|---|---|
| Space-fixed X–Y–Z | 60, 45, −75 | `[0.1830 0.6415 −0.7450; −0.6830 −0.4621 −0.5657; −0.7071 0.6124 0.3536]` | L2-4/s44 |
| Body-fixed X–Y–Z | 60, 45, −75 | `[0.1830 0.6830 0.7071; −0.3245 0.7209 −0.6124; −0.9280 −0.1174 0.3536]` | L2-4/s50, s59 |
| Body-fixed Z–Y–Z | 30, 60, 90 | `[−0.5 −0.4330 0.75; 0.8660 −0.25 0.4330; 0 0.8660 0.5]` | L2-4/s64 |
| Rotation about Z | 45 | `[1/√2 −1/√2 0; 1/√2 1/√2 0; 0 0 1]` | L2-4/s12 |

## 3. Position and pose

| Item | Convention | Tag |
|---|---|---|
| Point in body | `^A p = ^A O_B + ^A_B[R] ^B p` | [LECTURE] L7-9/s20 |
| Pose storage | position `^A O_B` (3×1, m) + `^A_B[R]` (3×3). Homogeneous 4×4 transforms are **not** used in the core. | [EXTENSION] lecture uses `(O, R)` pairs; avoids introducing notation not taught |

## 4. Velocity and angular velocity

| Item | Convention | Tag |
|---|---|---|
| Linear velocity | `^0V_{O_i} = d/dt ^0O_i`; described in `{j}` via `^j_0[R] ^0V` | [LECTURE] L7-9/s4 |
| Space-fixed angular velocity | `^0_i[Ω]_R = ^0_i[Ṙ] ^0_i[R]^T`, vector `^0ω_i^s` (components along fixed axes) | [LECTURE] L7-9/s5–6, s9 |
| Body-fixed angular velocity | `^0_i[Ω]_L = ^0_i[R]^T ^0_i[Ṙ]`, vector `^0ω_i^b` (components along body axes) | [LECTURE] L7-9/s9–10 |
| Skew matrix layout | `[Ω] = [0 −ωz ωy; ωz 0 −ωx; −ωy ωx 0]`, `[Ω] p = ω × p` | [LECTURE] L7-9/s6 |
| Relation | `^0ω_i^s = ^0_i[R] ^0ω_i^b`; `[Ω]_R = R [Ω]_L R^T` | [LECTURE] L7-9/s10 |
| Orientation kinematics used for integration | `Ṙ = [Ω]_R R = R [Ω]_L` | [LECTURE] follows directly from the two definitions (L7-9/s5, s9, s22) |
| Euler-angle rates, X–Y–Z | `ωx^s = θ̇1 + θ̇3 s2`, `ωy^s = θ̇2 c1 − θ̇3 s1 c2`, `ωz^s = θ̇2 s1 + θ̇3 c1 c2` | [LECTURE] L7-9/s7, s14, s16 |
| Euler-angle rates, Z–Y–Z | `ω^s = [θ̇3 c1 s2 − θ̇2 s1; θ̇3 s1 s2 + θ̇2 c1; θ̇3 c2 + θ̇1]`, `ω^b = [−θ̇1 s2 c3 + θ̇2 s3; θ̇1 s2 s3 + θ̇2 c3; θ̇1 c2 + θ̇3]` | [LECTURE] L7-9/s8, s10 |
| Angular acceleration | `^Aα_B = d/dt ^Aω_B^s` | [LECTURE] L7-9/s22 |
| Point acceleration | `^Aa_p = ^Aa_{O_B} + α × (R ^Bp) + ω^s × (ω^s × R ^Bp) + 2 ω^s × R ^BV_p + R ^Ba_p` | [LECTURE] L7-9/s23–24 |

## 5. Units

SI throughout: m, kg, s, N, N·m, rad, rad/s, kg·m². Degrees appear **only** at the GUI /
report boundary and are converted immediately. [EXTENSION] (the lectures use SI:
L10-13/s8, L14-15/s14).

## 6. Dynamics conventions

### 6.1 Mass properties

| Item | Convention | Tag |
|---|---|---|
| Inertia matrix | `^B[I]_C = −∫[^Br][^Br] ρ dV` about the centre of mass C, components in `{B}`; diagonal `I_xx = ∫(y²+z²)ρdV`; **products of inertia enter with a minus sign** (`I_xy = −∫xy ρdV`) | [LECTURE] L14-15/s6, L17-19/s4–5 |
| Frame change | `^A[I]_C = ^A_B[R] ^B[I]_C ^A_B[R]^T` | [LECTURE] L14-15/s8, L16/s4 |
| Parallel axis | `[I]_O = [I]_C − m [d]²` with `d` from C to O (corrected, SD-05) | [LECTURE] L14-15/s8 |
| Box | `diag(m(b²+c²), m(a²+c²), m(a²+b²))/12` for edges a, b, c along X̂, Ŷ, Ẑ | [LECTURE] L14-15/s9 |
| Cube | box with a = b = c: `(m a²/6) [U]` | [LECTURE] special case of L14-15/s9 |

### 6.2 Single rigid body (dice)

| Item | Convention | Tag |
|---|---|---|
| Newton | `m ^A a_C = ^A F` (sum of all external forces, inertial `{A}`) | [LECTURE] L17-19/s6 |
| Euler | `^B[I]_C ^A ω̇^b = ^B M_C − ^A ω^b × ^B[I]_C ^A ω^b`, moments about C in `{B}`; `^B M = ^A_B[R]^T ^A M` | [LECTURE] L16/s6, L17-19/s8 |
| Orientation kinematics | `Ṙ = R [ω^b ×]` | [LECTURE] L7-9/s9 |
| State vector (18×1) | `x = [ ^A r_C (1:3, m) ; ^A V_C (4:6, m/s) ; R(:) (7:15, column-major ^A_B[R]) ; ^A ω^b_B (16:18, rad/s, in {B}) ]` | [EXTENSION] — the rotation matrix is the orientation state (L2-4/s69: "ideal for analysis", no singularities); Euler angles are *derived outputs* only |
| Why ω^b (not ω^s) | Euler's equation is posed in `{B}` where `^B[I]_C` is constant (L16/s4, s6) | [LECTURE] reasoning |
| Moments | about the centre of mass C | [LECTURE] L16/s6 |

### 6.3 Planar serial chain (3R)

| Item | Convention | Tag |
|---|---|---|
| Frames | `{0}` fixed at O_1; `{i}` at joint O_i with X̂_i along link i; joint axes `+Ẑ` out of the page; `{Tool}` at the end of link 3, parallel to `{3}` | [LECTURE] L7-9/s49 |
| Joint angles | θ_i relative, CCW from X̂_{i−1} to X̂_i (θ1 from X̂_0) | [LECTURE] L7-9/s49, L20-21/s15 |
| Link parameters | `l_i` length, `m_i` mass, `r_i` distance from O_i to C_i along X̂_i, `I_i` = I_zz about C_i | [LECTURE] L20-21/s13 |
| Gravity | along −Ŷ_0 (implemented by `⁰V̇₀ = +g Ŷ_0`) | [LECTURE] L20-21/s11, s14 |
| Equation of motion | `τ = M(θ) θ̈ + C(θ,θ̇) + G(θ)` | [LECTURE] form of L20-21/s19 |
| Inverse dynamics | recursive Newton–Euler, `τ_i = ^i n_i · ^i Ẑ_i` | [LECTURE] L20-21/s10–11 |
| Forward dynamics | `θ̈ = M⁻¹(τ − C − G)`; M assembled column-by-column from the RNE with g = 0, θ̇ = 0, θ̈ = e_j | [EXTENSION] |
| State vector (6×1) | `x = [θ1; θ2; θ3; θ̇1; θ̇2; θ̇3]` (rad, rad/s) | [EXTENSION] |
| Torque sign | positive CCW about +Ẑ (joint axis) | [LECTURE] L20-21/s15 figure |

### 6.4 Four-bar mechanism

| Item | Convention | Tag |
|---|---|---|
| Geometry | ground l0 from O_L (origin of `{0}`) to O_R = (l0, 0); crank l1 (link 1), coupler l2 (link 2), output/rocker l3 (link 3) | [LECTURE] L5-6/s20 |
| Angles | θ1 = crank angle, CCW from X̂_L (input); φ1 = output angle, CCW from X̂_R; relative angles φ2, φ3 as on L5-6/s20 | [LECTURE] L5-6/s20–22 |
| Constraint | Freudenstein form η1(θ1, φ1) = 0 (L5-6/s22); full three-equation loop closure (L5-6/s21) reported as residual | [LECTURE] |
| Coupler angle | absolute coupler angle `θ2 = θ1 + φ2` (derived output) | [EXTENSION] naming |
| Assembly branch | `+1` = "open" (coupler joint B above the diagonal O_L-A→O_R side as defined in `fourbar/` docs), `−1` = "crossed"; fixed for a simulation, never switched silently | [EXTENSION] |
| State vector (2×1) | `x = [θ1; θ̇1]` (1 DOF, L5-6/s16/s20) | [LECTURE] DOF; [EXTENSION] choice of θ1 as generalised coordinate (L5-6/s21 "actuated joint θ = θ1") |
| Link mass properties | `m_i`, COM at distance `r_i` from the link's first joint along the link, `I_i` about COM | [EXTENSION] (mirrors L20-21/s13) |
| Gravity | −Ŷ_0 | [EXTENSION] consistent with planar chain |

### 6.5 Numerical integration — [EXTENSION] (lectures: "by numerical integration – using Matlab", L7-9/s14)

- Primary: fixed-step classical Runge–Kutta 4 (explainable, deterministic, uniform output
  grid for animation). After each step the rotation block is projected onto SO(3)
  (`orthonormalize`) and the projection size is logged.
- Reference: MATLAB `ode45` with tight tolerances for comparison studies.
- Every simulation logs: time step, solver, orthonormality error, energy, momentum, and (for
  the four-bar) loop-closure residual.

## 7. Dice face numbering — [LECTURE] L2-4/s13, s44, s50, s64

Verified from the three orthographic views on L2-4/s44 (initial configuration):

| Body face normal | Face value |
|---|---|
| +X̂_B | 4 |
| −X̂_B | 3 |
| +Ŷ_B | 2 |
| −Ŷ_B | 5 |
| +Ẑ_B | 1 |
| −Ẑ_B | 6 |

Opposite faces sum to 7. The face shown "up" is the face whose outward normal, expressed in
`{A}` (`^A n = ^A_B[R] ^B n`), has the largest `+Ẑ_A` component. [EXTENSION] for the
"up face" rule; the lecture does not define an outcome rule.
