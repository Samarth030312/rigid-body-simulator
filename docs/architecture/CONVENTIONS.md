# Conventions

Status: **frozen**. Every model, drawing routine and test in this repository is written
against the contracts below. Changing one means a new entry in
[`../knowledge-base/ENGINEERING_DECISIONS.md`](../knowledge-base/ENGINEERING_DECISIONS.md),
updated tests and a line in the changelog.

Each row is marked **standard** (the usual result of rigid-body mechanics, as in the works
listed in [`REFERENCES.md`](../knowledge-base/REFERENCES.md)) or **choice** (one of several
defensible options; the reason is given).

---

## 1. Frames

| Item | Convention | |
|---|---|---|
| Coordinate systems | Cartesian, right-handed, unit vectors X̂, Ŷ, Ẑ about an origin O | standard |
| Frame names | `{A}` world and `{B}` body for a free body; `{0}` ground and `{i}` link *i* for a mechanism | standard |
| World vertical | `+Ẑ_A` is up in three dimensions | choice: matches the way the scenes are drawn |
| Gravity, 3-D | `g = [0; 0; −9.81]` m/s² in `{A}` | standard value, direction by the row above |
| Planar mechanisms | motion in the X–Y plane of `{0}`, joint axes along `+Ẑ_0`, gravity along `−Ŷ_0` | choice: the usual planar drawing convention |
| Body frame origin | at the centre of mass C, axes along the cube edges | choice: Euler's equation is simplest about C |

## 2. Rotation matrix

| Item | Convention | |
|---|---|---|
| Symbol | `^A_B[R]`, the orientation of `{B}` seen from `{A}` | standard |
| Columns | `^A_B[R] = [ ^A X̂_B │ ^A Ŷ_B │ ^A Ẑ_B ]` | standard |
| Mapping | `^A r = ^A_B[R] ^B r` — body components to world components | standard |
| Properties | `det R = +1`, `RᵀR = I`, `^B_A[R] = ^A_B[R]ᵀ` | standard |
| MATLAB name | `R_AB` means `^A_B[R]` | choice |

### 2.1 Elementary rotations

```
R(X,φ) = [1 0 0; 0 cφ −sφ; 0 sφ cφ]
R(Y,φ) = [cφ 0 sφ; 0 1 0; −sφ 0 cφ]
R(Z,φ) = [cφ −sφ 0; sφ cφ 0; 0 0 1]
```

A positive angle turns counter-clockwise about the axis seen from its positive end
(right-hand rule). Angles are radians everywhere inside the code.

### 2.2 Axis and angle

Rodrigues' formula, `R(k̂, φ)`, with the inverse
`φ = acos((tr R − 1)/2)` and `k̂ = (r32−r23, r13−r31, r21−r12)ᵀ / (2 sin φ)` for
`φ ∉ {0, π}`. The two singular cases are handled explicitly: `φ ≈ 0` returns `k̂ = Ẑ` by
convention, and `φ ≈ π` takes the axis from the largest diagonal entry of `(R + I)/2`
(**choice**, since the axis is otherwise undetermined or sign-free there).

### 2.3 Order of successive rotations

| Rotation about | Composition | |
|---|---|---|
| axes carried with the body | **post-multiply**: `^A_B[R] = R₁ R₂ R₃` | standard |
| axes fixed in space | **pre-multiply**: `^A_B[R] = R₃ R₂ R₁` | standard |

So a body-fixed X–Y–Z triple is `R(X,θ₁) R(Y,θ₂) R(Z,θ₃)`, a space-fixed X–Y–Z triple is
`R(Z,θ₃) R(Y,θ₂) R(X,θ₁)`, and a body-fixed X–Y–Z triple equals the space-fixed Z–Y–X
triple of the same angles. `rbsim.math.rotBodyFixed` and `rbsim.math.rotSpaceFixed` take any
sequence of three axes with no two neighbours equal, so all twelve triples of each kind work.

### 2.4 Matrix back to angles

X–Y–Z and Z–Y–Z inverses use `atan2` and return the principal branch (`cos θ₂ ≥ 0` and
`sin θ₂ ≥ 0` respectively) — **choice**, as each matrix has two angle triples. The singular
branches (`r13 = ±1`, `r33 = ±1`) are detected and resolved by setting the free angle to zero.

### 2.5 Reference matrices used by the tests

| Case | Angles (deg) | `^A_B[R]` |
|---|---|---|
| space-fixed X–Y–Z | 60, 45, −75 | `[0.1830 0.6415 −0.7450; −0.6830 −0.4621 −0.5657; −0.7071 0.6124 0.3536]` |
| body-fixed X–Y–Z | 60, 45, −75 | `[0.1830 0.6830 0.7071; −0.3245 0.7209 −0.6124; −0.9280 −0.1174 0.3536]` |
| body-fixed Z–Y–Z | 30, 60, 90 | `[−0.5 −0.4330 0.75; 0.8660 −0.25 0.4330; 0 0.8660 0.5]` |
| about Z | 45 | `[1/√2 −1/√2 0; 1/√2 1/√2 0; 0 0 1]` |

Each is also checked against the symbolic closed form of its own triple, so the table is a
regression guard rather than the proof.

## 3. Position and pose

`^A p = ^A O_B + ^A_B[R] ^B p`. A pose is stored as the pair (position `^A O_B` in metres,
rotation `^A_B[R]`); 4×4 homogeneous transforms are deliberately **not** used in the core
(**choice**: the pair keeps the rotation and the translation separately testable).

## 4. Velocity and angular velocity

| Item | Convention | |
|---|---|---|
| Linear velocity | `^0V = d/dt ^0O` | standard |
| Space-fixed angular velocity | `[Ω]_R = Ṙ Rᵀ` gives `ω^s`, components along the fixed axes | standard |
| Body-fixed angular velocity | `[Ω]_L = Rᵀ Ṙ` gives `ω^b`, components along the body axes | standard |
| Skew layout | `[Ω] = [0 −ωz ωy; ωz 0 −ωx; −ωy ωx 0]`, so `[Ω]p = ω × p` | standard |
| Relation | `ω^s = R ω^b`, `[Ω]_R = R [Ω]_L Rᵀ` | standard |
| Used for integration | `Ṙ = R [ω^b ×]` | standard |
| Point acceleration | `a_p = a_O + α × r + ω × (ω × r) + 2 ω × v_rel + a_rel` | standard |

Euler-angle rate maps for the X–Y–Z and Z–Y–Z triples are implemented in
`rbsim.math.eulerRatesXYZ` and `rbsim.math.eulerRatesZYZ`, with the X–Y–Z inverse in
`rbsim.math.eulerRatesFromOmegaXYZ` (singular at `cos θ₂ = 0`, where it raises an error
rather than return a large number).

## 5. Units

SI throughout: m, kg, s, N, N·m, rad, rad/s, kg·m². Degrees exist only in the app's input
boxes and in printed reports, and are converted at that boundary.

## 6. Dynamics

### 6.1 Mass properties

| Item | Convention | |
|---|---|---|
| Inertia | `^B[I]_C = −∫[r][r] ρ dV` about C in `{B}`; products of inertia carry the minus sign, `I_xy = −∫xy ρ dV` | standard |
| Change of frame | `^A[I]_C = R ^B[I]_C Rᵀ` | standard |
| Parallel axis | `[I]_O = [I]_C − m [d]²` | standard |
| Box, edges a, b, c | `diag(m(b²+c²), m(a²+c²), m(a²+b²))/12` | standard |
| Cube | `(m a²/6) I` | standard (the box with a = b = c) |

### 6.2 Free rigid body (the die)

```
m a_C = F                                   Newton, in the inertial frame {A}
^B[I]_C ω̇^b = ^B M_C − ω^b × ^B[I]_C ω^b     Euler, about C, in the body frame {B}
Ṙ = R [ω^b ×]                               orientation kinematics
```

State, 18×1 (**choice**, see decision ADR-0008):

```
x = [ r_C (1:3, m) ; V_C (4:6, m/s) ; R(:) (7:15) ; ω^b (16:18, rad/s, in {B}) ]
```

The rotation matrix is the orientation state because it has no singularities; Euler angles
are produced as outputs only. After each integrator step the rotation block is projected
back onto SO(3) and the size of the correction is logged.

### 6.3 Planar 3R arm

Frames `{i}` at joint `O_i` with `X̂_i` along link *i*; joint angles relative and
counter-clockwise; joint axes along `+Ẑ`; gravity along `−Ŷ_0`. Each link carries a length
`l_i`, mass `m_i`, centre-of-mass offset `r_i` along `X̂_i` and inertia `I_i` about that
centre. The equation of motion is `τ = M(q) q̈ + C(q, q̇) + G(q)`; inverse dynamics is the
recursive Newton–Euler pass, forward dynamics inverts `M(q)` assembled column by column from
the same pass. State: `x = [q; q̇]`, 6×1.

### 6.4 Four-bar linkage

Ground link `l₀` from `O_L` (the origin of `{0}`) to `O_R = (l₀, 0)`; crank `l₁`, coupler
`l₂`, rocker `l₃`; crank angle `θ₁` counter-clockwise from `X̂_L`, output angle `φ₁`
counter-clockwise from `X̂_R`. The constraint is the loop closure in Freudenstein's form, and
the assembly branch (`+1` open, `−1` crossed) is an explicit parameter that is never changed
silently mid-simulation. State: `x = [θ₁; θ̇₁]`, 2×1, because the mechanism has one degree of
freedom.

### 6.5 Integration

Fixed-step classical Runge–Kutta of fourth order, with the SO(3) projection above
(**choice**, ADR-0009: a fixed step gives a uniform grid for animation and a reproducible
result). `ode45` with tight tolerances is kept as an independent reference for comparison
studies. Every run logs its step, solver, orthonormality error, energy, momentum and — for
the four-bar — the loop-closure residual.

## 7. Die face numbering

| Body face normal | +X̂ | −X̂ | +Ŷ | −Ŷ | +Ẑ | −Ẑ |
|---|---|---|---|---|---|---|
| Value | 4 | 3 | 2 | 5 | 1 | 6 |

Opposite faces sum to seven, as on a real die. The face shown uppermost is the one whose
outward normal, expressed in `{A}`, has the largest `+Ẑ_A` component (**choice**: the
simplest rule that is exact when the die rests flat).
