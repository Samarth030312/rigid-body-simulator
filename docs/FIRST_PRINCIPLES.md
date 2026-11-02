# First principles

What the simulator actually computes, in the order the code computes it, and where each
step is checked. Symbols and sign conventions are fixed in
[`architecture/CONVENTIONS.md`](architecture/CONVENTIONS.md); the derivations are in the
works listed in [`knowledge-base/REFERENCES.md`](knowledge-base/REFERENCES.md).

## 1. Orientation is a rotation matrix

A rigid body needs six numbers: three for where it is and three for how it is turned. The
three orientation numbers are stored as a 3×3 rotation matrix `^A_B[R]` whose columns are
the body axes written in world coordinates, so that `^A r = ^A_B[R] ^B r` converts body
components into world components. The matrix has nine entries and six constraints
(`RᵀR = I`, `det R = +1`), which is why it never runs into the singularity that any
three-angle description has.

Three angles are still the natural thing to *type*, so the code builds a rotation from any
triple of axes, in either of the two senses that exist:

- **about axes carried with the body** — each new rotation acts in the frame the previous
  ones produced, so the matrices post-multiply;
- **about axes fixed in space** — each rotation acts in the original frame, so they
  pre-multiply.

The two give different results from the same numbers, and the same result when one sequence
is reversed. `test_rotations` checks both against their closed forms, checks that a
body-fixed X–Y–Z triple equals the space-fixed Z–Y–X triple of the same angles, and
reproduces the space-fixed construction a second way, by rotating about each fixed axis
expressed in the current body frame.

## 2. Angular velocity comes from the derivative of that matrix

Angular velocity is not the derivative of three angles. Differentiating `RᵀR = I` shows that
`Ṙ Rᵀ` and `Rᵀ Ṙ` are both skew-symmetric, and each skew matrix is a vector in disguise:

```
[Ω]_R = Ṙ Rᵀ  ->  ω^s   components along the fixed axes
[Ω]_L = Rᵀ Ṙ  ->  ω^b   components along the body axes,  ω^s = R ω^b
```

Running this backwards, `Ṙ = R [ω^b ×]`, is how orientation is advanced in time. The tests
compute `Ṙ` by central differences of an analytically known orientation history and compare
the recovered `ω^s` and `ω^b` with the closed-form rate maps for X–Y–Z and Z–Y–Z angles.

Velocity and acceleration of a point of the body follow by differentiating
`^A p = ^A O_B + R ^B p` once and twice, which produces the familiar five terms —
origin, tangential, centripetal, Coriolis and relative. Each is checked against finite
differences of the position itself.

## 3. Mass and inertia

Mass is one number; the distribution of that mass is a symmetric positive-definite 3×3
matrix, `^B[I]_C = −∫[r][r] ρ dV`, taken about the centre of mass and expressed in the body
frame, where it is constant. Changing frame is a similarity transform, `R [I] Rᵀ`; moving
the reference point is the parallel-axis theorem. `test_inertia` does not trust the
formulas: it discretises the defining integral over a grid of mass elements and compares,
which also pins down the sign convention of the products of inertia.

A cube is the special case `a = b = c` of a rectangular block, and its inertia is isotropic,
`(m a²/6) I`. That has a consequence the simulator relies on for a test: for an isotropic
body `ω × [I] ω = 0`, so a tumbling cube with no applied moment spins about a fixed axis at
constant rate, and the exact orientation at any later time is known in closed form.

## 4. Newton and Euler

Two equations, in the two frames that make each of them simple:

```
m a_C = F                                 in the inertial frame
[I]_C ω̇^b = M_C − ω^b × [I]_C ω^b          about the centre of mass, in the body frame
```

The cross term is the one that makes a thrown object tumble rather than spin tidily. The
body frame is used for the second equation because `[I]_C` is constant there; the price is
the cross term, and the sign of it is checked independently by confirming that the rate of
change of angular momentum in world coordinates equals the applied moment.

Three checks pin the dynamics down: a body with no forces keeps its energy and its angular
momentum; a symmetric body with no moment reproduces the closed-form precession of its
angular-velocity vector about the symmetry axis; and a body under gravity alone follows the
parabola exactly.

## 5. Advancing time

The state is integrated with the classical fourth-order Runge–Kutta rule at a fixed step.
Two things are measured rather than assumed: the order of accuracy, by halving the step and
watching the error fall by the expected factor, and the agreement with `ode45` at tight
tolerance over the same interval.

Numerical integration slowly pushes the rotation block out of SO(3). After every step it is
projected back to the nearest proper rotation and the size of that correction is recorded,
so drift is visible instead of silent.

## 6. From one body to a chain

A serial arm is the same mechanics applied link by link. Velocities and accelerations
propagate outwards from the fixed base to the tool, each joint adding its own rate about its
own axis; forces and moments propagate inwards from the tool back to the base, each link
contributing the inertial force and moment the outward pass computed. The joint torque is
the component of the joint moment along the joint axis. Gravity enters as an acceleration of
the base, which is the standard trick that avoids a separate gravity term.

That one routine answers both questions: given the motion, what torques are needed, and —
by calling it with zero velocity and a unit acceleration at each joint in turn — what the
joint-space inertia matrix is, which then gives the motion from the torques.

The arm's energy is computed independently, link by link from centre-of-mass velocities, and
never from the inertia matrix, so the two can be compared. For a two-link special case the
torques are compared against the standard closed-form planar equations.

## 7. From a chain to a loop

A four-bar linkage is not free: the four links must close on themselves. Writing that
closure as vectors and eliminating the coupler gives one scalar constraint between the crank
angle and the rocker angle, which is solved in closed form by intersecting two circles. Two
intersections exist — the two ways the linkage can be assembled — and which one is in use is
an explicit parameter rather than whatever the solver happened to return.

Differentiating the closure once and twice gives the angular velocities and accelerations as
linear systems. The determinant of those systems vanishes when the coupler and the rocker
line up, a toggle position where the velocities are genuinely undefined; the code detects it
and says so rather than returning a huge number.

Dynamics then comes from free-body diagrams: each link gets Newton's and Euler's equations,
with the unknown joint reactions appearing in both the link they act on and its neighbour.
Because every centre-of-mass acceleration is linear in the crank acceleration, the result is
nine linear equations in nine unknowns — eight reaction components and either the crank
acceleration or the torque needed to produce it.

Nothing in that derivation is taken on trust. The one-degree-of-freedom energy equation is
formed separately, from the effective inertia and the potential, and must give the same
crank acceleration; the static torque must equal the gradient of the potential; and the
ground reactions must carry the weight of the whole mechanism.

## 8. What the die does with all this

The die is the free rigid body of sections 1–5 plus a floor. Contact is deliberately the
simplest thing that can hold a cube up: each of the eight corners, when it is below the
floor, gets a spring and a damper along the normal and regularised Coulomb friction across
it. That is an engineering extension rather than textbook mechanics, and it is marked as one
throughout; with contact switched off the die is the pure free-flight model and reproduces
the analytic parabola and rotation exactly.

The face that ends up uppermost is read from the simulated orientation at the end — the
outward normal with the largest vertical component. It is not chosen, biased or
post-processed, which is why different throws give different faces and the same throw always
gives the same one.
