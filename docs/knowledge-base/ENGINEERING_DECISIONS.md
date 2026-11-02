# Engineering decisions

Numbered decisions, oldest first. The numbers are referenced from the code and never
reused, so a superseded decision keeps its number and says what replaced it.

## ADR-0001 — One core, three applications

The die, the arm and the linkage share a single set of mathematical and rigid-body
routines rather than each carrying its own copy. A sign error can therefore only be made
once, and fixing it fixes all three.

Status: accepted.

## ADR-0002 — The app computes no mechanics

The app reads its forms, calls a `simulate` function and draws what comes back. No equation
of motion lives in a callback. The consequence is that every model runs head-less, which is
what makes the test suite possible, and it is enforced by a test that greps the mechanics
packages for any reference to the app or the drawing code.

Status: accepted.

## ADR-0003 — Standard results, named in the code

Every derivation the code implements is a known result and is named where it is used —
Rodrigues' formula, the parallel-axis theorem, Freudenstein's equation, the recursive
Newton–Euler pass. Where a result has more than one common sign or ordering convention, the
choice is written down in `architecture/CONVENTIONS.md` and a test pins it. Nothing is
"obvious from the code".

Status: accepted.

## ADR-0004 — Physics before polish

A subsystem is not drawn until its numbers are checked, and the drawing is then built from
the simulated state rather than from a convenient parametrisation of the picture. The
visualisation tests assert that what is on screen equals what was computed.

Status: accepted.

## ADR-0005 — The repository is the record

Decisions, conventions and validation evidence live in `docs/`, in version control, beside
the code they govern. Anything that matters six months from now is written down here rather
than left in a notebook.

Status: accepted.

## ADR-0006 — Conventions are frozen

Frames, rotation order, angular-velocity definitions, units and state layouts are fixed in
`architecture/CONVENTIONS.md`. Changing one of them means a new decision entry, updated
tests and a changelog line — never a quiet edit, because a silent change of convention is
the hardest class of bug to find in this kind of code.

Status: accepted.

## ADR-0007 — Reference values must be independently computable

Every numeric expectation in the test suite is something a reader can reproduce: a
closed-form matrix, an analytic trajectory, a conservation law, or a finite-difference
check of the routine's own inputs. No test asserts a number merely because the code once
produced it, except for the three regression values that are labelled as such.

Status: accepted.

## ADR-0008 — The rotation matrix is the orientation state

Orientation is integrated as the 3×3 matrix with `Ṙ = R [ω^b ×]`, and the rotation block is
projected back onto SO(3) after every step. Euler angles are outputs only.

Three angles are a minimal description but have a singularity, and a tumbling die reaches
it routinely. Unit quaternions would also work and are the lighter choice for large
simulations; the matrix was preferred here because the project is meant to be read, and
every intermediate quantity in the matrix form is directly inspectable. The cost — nine
numbers and a projection step — is measured and logged rather than assumed small.

Status: accepted.

## ADR-0009 — Fixed-step Runge–Kutta as the working integrator

The classical fourth-order rule at a fixed step is the default. It produces a uniform time
grid, which the animation and the dashboards want, and a bit-identical result from run to
run, which the tests want. `ode45` is kept as an independent reference and the two are
compared. The order of accuracy is measured rather than claimed.

Status: accepted.

## ADR-0010 — The arm is planar

The three-link arm has all its joint axes parallel. That keeps the geometry readable
without losing any of the dynamics that matter — coupling, Coriolis and centripetal terms,
gravity, and a configuration-dependent inertia matrix are all present. A spatial arm would
add a frame-assignment convention (Denavit–Hartenberg or similar) whose details would
dominate the code without teaching anything further about rigid-body dynamics.

Status: accepted.

## ADR-0011 — Four-bar dynamics from free-body diagrams

The linkage dynamics are assembled from Newton's and Euler's equations for each moving
link, with the joint reactions as unknowns, rather than from a reduced one-degree-of-freedom
Lagrangian. The reduced form is shorter, but the free-body form also yields the joint
reaction forces, which are what a designer actually needs, and it generalises to other
linkages. The reduced energy equation is still derived independently and used as the check.

Status: accepted.

## ADR-0012 — The floor under the die is an extension, and is marked as one

Free flight is textbook mechanics; a die that lands is not, because contact needs a model
that classical rigid-body theory does not supply. A penalty model is used: at each corner
below the floor, a spring and damper along the normal and regularised Coulomb friction
across it. It is built only from elementary force elements, it is labelled an engineering
extension wherever it appears, and it can be switched off in the app to leave the pure
analytic model.

Status: accepted.

## ADR-0013 — Planar models use the same three-dimensional primitives

The arm and the linkage are planar, but their orientations are still rotation matrices
about Ẑ and their angular velocities are still three-vectors. One set of primitives, one
set of conventions, one set of tests.

Status: accepted.

## ADR-0014 — The app is a programmatic figure, not a binary App Designer file

The app is a MATLAB class that builds its own `uifigure`. An `.mlapp` is a binary archive:
it cannot be reviewed in a pull request, diffed, or merged. A class is plain text, its
public methods can be driven from a test without a human clicking anything, and the whole
interface is therefore covered by the suite. The app pins the light theme because the
colour set is chosen for a light surface.

Status: accepted.

## ADR-0015 — One package tree, one entry point

All MATLAB code lives under `matlab/` in a single `rbsim` package tree, one sub-package per
layer, and `setup.m` in the repository root is the only thing a new user has to run. The
package prefix makes every call site say which layer it is reaching into, which is what
keeps the layering in ADR-0002 honest.

Status: accepted.
