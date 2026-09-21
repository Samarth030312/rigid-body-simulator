# Architecture and Engineering Decisions

## ADR-0001 — Reusable rigid-body framework

The project will use a common core for mathematical and rigid-body functionality
rather than implementing dice, 3R, and four-bar mechanics as unrelated scripts.

Status: Accepted

## ADR-0002 — GUI separation

The GUI will remain a presentation/control layer. Core mechanics will not depend
on GUI callbacks.

Status: Accepted

## ADR-0003 — Source hierarchy

Official assignment > supplied lectures > project decisions > project
documentation > general knowledge.

Status: Accepted

## ADR-0004 — Validation before visual polish

Physics and numerical correctness must precede advanced visualization and GUI polish.

Status: Accepted

## ADR-0005 — Persistent context

Project status, implementation history, decisions, and validation evidence are
stored in the repository so work can continue across sessions.

Status: Accepted

## ADR-0006 — Mechanics conventions frozen from lecture evidence (2026-09-21)

`docs/architecture/CONVENTIONS.md` is frozen. Rotation matrix `^A_B[R]` maps body
components to world components (L2-4/s8–9); body-fixed rotations post-multiply,
space-fixed rotations pre-multiply (L2-4/s21, s36); angular velocity defined from
`Ṙ R^T` (space-fixed) and `R^T Ṙ` (body-fixed) (L7-9/s5–10); Euler's equation in the body
frame at C (L16/s6); SI units; radians internally.

Status: Accepted

## ADR-0007 — Evidence from rendered original slides, not extracted text

Lecture equations are embedded images (only 4 of 291 slides contain OMML text). All
convention-relevant claims were verified against slides rendered to PNG with LibreOffice →
PDF → pdftoppm (renders are kept outside the repository; they are derived copies of course
material). Extracted text is used only as a search index.

Status: Accepted

## ADR-0008 — Rotation matrix as the orientation state

The dice orientation state is the 3×3 rotation matrix (9 numbers), integrated with
`Ṙ = R[ω^b×]` and projected back onto SO(3) after each step. Rationale: the lecture calls
the rotation matrix "ideal for analysis" (L2-4/s69) and it has no singularities; Euler
angles are singular (L2-4/s27, s31) and are therefore output quantities only; quaternions
are mentioned in the lectures (L2-4/s66–68) but not developed, so using them would add
unexplained machinery. The Euler-angle ODE of L7-9/s14–16 is still implemented and used as
an independent validation path.

Status: Accepted

## ADR-0009 — Fixed-step RK4 as primary integrator

Lectures only say "numerical integration – using Matlab" (L7-9/s14). Fixed-step classical
RK4 is chosen: simple enough to derive in a viva, 4th-order accurate, deterministic, and
gives a uniform time grid for animation and dashboards. `ode45` is used as a reference in
validation studies. Convergence order is measured (Level 4 tests).

Status: Accepted

## ADR-0010 — 3R manipulator is the lecture planar 3R

The "3R robotic manipulator" of the assignment is modelled as the planar 3R of L7-9/s49
(all joint axes parallel, out of the page), with dynamics by the recursive Newton–Euler
algorithm of L20-21/s10–11 and the 2R closed form of L20-21/s19 as a reduction check.
A spatial 3R would require frame-assignment conventions (e.g. D-H) that the lectures do not
teach.

Status: Accepted

## ADR-0011 — Four-bar dynamics by per-link Newton–Euler with constraint kinematics

The lectures give four-bar kinematics (L5-6/s20–22) and generalised forces (L10-13/s20) but
no equations of motion. Dynamics are built from lecture tools: each moving link gets
Newton's law and Euler's equation with a free-body diagram (L17-19/s9–13); COM
accelerations are written as `a = J θ̈1 + J̇ θ̇1` from the differentiated loop-closure
constraint; the 9 planar equations are solved for θ̈1 and the joint reactions.
Classified EXTENSION. Validated by energy conservation and power balance `τ θ̇1 = dE/dt`.

Status: Accepted

## ADR-0012 — Dice ground contact is an engineering extension

The assignment asks for "straightforward" equations of motion (mass, cube inertia, gravity,
initial conditions). That free-flight model is implemented first and validated
analytically. A face outcome requires the dice to land; ground contact is not taught, so it
is added as a clearly-labelled extension using only lecture force elements: a spring
(`F = KΔx`), damper (`F = cẋ`) and Coulomb friction (`|F| = μN`) at each cube vertex
(L10-13/s6–7). The GUI allows contact to be switched off, leaving the pure lecture model.

Status: Accepted

## ADR-0013 — Planar mechanics use the same 3D primitives

The 3R and four-bar are planar, but link orientations are still represented as
`^0_i[R] = R(Z, ·)` and angular velocities as 3-vectors along Ẑ, so that the same rotation
and cross-product primitives (and the same conventions) serve all three applications.

Status: Accepted

## ADR-0014 — Programmatic uifigure GUI instead of an App Designer .mlapp

The GUI is `app/RigidBodySimulatorApp.m`, a handle class that builds a `uifigure`
programmatically (base MATLAB). An `.mlapp` file is a binary archive that cannot be reviewed
or diffed in Git and is awkward to test. The class is plain text, its public methods
(`run`, `showSample`, `setField`, `getParams`, `resetForm`, `play`) are exercised headlessly
by `tests/app/test_app.m`, and callbacks only collect inputs, convert units at the boundary,
call `simulate*` and pass the result to `viz*`. The app pins the light theme because the
validated palette targets a light surface (dark-mode MATLAB made axis text unreadable).

Status: Accepted
