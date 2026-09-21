# System Architecture

## Layer 1 — Mathematical Foundations

Rotation matrices, vector operations, transformations, frame utilities,
geometry primitives.

## Layer 2 — Kinematics

Rigid-body state propagation and mechanism kinematics.

## Layer 3 — Dynamics

Newton-Euler equations and mechanism dynamics.

## Layer 4 — Numerical Integration

Time integration and solver infrastructure.

## Layer 5 — Applications

Dice, 3R manipulator, four-bar mechanism.

## Layer 6 — Visualization

3D geometry, coordinate frames, trajectories, plots, dashboards.

## Layer 7 — GUI

Interactive MATLAB application.

## Dependency direction

GUI
  ↓
Visualization
  ↓
Applications
  ↓
Dynamics / Kinematics
  ↓
Mathematical Foundations

Lower layers must not depend on higher layers.
