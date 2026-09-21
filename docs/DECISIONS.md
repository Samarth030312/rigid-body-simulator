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
