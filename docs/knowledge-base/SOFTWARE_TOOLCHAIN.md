# Software toolchain

## What is required

| Component | Version | Why |
|---|---|---|
| MATLAB | R2021b or newer | argument blocks, `uifigure`, `matlab.unittest` |
| MATLAB toolboxes | none | the mechanics, the app and the tests use base MATLAB only |

Developed and tested on MATLAB R2026a Update 5 (26.1.0.3346908) on Ubuntu 22.04. The code
is plain MATLAB with no compiled components, so Windows and macOS need nothing extra; they
are not part of the routine test runs and are marked as untested in
[`OPEN_ITEMS.md`](OPEN_ITEMS.md).

`rbsim.doctor` reports the release, whether the package is on the path and whether a
display is available; `setup` prints anything that is not a pass.

## Policy on toolboxes

A toolbox is added only with a demonstrated need, a recorded licence check and a test that
still passes without it. Nothing in the project currently justifies one: the solvers are a
dozen lines, the graphics are base handle graphics, and the test framework ships with
MATLAB. A test asserts that the mechanics packages never reach for the app or the graphics,
which also keeps them importable in a bare environment.

## Repository tooling

| Tool | Used for |
|---|---|
| `git` | history; binary MATLAB formats are marked in `.gitattributes` so they are never diffed as text |
| GitHub Actions | runs the repository hygiene check and the MATLAB suite on every push |
| Python 3 | the hygiene script only (`scripts/validation/check_repo.py`); no Python is needed to use the simulator |
