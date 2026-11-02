# Third-party components

Nothing is vendored, bundled or redistributed in this repository. Every file here is
original work, licensed under the MIT terms in [LICENSE](LICENSE).

| Component | Role | Obtained from |
|---|---|---|
| MATLAB (MathWorks) | the language and the base library the simulator runs on | the user's own installation; not redistributed |
| `matlab-actions/setup-matlab`, `matlab-actions/run-command` | install and drive MATLAB in CI | GitHub Actions, at the versions pinned in `.github/workflows/ci.yml` |
| `actions/checkout` | checkout in CI | GitHub Actions |

No MATLAB toolbox is required. The simulator uses base MATLAB only: handle graphics for the
app and the plots, `matlab.unittest` for the suite, and `ode45` for the reference solution
the fixed-step integrator is compared against.

The mechanics implemented here are standard results of rigid-body dynamics. The works where
they are derived are listed in
[docs/knowledge-base/REFERENCES.md](docs/knowledge-base/REFERENCES.md); no text, figure or
data from any of them is reproduced in this repository.
