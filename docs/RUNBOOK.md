# ENR209 Rigid Body Simulator Runbook

## 1. Environment

```bash
./scripts/check_environment.sh     # prints OS, Git, MATLAB release
./scripts/verify_sources.sh        # local course sources vs SHA-256 manifest (maintainer only)
./scripts/audit_control_plane.sh
```

## 2. Test and validate

```bash
./scripts/run_tests.sh             # all levels; exits non-zero on any failure
./scripts/run_tests.sh core        # subset
```

Record results in `docs/validation/VALIDATION_MATRIX.md` (never mark NOT TESTED as PASS).

## 3. Run

```matlab
addpath app; launchRigidBodySimulator          % GUI
addpath examples; runExamples                  % headless demonstrations
```

## 4. Regenerate figures

```bash
matlab -batch "addpath('scripts'); make_figures"
```

## 5. Regenerate lecture evidence (maintainer, needs local course files)

```bash
python3 scripts/extract_source.py <deck.pptx> docs/reference/source_text/<deck>.txt
soffice --headless --convert-to pdf --outdir <scratch> docs/lecture_notes/source/*.pptx
pdftoppm -r 80 -png <scratch>/<deck>.pdf <scratch>/png/<deck>/s
```

Slide renders are derived course material: keep them outside the repository.

## 6. Development order (for changes)

requirements → lecture evidence → conventions check → implementation → tests →
physical validation → visualization → GUI → documentation → commit.
