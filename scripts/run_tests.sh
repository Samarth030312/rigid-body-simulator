#!/usr/bin/env bash
# Run the MATLAB test suite headlessly. Optional arg: subfolder of tests/ (e.g. core).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
matlab -batch "addpath('tests'); runAllTests('${1:-}');"
