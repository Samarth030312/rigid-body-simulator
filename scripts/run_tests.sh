#!/usr/bin/env bash
# Run the MATLAB test suite head-lessly. Exits non-zero on any failure.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
matlab -batch "setup test"
