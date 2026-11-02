#!/usr/bin/env bash
# What this computer offers: OS, git and the MATLAB release the simulator would use.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
echo "Repository: $ROOT"
echo "--- OS ---"
lsb_release -ds 2>/dev/null || grep PRETTY_NAME /etc/os-release || uname -a
echo "--- git ---"
git --version
echo "--- MATLAB ---"
if command -v matlab >/dev/null 2>&1; then
  matlab -batch "disp(['release ' version('-release')]); rbsim_root = pwd; run('matlab/setup_path.m'); rbsim.doctor;"
else
  echo "MATLAB not found on PATH"
fi
echo "--- repository state ---"
git status --short --branch
