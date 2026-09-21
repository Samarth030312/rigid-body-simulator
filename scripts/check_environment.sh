#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
echo "=== ENR209 ENVIRONMENT ==="
echo "Project: $ROOT"
echo "--- OS ---"
lsb_release -ds 2>/dev/null || grep PRETTY_NAME /etc/os-release
echo "--- Git ---"
git --version
echo "--- MATLAB ---"
if command -v matlab >/dev/null 2>&1; then echo "MATLAB: $(command -v matlab)"; timeout 30s matlab -batch "fprintf('Release: %s\\n',version('-release')); fprintf('Version: %s\\n',version);"; else echo "MATLAB: NOT FOUND ON PATH"; fi
echo "--- Git State ---"
git status --short --branch
