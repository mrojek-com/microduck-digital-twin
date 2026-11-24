#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> Running Python pytest suite..."
if command -v pytest >/dev/null 2>&1; then
  pytest -v tests/
else
  uv run --with pytest pytest -v tests/
fi

if command -v node >/dev/null 2>&1; then
  echo "==> Checking JS syntax & running Node kinematics regression tests..."
  node --check web/simulator.js
  node --test tests/js/*.test.js
else
  echo "WARNING: node not found; skipping JS kinematics tests."
fi
