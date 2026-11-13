#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if command -v pytest >/dev/null 2>&1; then
  pytest -v tests/test_microduck_sim.py
else
  uv run --with pytest pytest -v tests/test_microduck_sim.py
fi
