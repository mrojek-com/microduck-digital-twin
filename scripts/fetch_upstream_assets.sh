#!/usr/bin/env bash
# =============================================================================
# MicroDuck Digital Twin — Upstream Asset Bootstrapper
# =============================================================================
# This repository ships only clean-room simulator code, safety/kinematics
# models, and tests. It does NOT redistribute upstream source code, proprietary
# CAD/STL shells, or pre-trained ONNX/WAV binaries.
#
# Run this script after cloning to fetch the official upstream repositories
# directly from GitHub and build the local `web/assets/` runtime bundle:
#
#   1. pollen-robotics/microduck         (Apache-2.0)
#   2. LuwuDynamics/xgoduck_hardware     (Personal/Educational Hardware Repo)
#   3. pollen-robotics/microduck-policies (Hugging Face ONNX policies)
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

MICRODUCK_REPO_URL="https://github.com/pollen-robotics/microduck.git"
XGODUCK_HW_URL="https://github.com/LuwuDynamics/xgoduck_hardware.git"

echo "=================================================================="
echo "  MicroDuck Digital Twin — Fetching Official Upstream Repositories"
echo "=================================================================="

if [ ! -d "${PROJECT_ROOT}/repo/.git" ]; then
  echo "[1/4] Cloning pollen-robotics/microduck -> ./repo ..."
  git clone --depth 1 "${MICRODUCK_REPO_URL}" "${PROJECT_ROOT}/repo"
else
  echo "[1/4] Found existing ./repo (pollen-robotics/microduck)"
fi

if [ ! -d "${PROJECT_ROOT}/xgoduck_hardware/.git" ]; then
  echo "[2/4] Cloning LuwuDynamics/xgoduck_hardware -> ./xgoduck_hardware ..."
  git clone --depth 1 "${XGODUCK_HW_URL}" "${PROJECT_ROOT}/xgoduck_hardware"
else
  echo "[2/4] Found existing ./xgoduck_hardware (LuwuDynamics/xgoduck_hardware)"
fi

echo "[3/4] Extracting articulated CAD rig, MJCF kinematics, emotions & ONNX policies..."
python3 "${PROJECT_ROOT}/src/extract_ground_truth.py"

echo "[4/4] Building XGO-Duck V2/V5 STL catalog, roller tire mesh & 3D print plates..."
python3 "${PROJECT_ROOT}/src/build_xgoduck_stl_assets.py"

echo ""
echo "[✓] Upstream assets extracted into web/assets/."
echo "    Start the simulator with:"
echo "    python3 -m http.server 8778 --bind 127.0.0.1 --directory web"
