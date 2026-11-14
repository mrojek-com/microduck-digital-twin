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
#   1. pollen-robotics/microduck           (Apache-2.0)
#   2. LuwuDynamics/xgoduck_hardware       (Personal/Educational Hardware Repo)
#   3. pollen-robotics/microduck-policies  (Hugging Face ONNX policies, Apache-2.0)
#   4. pollen-robotics/microduck-simulator (Hugging Face Space MJLab high-res GLB,
#                                           inline roller STLs & roller ONNX, Apache-2.0)
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

MICRODUCK_REPO_URL="https://github.com/pollen-robotics/microduck.git"
XGODUCK_HW_URL="https://github.com/LuwuDynamics/xgoduck_hardware.git"
HF_SIM_BASE="https://huggingface.co/spaces/pollen-robotics/microduck-simulator/resolve/main/app/public"

echo "=================================================================="
echo "  MicroDuck Digital Twin — Fetching Official Upstream Repositories"
echo "=================================================================="

if [ ! -d "${PROJECT_ROOT}/repo/.git" ]; then
  echo "[1/5] Cloning pollen-robotics/microduck -> ./repo ..."
  git clone --depth 1 "${MICRODUCK_REPO_URL}" "${PROJECT_ROOT}/repo"
else
  echo "[1/5] Found existing ./repo (pollen-robotics/microduck)"
fi

if [ ! -d "${PROJECT_ROOT}/xgoduck_hardware/.git" ]; then
  echo "[2/5] Cloning LuwuDynamics/xgoduck_hardware -> ./xgoduck_hardware ..."
  git clone --depth 1 "${XGODUCK_HW_URL}" "${PROJECT_ROOT}/xgoduck_hardware"
else
  echo "[2/5] Found existing ./xgoduck_hardware (LuwuDynamics/xgoduck_hardware)"
fi

echo "[3/5] Fetching MJLab high-res CAD GLB, inline roller STLs & roller ONNX (Apache-2.0)..."
mkdir -p "${PROJECT_ROOT}/web/assets/mjlab/meshes" "${PROJECT_ROOT}/web/assets/policies"
for asset in \
  "robot/mjlab/microduck.glb:web/assets/mjlab/microduck.glb" \
  "robot/mjlab/kinematics.json:web/assets/mjlab/kinematics.json" \
  "robot/mjlab/kinematics_rollers.json:web/assets/mjlab/kinematics_rollers.json" \
  "robot/mjlab/meshes/ankle_l_v1.stl:web/assets/mjlab/meshes/ankle_l_v1.stl" \
  "robot/mjlab/meshes/ankle_r_v1.stl:web/assets/mjlab/meshes/ankle_r_v1.stl" \
  "robot/mjlab/meshes/roller_blade.stl:web/assets/mjlab/meshes/roller_blade.stl" \
  "robot/mjlab/meshes/rim.stl:web/assets/mjlab/meshes/rim.stl" \
  "robot/mjlab/meshes/tire.stl:web/assets/mjlab/meshes/tire.stl" \
  "policies/BEST_roller.onnx:web/assets/policies/BEST_roller.onnx" \
  "policies/BEST_roller_crouch.onnx:web/assets/policies/BEST_roller_crouch.onnx"
do
  src="${asset%%:*}"
  dst="${PROJECT_ROOT}/${asset##*:}"
  if [ ! -f "${dst}" ]; then
    curl -fsSL "${HF_SIM_BASE}/${src}" -o "${dst}"
  fi
done

echo "[4/5] Extracting articulated CAD rig, MJCF kinematics, emotions & ONNX policies..."
python3 "${PROJECT_ROOT}/src/extract_ground_truth.py"

echo "[5/5] Building XGO-Duck V2/V5 STL catalog, roller tire mesh & 3D print plates..."
python3 "${PROJECT_ROOT}/src/build_xgoduck_stl_assets.py"

echo ""
echo "[✓] Upstream assets extracted into web/assets/."
echo "    Start the simulator with:"
echo "    python3 -m http.server 8778 --bind 127.0.0.1 --directory web"
