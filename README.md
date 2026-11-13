# 🦆 MicroDuck Digital Twin — 50 Hz Browser & Python Simulator

[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-emerald.svg)](src/)
[![WebGL / Three.js](https://img.shields.io/badge/WebGL-Three.js_50Hz-amber.svg)](web/)

An interactive **50 Hz Digital Twin, Safety Verification Suite, and WebGL 3D Simulator** for the 15-DOF bipedal **MicroDuck / XGO-Duck** robot platform (14 policy-driven Dynamixel XL330-M288-T joints + 1 independent beak/mouth actuator).

> **💡 Why I Built This (A Companion Sandbox, Not a Replacement):**  
> I ordered a physical **MicroDuck** and wanted an interactive environment while waiting for it to arrive — a place to understand its 15-DOF kinematics, brainstorm new behaviors, prototype software features, and experiment with hardware/firmware upgrades (like VL53L5CX ToF Theremin synthesis, 4-duck BLE Chorale sync, and Impact Guard tuning). **This simulator is a companion ideation & testing sandbox and is in no way meant to replace the physical MicroDuck robot!**

![MicroDuck 50 Hz Digital Twin Dashboard](docs/screenshots/microduck_3col_dashboard.png)

![MicroDuck Compact Telemetry View](docs/screenshots/microduck_digital_twin_ui.png)

> **Clean-Room Zero-Redistribution Architecture:**  
> This repository contains **only original simulator code, kinematic/safety models, and deterministic unit tests**. It does **not** redistribute upstream source code, third-party STL/CAD files, or pre-trained ONNX/WAV binaries. Instead, a one-command bootstrap script ([`scripts/fetch_upstream_assets.sh`](scripts/fetch_upstream_assets.sh)) clones the official upstream repositories directly on your machine and builds the local runtime asset bundle.

---

## ✨ Key Capabilities

1. **Articulated 15-DOF SE(3) CAD Rig & Hardware Shell Visualizer**
   - Real-time forward kinematics across all 15 robot bodies (`left/right hip_yaw, hip_roll, hip_pitch, knee, ankle`, `neck_pitch`, `head_pitch, head_yaw, head_roll`, and `mouth`).
   - Supports 5 authentic hardware shell colorways (*Cream White + Orange Beak*, *Sky Blue V5*, *Lavender Purple*, *Slate Tactical*, and *Raw CAD*).
   - Optional ankle-mounted roller-skate wheel assembly (`V5_AUDIO_VIDEO_TIRE_01.stl`) for `roller` locomotion mode.

2. **50 Hz Control Loop (`dt = 0.02 s`) & 61-D Observation Inspector**
   - Faithfully models the 61-dimensional policy observation vector (`gyro[3]`, `gravity[3]`, `joint_pos_err[14]`, `joint_vel[14]`, `prev_action[14]`, `cmd_twist[3]`, `cmd_head[4]`, `pad[2]`, `cmd_body[3]`, `pad[1]`) while excluding beak actuator index `9` (`mouth`) from closed-loop locomotion feedback.
   - Live browser **ONNX Runtime Web** policy inference (`velstand.onnx`, `pickup_detector.onnx`, `pet_detect.onnx`) with a deterministic phase-locked analytical CPG fallback when running offline.

3. **Hardware Safety & Impact Guard Failure Laboratory**
   - **Impact Guard Stress-Test:** Compare factory **Level `5`** (where an $8.2\text{ rad/s}$ ground-contact spike trips a false-positive motor stall and collapses the stance leg) against hardened **Setting `R`** ($12.0\text{ rad/s}$ threshold survival).
   - **63-D Pickup Detector & `PAUSE_POSE` Ramp:** Simulates the $630\text{ D}$ ($10 \times 63$) sliding window pickup classifier and $300\text{ ms}$ smooth leg-tuck ramp.
   - **Safety Invariants:** $500\text{ ms}$ deadman command timeout, $[-\pi, +\pi]\text{ rad}$ target clamping, non-finite (`NaN`/`Inf`) frame rejection, and $200\text{ ms}$ fall-recovery debounce (`GAIN_NORMAL` $\to$ `GAIN_LIMP`).

4. **Multi-Sensor & Behavior Emulation**
   - **VL53L5CX $8\times 8$ ToF Depth Matrix & Theremin:** Interactive hand-distance slider and $8\times 8$ status/range grid with real-time WebAudio Theremin pitch + beak opening modulation.
   - **Autonomous Brains:** `autod` obstacle avoidance, HSV `laser-chase` pursuit, `soccer` ball approach & kick FSM, 13 keyframed emotions, and 4-duck synchronized BLE Chorale (`wistful.duckscore`).

---

## 🔗 Referenced Upstream Repositories & Credits

This project builds upon and references the open hardware and software ecosystem created by **Pollen Robotics**, **Luwu Intelligence Technology (XGO)**, **MuJoCo Lab**, and **Rhoban**:

| Upstream Project | Repository / Resource | Role & How It Is Referenced | Upstream License |
| :--- | :--- | :--- | :--- |
| **Pollen Robotics — MicroDuck Software Stack** | [`pollen-robotics/microduck`](https://github.com/pollen-robotics/microduck) | Reference specification for the 15-DOF joint order, 61-D observation vector, 63-D pickup feature layout, MJCF kinematics, and emotion keyframes fetched by [`src/extract_ground_truth.py`](src/extract_ground_truth.py). | Apache-2.0 |
| **Pollen Robotics — MicroDuck RL & Policies** | [`pollen-robotics/microduck_rl`](https://github.com/pollen-robotics/microduck_rl) & [`pollen-robotics/microduck-policies`](https://huggingface.co/pollen-robotics/microduck-policies) | Pre-trained ONNX locomotion policies (`velstand`, `walk`, `stand`, `roller`) downloaded on demand into `web/assets/policies/`. | Apache-2.0 |
| **Luwu Dynamics (XGO) — XGO-Duck Hardware** | [`LuwuDynamics/xgoduck_hardware`](https://github.com/LuwuDynamics/xgoduck_hardware) | Official V2 Standard (47 STLs) and V5 Audio-Video (50 STLs) 3D-printable shell & roller-wheel CAD files cataloged locally by [`src/build_xgoduck_stl_assets.py`](src/build_xgoduck_stl_assets.py). | Personal / Educational (`LICENSING.md`) |
| **MuJoCo Lab — `mjlab`** | [`mujocolab/mjlab`](https://github.com/mujocolab/mjlab) | Underlying reinforcement learning training framework used by upstream MicroDuck policies. | Apache-2.0 |
| **Rhoban — `bam`** | [`Rhoban/bam`](https://github.com/Rhoban/bam) | Better Actuator Model (XL330 friction, armature inertia, and torque-speed Identification) referenced for joint dynamics. | MIT |

---

## 🚀 Quickstart

### 1. Clone & Install Python Dependencies

```bash
git clone https://github.com/mrojek-com/microduck-digital-twin.git
cd microduck-digital-twin
python3 -m pip install numpy trimesh pytest
```

### 2. Fetch Official Upstream Assets (Optional but Recommended)

To populate `web/assets/` with the real articulated CAD meshes, XGO-Duck STL shells, emotion keyframes, and ONNX policies directly from the official upstream repositories:

```bash
bash scripts/fetch_upstream_assets.sh
```

*(Note: Even before running `fetch_upstream_assets.sh`, the Python physics/kinematics test suite and procedural 3D fallback work out of the box.)*

### 3. Launch the Interactive WebGL Digital Twin

```bash
python3 -m http.server 8778 --bind 127.0.0.1 --directory web
```

Then open **`http://127.0.0.1:8778`** in Chrome, Firefox, or Safari.

### 4. Run the Verification Test Suite

```bash
bash run_tests.sh
```

---

## 📁 Repository Structure

```text
.
├── LICENSE                          # Apache-2.0 license + upstream attribution notice
├── README.md                        # Project documentation & upstream references
├── .gitignore                       # Excludes secrets, PII, and upstream clones/binaries
├── run_tests.sh                     # Deterministic pytest runner
├── scripts/
│   └── fetch_upstream_assets.sh     # Clones upstream repos & builds local web/assets/
├── src/
│   ├── models.py                    # 15-DOF joint definitions, actuator constants & limits
│   ├── kinematics_obs.py            # SE(3) forward kinematics, 61-D obs & 63-D pickup features
│   ├── sim_engine.py                # 50 Hz physics engine, safety guards & Impact Guard model
│   ├── extract_ground_truth.py      # Extracts CAD rig, MJCF, emotions & ONNX from upstream
│   └── build_xgoduck_stl_assets.py  # Catalogs & converts XGO-Duck V2/V5 STL hardware shells
├── tests/
│   └── test_microduck_sim.py        # 9 deterministic unit & adversarial NaN/safety tests
├── docs/
│   └── CHEATSHEET_MASTER_GUIDE.md   # Deep architectural & hardware reference guide
└── web/
    ├── index.html                   # 3-column dark-mode telemetry & WebGL UI
    ├── styles.css                   # Responsive styling & non-overlapping viewport HUD
    └── simulator.js                 # 50 Hz Three.js + ONNX Runtime Web + WebAudio engine
```

---

## 📜 License & Hardware Usage Notice

- **Simulator Source Code (`src/`, `web/`, `tests/`, `scripts/`):** Released under the **Apache License 2.0** (see [`LICENSE`](LICENSE)).
- **Upstream Software (`pollen-robotics/microduck`, `pollen-robotics/microduck_rl`):** Licensed by Pollen Robotics under the **Apache License 2.0**.
- **Upstream Hardware CAD/STL Assets (`LuwuDynamics/xgoduck_hardware`):** Fetched locally via `scripts/fetch_upstream_assets.sh` for personal and educational use in accordance with Luwu Intelligence Technology's [`LICENSING.md`](https://github.com/LuwuDynamics/xgoduck_hardware/blob/main/LICENSING.md). Commercial use of the physical hardware designs requires separate authorization from the upstream hardware maintainer.
