# MicroDuck Operator & Developer Master Cheatsheet

- **Compiled By:** `gemma4:26b` from `repo/docs/robot/cheatsheet.md` & `cheatsheet-dev.md`
- **Generated:** 2026-11-13 15:28:32 UTC

---

# 🤖 Robot Master Cheatsheet: Operator & Developer

## 🛠 Core Command Reference
*Note: Read-only commands require no privilege. Commands that **change** state require `sudo`.*

| Command | Purpose | Key Notes |
| :--- | :--- | :--- |
| `robotctl version` | **Sanity Check** | Compares *running* vs *installed* daemons. Run this first to detect "ghost" code after updates. |
| `robotctl health` | **System Status** | Returns hardware/software health. Exits non-zero if unhealthy. Use `--json` for support bundles. |
| `robotctl monitor` | **Live Telemetry** | Real-time loop visualization (Joints, IMU, Battery, Loop Rate). |
| `robotctl update apply` | **Update System** | Installs software. **Warning:** `apply daemon` without `--ref` installs *stable*, which may downgrade a dev board. |

---

## 👁 Live Monitoring (`robotctl monitor`)
The monitor provides a high-signal stream of the robot's internal state.

*   **The Loop:** Displays commanded vs. actual joint positions, IMU gravity vector `[0, 0, -1]`, and achieved loop rate.
*   **Safety Clamps:** If a command differs from the applied state, the monitor names the reason (e.g., `deadman — no intent arrived recently`).
*   **The Header (Condition Row):**
    *   **Battery:** Displayed in Volts and % (Countdown to `BATTERY_EMPTY_V`). Yellow at 30%, Red at 15%.
    *   **Thermal:** Hottest servo and board temperature.
    *   **Errors:** Reports `degraded` (e.g., motor bus failure) or `unhealthy` (e.g., low loop frequency).
    *   **Stale Reads:** Watch for `orientation frozen`—indicates a board that has stopped fusing but is still answering the bus.
*   **The Footer (Policy):** Displays the active `.onnx` policy and network configuration.

---

## ⚙️ Configuration & Development

### Dev Board Setup
To enable development features on a board:
```bash
# Enable dev daemon mode
sudo robotctl configure --allow-dev-daemon on

# Revert to standard production mode
sudo robotctl configure --allow-dev-daemon off
```

### Software Updates & Branch Management
Use these to target specific CI builds or branches.

| Goal | Command |
| :--- | :--- |
| **Install specific branch** | `sudo robotctl update apply --ref <branch> daemon` |
| **Install Mainline** | `sudo robotctl update apply --ref main daemon` |
| **Pin exact version** | `sudo robotctl update apply --version <version> daemon` |
| **Install latest Stable** | `sudo robotctl update apply daemon` |

> [!CAUTION]
> **The Downgrade Trap:** On a dev board, `apply daemon` (without `--ref`) installs the latest **stable** release. If your dev branch is ahead of stable, this will perform a downgrade.

---

## 🚀 Advanced Operations (Quick Reference)

### Policies, Skills & Input
*   **Load Policy:** `robotctl load <policy_name>` (Loads `.onnx` models).
*   **Skill Management:** Use `add` to expand capabilities and `pad bind` to map inputs.
*   **Input Modes:**
    *   **Gamepad:** Standard control via sticks/buttons.
    *   **D-Pad Modes:** Specialized directional input modes.
    *   **Audio/Special:** Control via `Chorale` or `Theremin` interfaces.
    *   **Specialty Modes:** `Laser`, `Soccer`, and `autod` (Automated/Autonomous modes).

### Power & Hardware
*   **Auto-Adoption:** The system automatically adopts new Power and Servo configurations upon detection.
*   **Emergency Power:** `robotd` will automatically sit the robot down and cut power when `BATTERY_EMPTY_V` is reached.

---

## 🖨 XGO-Duck 97-STL Printable Hardware Kit (`LuwuDynamics/xgoduck_hardware`)

| Kit Variant | STL Count | Source `.3mf` | Key Parts & Hardware |
| :--- | :---: | :--- | :--- |
| `V2_STANDARD_` | 47 STLs | `XGO_duckV2.3mf` | Standard biped shells + optional 10-part roller skate kit + V1/V1.1 backpacks |
| `V5_AUDIO_VIDEO_` | 50 STLs | `xgoduckV5.3mf` | Adds `FACE`, `FACE_CAMERA`, `FACE_LIGHT`, `FACE_TOF`, audio/video head shells |
| Flexible `TPU` Parts | 3–4 STLs | Same `.3mf` | `MOUTH_UP_TPU`, `MOUTH_BOTTOM_TPU`, `FOOT_BOTTOM_TPU_01..02` — print in flexible TPU |
| Roller Skates | 10 STLs | Included in V2 & V5 | `2x_tire_support_01..02`, `4x TIRE_01..04`, `4x_tire_axis_01..04` + `8× 10×15×3mm` bearings |

### Critical Print & Assembly Invariants
1. **Single-Instance Rule:** Never multiply STL files containing `2x` or `4x` in the filename — each `.stl` is already exported once per physical instance (`mm` scale, 100%).
2. **Enclosure Choice:** Match `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1.stl` vs `..._FOR_V1P1.stl` to your Luwu expansion PCB revision (`ArduinoUnoQV1.1` recommended).
3. **Feetech 1910 Servo IDs & Zeroing:** Configure one servo at a time: IDs `10–14` (Left Leg), `20–24` (Right Leg), `30–34` (Neck/Head, `34` = Mouth), centered at raw `2047`. Retain secondary shafts ONLY on `30` and `31`. Back up `data/zero_pos.json`.
