# XGO-Duck (`LuwuDynamics/xgoduck_hardware`) 97-STL & Hardware Research Report

- **Generated:** 2026-11-13 15:36 UTC
- **Hardware Repo Cloned:** `apps/MicroDuck/xgoduck_hardware` (`V2_STANDARD_`: 47 STLs, `V5_AUDIO_VIDEO_`: 50 STLs)
- **Local LLM Council:** Sequential Metal inference (`gemma4:26b`, `mistral-small:24b`, `deepseek-r1:14b`, `qwen3:30b-a3b`) with `keep_alive: 0`

---

## 1. CAD Topology & 97-STL Architecture — `gemma4:26b`

### 1. Structural Delta: V2 vs. V5
The repository exhibits a clear evolutionary jump from the V2 Standard to the V5 Audio/Video architecture:

*   **V2_STANDARD (47 STLs):** A baseline configuration focused on the core mechanical structure.
*   **V5_AUDIO_VIDEO (50 STLs):** An expanded kit featuring integrated sensor housings. The primary structural delta is the modular head/face assembly, which includes dedicated cutouts and components for:
    *   **Camera:** `V5_AUDIO_VIDEO_FACE_CAMERA.stl`
    *   **Light:** `V5_AUDIO_VIDEO_FACE_LIGHT.stl`
    *   **ToF (Time-of-Flight) Sensor:** `V5_AUDIO_VIDEO_FACE_TOF.stl`
*   **Backpack Enclosures:** The V5 kit provides compatibility for legacy V1/V1.1 boards via two distinct enclosure options:
    *   `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1.stl`
    *   `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1P1.stl`

### 2. Material Specification & Slicer Organization
The hardware utilizes a hybrid material strategy to balance structural rigidity with impact absorption:

*   **Rigid Shells (PLA/PETG):** Used for primary structural components including the `BODY_SHELL_LEFT/RIGHT`, `BATTERY` housing, and `HEAD_BOTTOM`.
*   **Flexible Parts (TPU):** Critical for contact points and organic shapes to provide grip and damping:
    *   `MOUTH_UP_TPU` (Flexible beak pieces)
    *   `MOUTH_BOTTOM_TPU`
    *   `FOOT_BOTTOM_TPU` (e.g., `V5_AUDIO_VIDEO_FOOT_BOTTOM_TPU_01/02`)
*   **Bambu Studio Workflow:** Parts are exported from a pre-arranged **9-plate 3MF project**. Users are instructed to import STLs at 100% scale, noting that numbered files (e.g., `_01`, `_02`) represent individual instances to prevent accidental duplication of parts labeled with `2x` or `4x` prefixes.

### 3. Roller Skate Add-on & Bearing Systems
The optional Roller Skate kit is integrated into the STL counts of both versions and consists of a 10-part assembly:

*   **Component Breakdown:**
    *   **2x** Tire Supports (`V5_AUDIO_VIDEO_2x_tire_support_01/02`)
    *   **4x** Tires (`4x_TIRE`)
    *   **4x** Tire Axes (`V5_AUDIO_VIDEO_4x_tire_axis_01` through `04`)
*   **Hardware Requirements:**
    *   **Bearings:** Utilizes a dual-bearing system requiring **8x** total bearings (specifically **10x15x3 mm**).
    *   **Fasteners:** Requires **4x** M2 × 6 mm countersunk, flat-end self-tapping screws.

---

## 2. MicroDuck Alpha (`pollen-robotics`) vs XGO-Duck (`LuwuDynamics`) — `mistral-small:24b`

```markdown
# Technical Comparison: Pollen Robotics MicroDuck Alpha vs. Luwu Dynamics XGO-Duck

## Compute-Split Difference

| Feature                | MicroDuck Alpha (Pollen Robotics) | XGO-Duck (Luwu Dynamics) |
|------------------------|-----------------------------------|----------------------------|
| Compute Architecture   | `robotd` 8-daemon Rust IPC       | Arduino UNO Q Linux + STM32 Bridge |
| Communication Protocol | Rust IPC                          | Linux to STM32 Bridge     |

## Battery/Power Delivery

| Feature                | MicroDuck Alpha (Pollen Robotics) | XGO-Duck (Luwu Dynamics) |
|------------------------|-----------------------------------|----------------------------|
| Battery Type           | Not specified                    | 18650 battery pack, 8.4 V  |
| Battery Capacity       | Not specified                    | > 3,500 mAh                |
| Discharge Rating       | Not specified                    | > 3C                       |
| Charger                | Not specified                    | 8.4 V Li-ion battery charger with 5.5 × 2.1 mm DC barrel plug |

## Fastener/Bearing BOM

| Part                    | MicroDuck Alpha (Pollen Robotics) | XGO-Duck (Luwu Dynamics) |
|-------------------------|-----------------------------------|----------------------------|
| M2 × 6 countersunk screw | Not specified                    | 204 (200+4 for roller)     |
| M2.3 × 8 flat-top screw  | Not specified                    | 6                          |
| M3 × 16 round-head screw | Not specified                    | 4                          |
| Bearing, 10 × 15 × 3 mm | Not specified                    | 10 (2+8 for roller)        |
| Bearing, 16 × 22 × 4 mm | Not specified                    | 11                         |

## PCBA V1.1 Specs

| Feature                | MicroDuck Alpha (Pollen Robotics) | XGO-Duck (Luwu Dynamics) |
|------------------------|-----------------------------------|----------------------------|
| IMU                    | BNO085/BMI088                    | QMI8658A 6-axis IMU        |
| ToF Sensor             | VL53L8CX 8x8 ToF                 | Not specified              |
| Servo Interface        | 15x Dynamixel XL330-M288-T        | 15x Feetech HLS1910        |
| Expansion Board        | Not specified                    | Luwu V1.1 PCBA             |
| Servo IDs              | 11-15/20-24/30-34                | 10-14/20-24/30-34          |
| Center Value           | Not specified                    | 2047                       |
| Power Supply           | Not specified                    | 8.4V 2S 18650 >3500mAh >3C  |

## Electronics and Control

| Feature                | MicroDuck Alpha (Pollen Robotics) | XGO-Duck (Luwu Dynamics) |
|------------------------|-----------------------------------|----------------------------|
| Microcontroller        | RK3566                           | Arduino UNO Q              |
| Servo Type             | Dynamixel XL330-M288-T            | Feetech HLS1910            |
| IMU                    | BNO085/BMI088                    | QMI8658A 6-axis IMU        |
| ToF Sensor             | VL53L8CX 8x8 ToF                 | Not specified              |
| Expansion Board        | Not specified                    | Luwu V1.1 PCBA             |
| Servo IDs              | 11-15/20-24/30-34                | 10-14/20-24/30-34          |
| Center Value           | Not specified                    | 2047                       |
| Power Supply           | Not specified                    | 8.4V 2S 18650 >3500mAh >3C  |
```

This comparison highlights the key differences in compute architecture, battery specifications, fastener and bearing BOM, and PCBA specifications between the Pollen Robotics MicroDuck Alpha and the Luwu Dynamics XGO-Duck.

---

## 3. Sim-to-Real & Mechanical Failure Audit — `deepseek-r1:14b`

Here is the audit of critical failure modes for the XGO-Duck robot, organized into numbered forensic risk items with root causes and mitigations:

1. **Blindly Dropping ONNX Policies Without Actuator Consideration**
   - **Root Cause**: The XGO-Duck uses Feetech 1910 actuators, which differ from Rhoban BAM in friction, armature, and latency. Upstream MicroDuck policies, trained on Rhoban actuators, may not directly apply.
   - **Mitigation**: Adapt policies to XGO-Duck actuators and ensure `data/zero_pos.json` calibration for proper alignment.

2. **Neck/Head Assembly Mechanical Failure**
   - **Root Cause**: Incorrect removal of secondary shafts (IDs 30-31) or pinched cables due to enclosed links and adhesive steps can cause mechanical issues.
   - **Mitigation**: Follow assembly instructions precisely, use retaining compounds, and ensure proper cable routing and retention.

3. **Slicing/Tolerance Hazards**
   - **Root Cause**: Double-printing or single-instance STL issues can lead to fitment problems, while unspecified material hardness affects durability.
   - **Mitigation**: Validate print settings, specify Shore hardness, and ensure material suitability.

4. **Electrical Hazards**
   - **Root Cause**: Reverse polarity and sagging under load can damage components.
   - **Mitigation**: Check electrical connections, use proper power management, and avoid overloading.

5. **Torque-Off Calibration Issues**
   - **Root Cause**: Incorrect torque settings can lead to structural collapse.
   - **Mitigation**: Follow torque guidelines, double-check during assembly, and ensure proper calibration.

Each item addresses potential failure points with clear mitigations to ensure the XGO-Duck's reliability and performance.

---

## 4. Hybrid MicroDuck × XGO-Duck STL & Hardware Mods — `qwen3:30b-a3b`

We are combining the `pollen-robotics/microduck` software (with specific features: ToF Theremin, BLE Chorale, Pickup Detector, Setting R Impact Guard) with `LuwuDynamics/xgoduck_hardware` (97 STLs, Arduino UNO Q, V5 Audio/Video faceplate, Roller Skates).

We are to propose 8 concrete, buildable custom modifications. For each, we must specify:
- Exact STL/software filenames touched
- CAD/wiring modification
- User capability unlocked

We are to output ONLY a numbered Markdown list (1 through 8) concisely.

Given the hardware and software, we'll focus on integrating the software features with the physical hardware (V5 Audio/Video faceplate, Roller Skates, etc.) by modifying specific STLs and wiring.

Important: The V5_AUDIO_VIDEO STLs are the ones we can modify (as they are the ones with the audio/video faceplate). We have a list of sample STLs.

We are to use the provided STLs and the software features.

Let's plan 8 proposals:

1. **ToF Theremin Integration**: Use the ToF sensor (Time-of-Flight) for a theremin-like instrument. We'll need to mount a ToF sensor (like VL53L0X) and modify the head/faceplate to have a hole for the sensor. The software feature "ToF Theremin" will map distance to pitch.

   - STLs: Modify `V5_AUDIO_VIDEO_EYE.stl` (to add a hole for the sensor) and possibly `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1.stl` (to route wires). But note: the eye STL is the eye, so we can modify the eye to have a small hole for the sensor.
   - Software: `pollen-robotics/microduck`'s ToF Theremin feature.
   - Modification: Add a small hole (e.g., 5mm diameter) in the eye STL at the front, and route the sensor wires through the enclosure. We'll need to adjust the CAD to have a mounting point for the sensor.
   - Capability: User can play a theremin by moving hands in front of the robot's eye.

2. **BLE Chorale**: Use BLE (Bluetooth Low Energy) to connect multiple robots and play a harmonized melody. We'll need to add a BLE module (like HC-05) and modify the body to have a port for the module.

   - STLs: Modify `V5_AUDIO_VIDEO_BODY_SHELL_LEFT.stl` or `V5_AUDIO_VIDEO_BODY_SHELL_RIGHT.stl` to have a small cavity for the BLE module (or use the existing battery compartment? But we have a battery STL). Alternatively, we can use the `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1.stl` to add a port.
   - Software: `pollen-robotics/microduck`'s BLE Chorale feature.
   - Modification: Add a small cavity (e.g., 10x10x5mm) in the body shell for the BLE module, and add a small hole for the antenna. Also, wire the BLE module to the Arduino UNO Q.
   - Capability: User can connect multiple MicroDucks to form a BLE network and play a coordinated melody.

3. **Pickup Detector**: A feature that detects when the robot is picked up (via accelerometer) and triggers a sound. We'll need to add an accelerometer (like MPU6050) and modify the body to have a mounting point.

   - STLs: Modify `V5_AUDIO_VIDEO_BODY.stl` (the main body) to have a small recess for the accelerometer.
   - Software: `pollen-robotics/microduck`'s Pickup Detector feature.
   - Modification: Add a small recess (e.g., 5x5x3mm) in the body STL to mount the accelerometer. Wire the accelerometer to the Arduino.
   - Capability: When the robot is picked up, it plays a unique sound (e.g., a chime).

4. **Setting R Impact Guard**: This feature likely uses an impact sensor (like a piezo) to detect impacts and trigger a response. We'll add a piezo sensor to the front (to detect bumps) and modify the front STL.

   - STLs: Modify `V5_AUDIO_VIDEO_BODY_SHELL_LEFT.stl` (or the front part) to have a hole for the piezo. The front of the robot is the beak, so we can use the `V5_AUDIO_VIDEO_EYE.stl` or a front part? But note: the eye is on the head. Alternatively, the `V5_AUDIO_VIDEO_BODY_SHELL_LEFT` is the left shell, so we can add a hole on the front of the shell.
   - Software: `pollen-robotics/microduck`'s Setting R Impact Guard feature.
   - Modification: Add a small hole (2mm diameter) in the front of the body shell for the piezo sensor, and wire it to the Arduino.
   - Capability: When the robot bumps into something, it plays a sound and/or changes behavior (e.g., stops moving).

5. **Roller Skates with Audio Feedback**: The roller skates (V5_AUDIO_VIDEO_2x_tire_support_01.stl, etc.) can be modified to have an audio feedback system (e.g., when the wheels rotate, it plays a sound). We'll add a hall effect sensor (for wheel rotation) and a small speaker.

   - STLs: Modify the tire support STLs (e.g., `V5_AUDIO_VIDEO_2x_tire_support_01.stl`) to have a small cavity for the hall sensor and a hole for the speaker wire.
   - Software: We'll need to extend the software to read the hall sensor and trigger audio (using the existing audio system). The software feature might be a new one, but we can use the existing audio system.
   - Modification: Add a small cavity (e.g., 3x3x2mm) in the tire support for the hall sensor. Also, add a small hole for the speaker wire to go to the body.
   - Capability: As the robot moves on roller skates, it plays a rhythmic sound that corresponds to the wheel rotation.

6. **V5 Audio/Video Faceplate with Real-time Video Processing**: The V5 faceplate has a camera? We don't have a camera in the provided hardware, but the faceplate might be designed for a camera. We'll assume we can add a camera module (like OV7670) and use the faceplate for video processing (e.g., face tracking for the theremin).

   - STLs: Modify `V5_AUDIO_VIDEO_EYE.stl` to have a hole for the camera lens (if not already present) and add a cavity for the camera module.
   - Software: We'll use the existing `pollen-robotics/microduck` software and extend it to use the camera for face tracking (to control the theremin). But note: the software features listed don't include face tracking, so we are adding a new capability. However, the problem says "combining" the given software features. So we might not be adding a new feature but using the existing ones in a new way? But the problem says "combining" the given software features (ToF Theremin, BLE Chorale, etc.) with the hardware. So we can use the ToF Theremin and also use the camera to enhance it? But the camera isn't in the software features. So let's stick to the given features and use the camera for a new purpose? Actually, the problem says "combining" the given software features, so we can use the camera to trigger one of the existing features? But the problem doesn't specify. Alternatively, we can use the camera for a new feature, but the problem says "combining" the given software features. So we'll avoid adding a new feature and instead use the camera to control an existing feature (like the ToF Theremin) but that might be overcomplicating.

   Let's reinterpret: The V5 faceplate is for audio/video, so we can use it for video input to control the audio (e.g., if a face is detected, play a specific sound). But the software features don't include video processing. So we'll have to extend the software? But the problem says "combining" the given software features. So we'll assume that the software can be extended to use the camera as an input for the existing features.

   However, to be safe, we'll propose a modification that uses the existing software features and the hardware. We'll use the camera to trigger the BLE Chorale (e.g., when a face is detected, the robot connects to a nearby robot). But note: the BLE Chorale is for multiple robots, so we can have a face detection trigger the BLE connection.

   But the problem says "buildable", so we'll focus on the hardware modification.

   - STLs: Modify `V5_AUDIO_VIDEO_EYE.stl` to have a hole for the camera lens (if not already present) and a cavity for the camera module.
   - Software: We'll extend the BLE Chorale feature to trigger on face detection (via the camera). But note: the software feature is called "BLE Chorale", so we are using it in a new way. However, the problem says "combining" the given software features, so we are using the BLE Chorale feature and adding a new input (camera) to trigger it.
   - Modification: Add a small hole (for the lens) and a cavity in the eye STL for the camera module. Wire the camera to the Arduino.
   - Capability: When a face is detected in front of the robot, it automatically connects to other robots via BLE and starts playing a coordinated melody.

7. **Impact Guard with Physical Feedback**: The Setting R Impact Guard (which we assume is an impact sensor) can be enhanced to provide physical feedback (like vibrating the robot when an impact is detected). We'll add a vibration motor (like a small buzzer) and modify the body to have a mounting point.

   - STLs: Modify `V5_AUDIO_VIDEO_BODY.stl` to have a small recess for the vibration motor.
   - Software: Use the existing "Setting R Impact Guard" feature to trigger the vibration motor.
   - Modification: Add a small recess (e.g., 5x5x2mm) in the body for the vibration motor. Wire the motor to the Arduino.
   - Capability: When an impact is detected, the robot vibrates (in addition to playing a sound), providing haptic feedback.

8. **Roller Skates with Directional Audio**: The roller skates can be modified to have directional speakers (so the sound comes from the direction of the wheel). We'll add small speakers to the roller skate supports and wire them to the audio system.

   - STLs: Modify `V5_AUDIO_VIDEO_2x_tire_support_01.stl` and `V5_AUDIO_VIDEO_2x_tire_support_02.stl` to have small cavities for the speakers (on the side of the tire support).
   - Software: We'll use the existing audio system to send different audio channels to the left and right speakers (so directional audio).
   - Modification: Add two small cavities (one on each side) inside the tire support housings to embed slim micro-speakers wired to the UNO Q audio out.
   - Capability: Provides left/right spatial sound cues synced to wheel speed during roller-skate locomotion.
