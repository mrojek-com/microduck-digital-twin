# MicroDuck Deep Codebase & Systems Analysis — Local Tri-Council Report

- **Generated:** 2026-11-13 15:28:32 UTC
- **Target Repository:** `https://github.com/pollen-robotics/microduck` (cloned to `apps/MicroDuck/repo`)
- **Execution Law:** Sequential Apple Silicon Metal inference (`keep_alive: 0`, `num_ctx: 4096`, `think: false`)

---

## 1. Structural & Multi-Daemon Architecture — `gemma4:26b`

Based on the provided documentation for the `microduck` workspace, here is a deep technical analysis of the architecture.

### 1. The Multi-Daemon NDJSON Architecture and the Recovery Triad

The architecture utilizes a distributed service model where eight distinct daemons communicate via **Unix Domain Sockets** using **JSON-RPC 2.0 over NDJSON** (Newline Delimited JSON). This design decouples the high-frequency control loop from the low-frequency management and transport layers.

#### The Service Split
*   **The Controller (`robotd`):** The core of the system. It runs a high-priority **50 Hz loop** that manages the Dynamixel servo bus (15 servos + IMU) and implements safety logic. It is the only daemon with direct hardware access to the motor controllers.
*   **The Sensors/Transports (`tofd`, `mediad`, `mapd`):** These are "ownerless" services. For example, `tofd` (the depth matrix provider) acts as a pure publisher; it writes to a socket and "reads no one," allowing `robotd` and `mediad` to consume the data without the sensor needing to manage client states.
*   **The Management Layer (`configd`, `updaterd`, `btd`):** These handle the "meta" aspects of the robot (identity, connectivity, and software lifecycle).

#### The Zero-Dependency Recovery Triad
The architecture explicitly separates the "brain" (`robotd`) from the "nervous system" and "identity" to ensure that even if the robot's control logic crashes or is corrupted, the board remains reachable. This is achieved through the **Recovery Triad**:

1.  **`btd` (Bluetooth Daemon):** Manages the BLE/USB interface for the gamepad and phone.
2.  **`configd` (Configuration Daemon):** Manages the robot's identity (name, WiFi, bonding).
3.  **`updaterd` (Update Daemon):** Manages the A/B rootfs swapping and software updates.

**Why this matters:** If a new `robotd` binary is buggy and causes a kernel panic or a loop crash, the user can still connect via a phone (via `btd`) or a laptop to trigger an update or a configuration change. Because `updaterd` and `configd` do not depend on the 50 Hz control loop to function, they can facilitate a "rollback" or a "re-flash" even when the robot is physically unable to move.

---

### 2. State Ownership and A/B Rootfs Updates

To support robust, fail-safe updates (as hinted at in the `updater-design.md` reference and the `updaterd` description), the system employs a two-layer configuration/state strategy:

*   **`/etc/robot/robotd.toml` (Static/Immutable Configuration):** This represents the "desired state" or the configuration baked into the current OS image. It defines how the system *should* behave upon boot.
*   **`/var/lib/robot/robotd.toml` (Dynamic/Persistent State):** This represents the "actual state" or runtime overrides. 

**The A/B Update Mechanism:**
The `updaterd` manages the transition between two different root filesystems. When an update is applied:
1.  The system downloads a new image.
2.  The `updaterd` performs a `swap` of the active partition.
3.  The system restarts.
4.  **The Health Gate:** Upon reboot, the system doesn't just assume success. It monitors `robot.health`. If the new version fails to initialize the hardware or the 50 Hz loop fails to stabilize, the `updaterd` detects the failure and rolls back to the previous known-good partition.

This separation ensures that even if a configuration change is made (via `configd`), it can be persisted across updates or reverted if the new software version is incompatible with the existing hardware state.

---

### 3. The MuJoCo Twin and the Real-Time Health Gate

The `scripts/duck-sim` environment is designed to be a "digital twin" that is functionally indistinguishable from the hardware at the software level.

#### The Simulation Interface
The simulation uses a "seam" approach. Instead of the `robotd` code knowing it is in a simulator, the `duck_control::sim::RemoteIo` trait replaces the physical Dynamixel/UART driver. 
*   **Input:** The simulator (MuJoCo) sends joint positions, velocities, and IMU data over a TCP socket to `robotd`.
*   **Output:** `robotd` sends its calculated motor targets back to the simulator.
*   **Result:** This allows developers to test the entire stack—including the `robotctl` CLI, the JSON-RPC IPC, and the complex 50 Hz control policies—without needing a physical robot.

#### The 45 Hz Health Gate & Real-Time Factor
In robotics, control loops are strictly time-dependent. The `robotd` loop is designed for **50 Hz (20ms per tick)**. 

In the simulation, if the MuJoCo physics engine or the computer's CPU cannot keep up with the demands of the control loop, the **Real-Time Factor (RTF)** drops below 1.0x. This means the simulation is running "slower" than real-time.

**The Rollback Trigger:**
The `updaterd` and `robotd` monitor the stability of the control loop. If the system detects that the loop frequency is inconsistent or dropping (e.g., falling toward or below **45 Hz**), it triggers a failure state. In a real-world deployment, this prevents a "zombie" robot from running with stale sensor data or delayed motor commands, which could lead to physical oscillations or hardware damage. In the context of the `updaterd`, if a new software version causes the control loop to jitter or slow down, the **health gate** will automatically trigger a rollback to the previous stable version.

---

## 2. Quantitative Control, Kinematics & 48 kHz DSP — `mistral-small:24b`

### 1. Observation Vector Layout

The observation vector is a 61-dimensional array that serves as the input to the policy, which in turn produces a 14-dimensional action vector. The layout of the observation vector is as follows:

- **Indices 0-2**: Gyroscope data (trunk frame, rad/s)
- **Indices 3-5**: Projected gravity vector (trunk frame, unit vector)
- **Indices 6-19**: Joint positions relative to the home pose (14 joints, excluding the mouth)
- **Indices 20-33**: Joint velocities (14 joints, excluding the mouth)
- **Indices 34-47**: Previous actions (14 joints, excluding the mouth)
- **Indices 48-60**: Command block (13 values)

The command block includes:
- **Indices 48-50**: Velocity commands (vx, vy, vyaw)
- **Indices 51-54**: Head commands (neck_pitch, head_pitch, head_yaw, head_roll)
- **Indices 55-56**: Body x and y commands (always zero)
- **Index 57**: Body z command
- **Index 58**: Body roll command
- **Index 59**: Body pitch command
- **Index 60**: Body yaw command (always zero)

**Exclusion of Mouth Index 9**: The mouth index (9) is excluded because it is not part of the policy's action space. The policy is designed to control 14 joints, and the mouth is not one of them.

**Ordering of `[body.z, body.roll, body.pitch]`**: The ordering of these commands is crucial because it matches the expected input format for the policy. The policy was trained with this specific ordering, and any deviation would result in incorrect behavior.

### 2. Kinematics Flattening for Zero-Allocation 50 Hz FK

The `kinematics` module optimizes forward kinematics (FK) by flattening the MJCF root-to-site chains. This optimization ensures zero allocation during the 50 Hz control loop, which is critical for real-time performance. Here’s how it works:

1. **Model Compilation**: The kinematic tree is compiled once at load time. This involves resolving names to indices and creating flattened root-to-site chains for each site.
2. **Query Optimization**: Each query for a site's position involves a fold over its precomputed chain, avoiding hashing, allocation, and recomputation of body positions.
3. **Efficient Data Structures**: Joint angles are stored in a plain `&[f64]` array, indexed by the model's joint order. This ensures that joint indices are resolved once during initialization, not per query.

This approach is verified through tests that compare the results against MuJoCo's `mj_kinematics` on 64 random poses, ensuring correctness.

### 3. Audio Synthesis at 48 kHz and BLE Chorale Sync

**Synthesis at 48 kHz**: The `sounds` module synthesizes audio at 48 kHz because the Radxa's I²S clock tree is pinned to the 48 kHz family. This avoids the need for resampling, which was previously done with ffmpeg. The original synthesis rate was 22.05 kHz, but rendering at 48 kHz natively removes this step.

**Rescaling Time-Domain DSP Constants**: Two constants (jitter smoothing window and pink-noise pole) were originally tuned at 22.05 kHz. These are rescaled by `SR / 22050` to maintain their time-domain character, ensuring that the synthesized audio retains the intended characteristics.

**BLE Chorale Sync**: The BLE Chorale achieves ±20 ms ensemble synchronization without NTP by leveraging the precise timing capabilities of Bluetooth Low Energy (BLE). BLE allows for low-latency communication and synchronization, enabling multiple devices to stay in sync within a tight time window. This is crucial for applications requiring precise timing, such as synchronized audio playback.

---

## 3. Forensic Safety & Failure-Mode Audit — `deepseek-r1:14b`

**Step-by-Step Explanation and Answer:**

1. **ROBOTIS Impact Guard Factory Level 5 Causing Stance-Knee Collapse:**
   - **Issue:** At Level 5, the robot's stance-knee mechanism collapses frequently, while Setting R avoids this.
   - **Reasoning:** The safety subsystem's clamping might not adequately restrict the knee's movement under high forces, allowing it to exceed structural limits. This could be due to incorrect torque limits or insufficient safety checks in the control loop.
   - **Impact:** Frequent collapses can lead to mechanical damage and reduce operational safety.

2. **Silent Observation-Index Drift and Safety Enforcement:**
   - **Issue:** Drift in observation indices leads to incorrect state tracking, and safety only enforces global limits.
   - **Reasoning:** The safety system might not track each joint's state accurately, leading to some joints exceeding their individual limits. Global clamping overlooks per-joint constraints defined in MJCF.
   - **Impact:** This can cause actuators to malfunction or damage components beyond their intended operational range.

3. **Deadman Switch vs. Fall Limp Interaction:**
   - **Issue:** The deadman switch enforces a stop on lost commands, while `fall_limp` aims for controlled movement post-fall.
   - **Reasoning:** Inconsistent interaction between these systems can lead to the robot not stopping when necessary or limp when it should halt, affecting safety during falls or communication losses.
   - **Impact:** This inconsistency can result in unpredictable behavior, increasing the risk of accidents.

4. **Pickup Detector Edge Cases in Version Transition:**
   - **Issue:** Transitioning from v1 to v2 introduces detection issues with head grips, inversions, and current discrepancies.
   - **Reasoning:** The new model may not handle all real-world scenarios, especially those not adequately trained in simulations, leading to incorrect hold or release detection.
   - **Impact:** Incorrect detection can cause the robot to remain active when held, risking damage, or stop unnecessarily when it's safe.

5. **VL53L8CX ToF Sensor Status Code Discrepancies:**
   - **Issue:** Inconsistent status codes cause inaccurate distance measurements, leading to a 30 cm cutoff issue.
   - **Reasoning:** The sensor's status codes not aligning leads to incorrect distance calculations, affecting navigation and obstacle avoidance.
   - **Impact:** This can cause the robot to mishandle objects or environments, leading to collisions or inefficient movement.

6. **Simulation Timing Affects Health Monitoring:**
   - **Issue:** Simulated real-time inaccuracies trigger false health alerts and update rollbacks.
   - **Reasoning:** The health system monitors for real-time operation, and simulation drift causes it to misinterpret the robot's state, leading to unnecessary resets.
   - **Impact:** Frequent rollbacks disrupt operation and can cause system instability, reducing uptime and reliability.

**Conclusion:**
Each of these failure modes highlights critical areas where the robot's subsystems can fail or behave unexpectedly. Addressing them requires a thorough review of the control algorithms, safety limits, sensor accuracy, and system interactions to ensure robust and reliable operation.
