# What Else Is Possible With MicroDuck? — Sovereign Expansion RFC

- **Author Model:** `qwen3:30b-a3b` (Local Master Implementer)
- **Generated:** 2026-11-13 15:28:32 UTC

---

1. **Dynamic Obstacle Avoidance**  
(a) `autod/src/brain.rs` (obstacle module) + `autod/src/obstacle.rs`  
(b) ONNX model processes 8x8 ToF depth map → classifies obstacles → adjusts `Step` targets via `brain`'s path planner  
(c) Navigates cluttered, unstructured environments without pre-mapped paths (e.g., avoiding moving people)

2. **Voice-Controlled Navigation**  
(a) `autod/src/brain.rs` (Heard enum) + `robot.sound` API  
(b) ONNX keyword spotting on 48kHz audio → maps commands ("left," "stop") to `Step` targets  
(c) Hands-free operation for mobility-impaired users via natural speech

3. **NFC-Triggered Path Following**  
(a) `robotd` (NFC service) + `autod/src/map.rs`  
(b) NFC tags encode waypoints → `robotd` logs positions → `brain` uses odometry to follow tagged paths  
(c) User-defined physical navigation routes (e.g., museum exhibits) without app setup

4. **Laser Dot Chasing**  
(a) `autod/src/brain.rs` (Laser struct) + `laser.rs`  
(b) `Laser.dot` state → `brain` computes pursuit trajectory → adjusts `Step` targets in real-time  
(c) Playful, reactive interaction with laser pointers (e.g., public engagement at events)

5. **WebRTC Remote Operation**  
(a) `robotd` (WebRTC camera) + `robotctl` sockets  
(b) Browser-based WebRTC stream + rendezvous control lane → sends `Step` intents via JSON-RPC  
(c) Zero-install remote control from any device (e.g., emergency operation via smartphone)


6. **Autonomous Fall Recovery & Impact Guard Self-Righting**
   - (a) `fall_recovery.py`, `impact_guard.py`
   - (b) Utilizes inertial measurement unit (IMU) data to detect falls and impact forces to trigger self-righting mechanisms.
   - (c) Enables the robot to recover from falls autonomously and protect itself from impact damage.

7. **Contactless ToF Theremin & Musical Performance Mode**
   - (a) `tof_theremin.py`, `musical_performance.py`
   - (b) Employs time-of-flight (ToF) sensors to detect hand gestures and translate them into musical notes.
   - (c) Allows users to play music through hand gestures without physical contact.

8. **Multi-Duck BLE Chorale Swarm Choreography (wistful.duckscore ±20ms sync)**
   - (a) `ble_chorale.py`, `wistful.duckscore`
   - (b) Uses Bluetooth Low Energy (BLE) for synchronization and choreography among multiple duck robots.
   - (c) Enables synchronized swarm performances with precise timing (±20ms).

9. **Pet-Detection Gesture Trigger & Tactile Pickup Personality (pet_detect.onnx + pickup_detector.onnx)**
   - (a) `pet_detect.onnx`, `pickup_detector.onnx`, `gesture_trigger.py`
   - (b) Implements neural network models for pet detection and gesture recognition to trigger tactile responses.
   - (c) Allows the robot to interact with pets through gestures and tactile feedback.

10. **Roller-Skate Hybrid Policy & Cross-Hardware XGO-Duck Sim-to-Real Bridge**
    - (a) `roller_skate_policy.py`, `sim_to_real_bridge.py`
    - (b) Combines roller-skate dynamics with hybrid control policies and sim-to-real transfer techniques.
    - (c) Enables the robot to transition smoothly from simulation to real-world environments.
