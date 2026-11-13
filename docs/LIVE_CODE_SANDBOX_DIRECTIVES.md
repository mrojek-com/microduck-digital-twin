# 🦆 MicroDuck Live Code Sandbox & AI Motion Copilot — Architecture & Directives

### Gemma 4 Architecture:
To achieve a 50 Hz (20ms frame budget) digital twin that allows live code injection and LLM-driven motion synthesis, you must decouple the **high-frequency physics/rendering loop** from the **asynchronous logic/inference loop**.

Here is the architectural blueprint for a high-performance, browser-based robotics sandbox.

---

### 1. Runtime Architecture: The Dual-Loop System

To prevent a heavy Python script or an LLM call from freezing the 50 Hz Three.js animation, you must use a **Worker-based decoupled architecture**.

#### A. The High-Frequency Loop (Main Thread / Web Worker)
*   **Role:** Three.js rendering, Kinematics (Forward Kinematics/IK), and Collision Detection.
*   **Frequency:** 50 Hz.
*   **Mechanism:** A `requestAnimationFrame` loop that reads the current state of the 15-DOF robot from a shared memory buffer.

#### B. The Logic Loop (Web Worker)
*   **Role:** Executing user-written code (Python/JS) and managing the state machine.
*   **Frequency:** Variable (Event-driven or lower frequency, e.g., 10 Hz).
*   **Mechanism:** This worker hosts the **Pyodide WASM runtime**. It processes the user's code and sends "Target Pose" commands to the High-Frequency loop.

---

### 2. Execution Engine: Pyodide WASM vs. JS Sandbox

For a robotics twin, **Pyodide (Python WASM)** is the superior choice over a JS sandbox for two reasons: 
1. **Mathematical Ecosystem:** Users can use `NumPy` for matrix transformations and `SciPy` for trajectory optimization.
2. **Familiarity:** Robotics researchers think in Python.

#### The Implementation Strategy:
1.  **The Sandbox:** Run Pyodide inside a **Web Worker**. This ensures that even if the user writes an infinite loop (`while True: pass`), the browser UI and the Three.js animation remain responsive.
2.  **The Bridge:** Use `postMessage` to pass data between the Pyodide Worker and the Three.js Main Thread.

---

### 3. The "Duck API": Exposing the Robot

The user shouldn't interact with raw WebGL or complex Three.js objects. You must provide a "Duck API"—a simplified, high-level abstraction layer.

**The API Design (Python side):**
When the user types code into the `textarea`, you inject a pre-defined Python module that acts as a proxy.

```python
# What the user writes in the textarea:
from robot_api import Robot

bot = Robot()
bot.move_to(x=0.5, y=0.2, z=0.8) # High-level task
bot.arm.rotate(joint_id=3, angle=1.5) # Low-level control
```

**Under the hood (The "Duck" Implementation):**
1.  **The Proxy:** The `Robot` class in Python is a wrapper that calls `pyodide.ffi.to_js`.
2.  **The Command Buffer:** Instead of moving the robot instantly, the Python code populates a **Command Buffer** (a `Float32Array` in SharedArrayBuffer).
3.  **The Interpolator:** The 50 Hz Three.js loop reads the target positions from the buffer and uses **Slerp (Spherical Linear Interpolation)** to smoothly transition the 15-DOF joints between the current state and the user's commanded state.

---

### 4. Safety Sandboxing & Error Handling

Since users are writing live code, you must prevent "System Death."

*   **Timeouts:** Wrap the Pyodide execution in a `Promise.race` that rejects if the script takes longer than 500ms to return a control frame.
*   **Memory Limits:** Configure the WASM heap limit to prevent the browser tab from crashing during large matrix operations.
*   **The "Panic" Button:** Implement a global `AbortController`. If the user hits "Stop," the Main Thread ignores all incoming messages from the Worker and forces the robot to a "Safe Pose" (e.g., all joints at 0).
*   **Input Sanitization:** Use a regex or a simple parser to ensure the `textarea` content doesn't attempt to access `window` or `document` via Python's `js` module.

---

### 5. Integration with Local Ollama (NL-to-Motion)

To enable "Natural Language to Motion," you create a pipeline where an LLM acts as a **Code Generator** rather than a **Controller**.

#### The Workflow:
1.  **User Input:** "Wave your left arm and then pick up the red cube."
2.  **Ollama Inference (Local API):**
    *   The browser sends the prompt to `localhost:11434` (Ollama).
    *   **System Prompt:** *"You are a robotics expert. Output ONLY valid Python code using the `robot_api` library. Do not explain. Use `bot.move_to()` and `bot.arm.rotate()`."*
3.  **Code Synthesis:** Ollama returns a string of Python code.
4.  **Hot-Reload:** The application automatically injects this string into the `textarea` and triggers the Pyodide Worker.
5.  **Execution:** The Pyodide worker executes the generated code, and the

### Qwen 3 MoE Implementation:
We are building a browser-based robotics sandbox with a dual-loop architecture for 50 Hz performance.
 The front-end must include:
   1. A collapsible modal/drawer (with textarea, Run/Stop, Presets, and 'Ask Local AI' prompt bar)
   2. A JS execution wrapper that calls the user script at 50 Hz (with zero GC overhead) via a `onStep(t, dt, duck)` function.
   3. Error boundary that catches exceptions and highlights errors without crashing.

 We'll break down the implementation into the following parts:

 Part 1: The HTML Structure for the Modal/Drawer
 Part 2: The Dual-Loop System in JavaScript (Main Thread and Worker)
 Part 3: The Duck API (Python side) and the Bridge to the Main Thread
 Part 4: The Error Boundary and Safety Features

 However, note: the problem asks for the "concrete front-end components", so we focus on the HTML, CSS, and the main JavaScript that sets up the modal and the dual-loop.

 Let's design the modal:

 - It will be a drawer (like a sidebar) that can be collapsed.
 - It will contain:
      * A textarea for the user to write code (with syntax highlighting? but we'll keep it simple for now)
      * Buttons: "Run", "Stop", and a dropdown for "Presets" (predefined motions)
      * A prompt bar for "Ask Local AI" (with a button to trigger the Ollama call)

 We'll assume the modal is initially collapsed (minimized) and can be expanded.

 We are going to create a file `web/index.html` that includes the modal.

 Steps for the modal:

 1. We'll create a div with id="motion-lab" that is fixed on the right (or left) and can be toggled.
 2. Inside, we have:
      - A header with a toggle button (to collapse/expand)
      - A textarea (id="code-editor")
      - A button "Run" (id="run-btn")
      - A button "Stop" (id="stop-btn")
      - A dropdown for presets (id="preset-select")
      - A prompt bar: a text input (id="ai-prompt") and a button "Ask AI" (id="ask-ai-btn")

 We'll also have a console area (id="console") to show errors and logs.

 Now, the dual-loop system:

 - The main thread (browser) will run the Three.js animation at 50 Hz (using requestAnimationFrame).
 - We'll have a Web Worker for the logic (Pyodide). The main thread will communicate with the worker via postMessage.

 How the main thread (Three.js) loop will work:

   let lastTime = 0;
   function animate(time) {
     requestAnimationFrame(animate);
     const dt = (time - lastTime) * 0.001; // in seconds
     lastTime = time;

     // Read the current state from the shared buffer (which is updated by the worker)
     // But note: the worker writes to a SharedArrayBuffer (or a Float32Array in a SharedArrayBuffer) that the main thread can read.

     // We'll have a shared buffer for the target poses (15 joints, so 15 floats for target, and we also need current state? but the main thread has the current state from the previous frame)

     // Actually, the main thread will have:
     //   - current state (the actual joint angles) stored in a Float32Array (for the current frame)
     //   - target state (from the worker) stored in a separate Float32Array (also in a SharedArrayBuffer)

     // The main thread will interpolate between current and target.

     // We'll have a function `updateRobotState` that is called every frame.

     // We'll also have a function `onStep` that the user's code will call (but note: the user's code runs in the worker, so we don't call it in the main thread).

     // Instead, the user's code in the worker will write to the target buffer. The main thread then uses that target buffer to interpolate.

     // So the main thread loop does:
     //   - Read the target buffer (which the worker has updated)
     //   - Interpolate the current state towards the target (using Slerp for rotations? but note: we are using joint angles, so linear interpolation might be acceptable for small steps, but for rotations we might need Slerp. However, the problem says Slerp, so we assume the target buffer is for angles and we do linear for now? Actually, for joint angles, linear is common. But note: the problem says "Slerp" for the joints, so we'll do Slerp for the rotation part. However, for simplicity in the example, we'll do linear interpolation for the joint angles and note that for rotations we might need to handle differently. But the problem states "Slerp", so we'll assume the target buffer is for the desired joint angles and we do linear interpolation for the angles (which is acceptable for small steps).)

     // We'll create a shared buffer for the target poses (15 floats) and a shared buffer for the current state (which we update in the main thread).

     // However, note: the problem says the main thread reads the current state from a shared buffer. Actually, the main thread has the current state (it's the state of the robot at the current frame). The worker writes the target state to a buffer that the main thread reads.

     // We'll set up two shared buffers:
     //   - `currentJointAngles`: a Float32Array

## 👑 Claude Endgegner Verdict & Implementation




## 👑 Claude Endgegner: The Frontier Mastermind Breakthrough



## 👑 Claude Endgegner: The Frontier Mastermind Breakthrough



## 👑 Claude Endgegner: The Frontier Mastermind Breakthrough

# Endgegner Verdict

**Not naive. It's the frontier standard.** One caveat first: I can't see your repo, so I can't tell you what your simulator does *today*. Below is the architecture I'd build, with the exact contracts. Paste your sim loop and joint list and I'll map it onto your code.

---

## 1. Why this isn't naive

Isaac Sim, MuJoCo's viewer, Webots, and Gazebo all converge on the same loop: **edit behavior → hot-reload → watch it → measure → iterate**. A browser-based, live-code, LLM-assisted version is that loop with the friction removed:

- **Zero setup.** No toolchain, no flashing, no broken robot. A bad idea costs a reset button.
- **Sim-first safety.** Anything the LLM writes runs against a virtual duck first. Hallucinated joint angles break nothing real.
- **Tight loop.** Prompt → code → run → see → "make it wobble less" → run takes about 10 seconds.
- **LLMs are good at this.** A small, typed, well-documented API (like the `duck` API below) is exactly where even local 30B models write correct code.

The real design decisions are **sandboxing** and **determinism**, covered next.

---

## 2. Execution engine

```
┌────────────── Main thread ──────────────┐      ┌──── Web Worker (sandbox) ────┐
│ Physics/Render @ fixed 50 Hz            │ ───► │ user code: onStep(t,dt,duck) │
│  1. snapshot sensors (imu, tof, joints) │      │ no DOM, no network, no fs    │
│  2. post {t,dt,sensors} to worker       │ ◄─── │ returns command batch        │
│  3. validate + clamp commands           │      └──────────────────────────────┘
│  4. apply to sim, step physics          │
└──────────────────────────────────────────┘
```

**Rules that make it robust:**
- **Fixed timestep with accumulator.** `dt = 0.02` always, so runs are reproducible and A/B-comparable.
- **Worker isolation.** Pasted code can't touch the page or your API keys. A watchdog kills the worker if a step takes more than about 8 ms or doesn't return, then shows the error in the console panel.
- **Commands are data, not side effects.** The `duck` object in the worker only *records* commands. The main thread clamps them to joint limits and rate limits, so the simulator is the authority.
- **Python.** Run Pyodide inside the same worker, with the same `duck` proxy and the same `onStep` signature. JS is instant to start. Python costs a ~10 MB load and is worth it for people who want it.

```js
// worker.js (core)
let userStep = null;
onmessage = ({data}) => {
  if (data.type === 'load') {
    try {
      userStep = new Function('duck', 'console',
        `${data.code}\n; return typeof onStep==='function' ? onStep : null;`
      )(null, console