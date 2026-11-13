# 🦆 MicroDuck Simulator — Tri-Council & Claude Upgrade Specification

## 🏛️ Local Tri-Council Swarm Deliberation

### Gemma 4 (Structural UX & Title Analysis)
The current UX suffers from **"Information Overload via Concatenation."** By forcing the product name, version, purpose, and real-time telemetry into a single string, you are forcing the user to parse a "wall of text" to find anything useful.

### 1. Analysis: Why the current title is awkward

The current string `MicroDuck Alpha 3D Digital Twin [Badge]` fails for three specific reasons:

*   **Redundancy (Semantic Noise):** "Alpha" and "3D" are technical descriptors that belong in a versioning system or a settings menu, not the primary brand identity. "Digital Twin" is a category, not a name. Including it in the title makes the UI look like a prototype rather than a professional tool.
*   **Cognitive Load (The "Everything is Important" Trap):** When the title is a sentence, the user's eye doesn't know where to land. Is the most important thing the name of the robot? The fact that it's an Alpha? Or the current Hz?
*   **Lack of Visual Hierarchy:** By putting the telemetry (Hz, Latency, Multiplier) in the same line as the title, you are treating **transient data** (which changes every millisecond) with the same visual weight as **static identity** (the name of the project). This creates a "jittery" UI feeling where the title appears to be constantly vibrating because the numbers are changing.

---

### 2. The Recommended Architecture

To achieve an "authentic bipedal robotics" feel (reminiscent of ROS, NVIDIA Isaac Sim, or high-end industrial interfaces), you must separate **Identity**, **Status**, and **Telemetry**.

#### The Hierarchy Strategy:
1.  **Primary Identity (Static):** The Name.
2.  **Secondary Metadata (Contextual):** Version/Build.
3.  **System Status (Dynamic):** Performance/Health.
4.  **Telemetry (Real-time):** High-frequency data.

---

### 3. The Ideal Implementation

#### A. The Title & Subtitle (The "Identity" Layer)
Move the "Alpha" and "Digital Twin" descriptors to a subtitle. This cleans up the brand and provides professional context.

*   **Main Title:** `MicroDuck`
*   **Subtitle:** `Bipedal Digital Twin | v0.3-alpha`

#### B. The Badge/Status Bar (The "Telemetry" Layer)
Instead of one long string, use **segmented badges** with distinct visual styles. Use a "Monospace" font for these to prevent "layout shift" (where numbers changing width cause the UI to jump).

**The Badge Hierarchy (Left to Right):**

1.  **The Sync Badge (Performance):**
    *   *Content:* `50 Hz`
    *   *Style:* Green or Cyan text. Indicates the heartbeat of the simulation.
2.  **The Latency/Control Badge (The "Loop" Data):**
    *   *Content:* `obs: 1.6ms | act: 14.2ms`
    *   *Style:* Subtle grey/amber. This is the "Observer-to-Actuator" delta. It is critical for robotics engineers but shouldn't scream for attention.
3.  **The Hardware/Scale Badge (The "Instance" Data):**
    *   *Content:* `XL330 ×15`
    *   *Style:* Blue or White. Indicates the hardware configuration being simulated.

---

### 4. Visual Mockup of the New UX

**Current (Bad):**
`MicroDuck Alpha 3D Digital Twin [50 Hz · obs[1,61] → act[1,14] · 15× XL330]`

**Recommended (Professional):**

| **MicroDuck** |
| :--- |
| <small>Bipedal Digital Twin • Build Alpha 3</small> |

**[ 50 Hz ]** &nbsp;&nbsp; `obs: 1.6ms` &nbsp;&nbsp; `act: 14.2ms` &nbsp;&nbsp; | &nbsp;&nbsp; **XL330 [x15]** |

---

### Summary of Improvements
*   **Stability:** By separating the name from the telemetry, the name stays still. The user's eyes can rest.
*   **Scanability:** An engineer can look at the top bar and instantly see "I am in the MicroDuck environment, running at 50Hz, with 14ms latency."
*   **Professionalism:** Removing "3D" (which is implied in WebGL) and "Digital Twin" from the primary header moves the product from a "project" to a "platform."

### Qwen 3 MoE (Front-End & Shader Directives)
Here's the precise implementation for a professional, high-fidelity robotics simulator UI based on Gemma 4's critique. All code is production-ready with rationale:

---

### **1. Modern Header Implementation (HTML/CSS)**
```html
<header class="sim-header">
  <div class="header-content">
    <h1 class="main-title">MicroDuck</h1>
    <div class="subtitle">Bipedal Digital Twin • v0.3-alpha</div>
  </div>
  <div class="telemetry-badges">
    <span class="badge sync">50 Hz</span>
    <span class="badge latency">obs: 1.6ms | act: 14.2ms</span>
    <span class="badge hardware">XL330 ×15</span>
  </div>
</header>
```

```css
/* Glassmorphism topbar with professional spacing */
.sim-header {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  padding: 12px 24px;
  background: rgba(255, 255, 255, 0.12);
  backdrop-filter: blur(12px);
  border-bottom: 1px solid rgba(255, 255, 255, 0.15);
  display: flex;
  justify-content: space-between;
  align-items: center;
  z-index: 1000;
  font-family: 'Inter', sans-serif;
  color: #e0e0ff;
  box-shadow: 0 4px 24px rgba(0, 0, 0, 0.2);
}

.header-content {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.main-title {
  font-size: 1.8rem;
  font-weight: 700;
  letter-spacing: -0.02em;
  margin: 0;
  text-shadow: 0 0 8px rgba(0, 100, 255, 0.3);
}

.subtitle {
  font-size: 0.85

### DeepSeek-R1 (Adversarial Usability & Stability Audit)
### **UI and Header Implementation Audit**

The proposed UI and header design is modern, professional, and implements key feedback from Gemma 4 and Qwen 3. However, there are a few potential pitfalls and areas for improvement to ensure a robust, high-fidelity implementation:

---

### **1. Layout Shift on Mobile/Compact Viewports**
- **Issue**: The fixed header may cause layout shift regressions on mobile devices, particularly when the viewport is compact. The `position: fixed` property could interfere with scrolling behavior or content positioning.
- **Solution**: Ensure the header's `z-index` is appropriately managed to prevent it from overlapping critical content (e.g., navigation buttons or canvas controls). Consider adding a "safe area" margin around the header for compact viewports.

---

### **2. Button Event Collisions**
- **Issue**: The telemetry badges (`<span class="badge">`) lack click handlers or interactive states, which could lead to unintended behavior if they overlap with control buttons.
- **Solution**: Add `pointer-events: none` to the `.badge` class to prevent unintended clicks. Alternatively, ensure proper spacing between badges and interactive elements.

---

### **3. WebGL Canvas Resize Behavior**
- **Issue**: The header's fixed positioning and backdrop-filter could interfere with WebGL canvas rendering, particularly if the canvas is not properly sized or recalculated on window resize.
- **Solution**: Implement a resize handler for the WebGL canvas to ensure it adapts to viewport changes. Use `window.addEventListener('resize', () => { ... })` to trigger canvas resizing.

---

### **4. Performance Considerations**
- **Backdrop-Filter**: The `backdrop-filter: blur(12px)` property may impact performance on devices with weaker GPU acceleration. Consider reducing the blur intensity or using a fallback for older browsers.
- **Z-Index Management**: The header's `z-index: 1000` should be carefully managed to prevent conflicts with other elements (e.g., modals or overlays).

---

### **5. Typography and Spacing**
- **Issue**: The subtitle font size (`0.85`) appears incomplete in the code snippet. Ensure proper font sizing and spacing for optimal readability.
- **Solution**: Use a consistent typography scale (e.g., `0.875rem`) and ensure proper line height and letter spacing for all text elements.

---

### **6. Mobile Responsiveness**
- **Issue**: The header's padding (`padding: 12px 24px`) may be too large for compact viewports, leading to content truncation or overflow.
- **Solution**: Implement responsive padding using media queries:
  ```css
  @media (max-width: 768px) {
    .sim-header {
      padding: 8px 16px;
    }
  }
  ```

---

### **7. Interactive States**
- **Issue**: The header and badges lack hover or active states, which could reduce their perceived interactivity.
- **Solution**: Add subtle hover effects for interactive elements (e.g., buttons or links):
  ```css
  .sim-header:hover {
    background: rgba(255, 255, 255, 0.15);
  }
  ```

---

### **8. Canvas Performance**
- **Issue**: The WebGL canvas may not properly account for the header's height when resizing, leading to rendering issues.
- **Solution**: Ensure the canvas height is adjusted to account for the header's fixed height:
  ```javascript
  const headerHeight = document.querySelector('.sim-header').offsetHeight;
  canvas.height = window.innerWidth * (window.innerHeight - headerHeight);
  ```

---

### **


## 👑 Claude Endgegner: The Finishing Touch

# FINISHING TOUCH DIRECTIVES: MicroDuck

**Caveat:** I can't see your actual files, so selectors and variable names (`controls`, `renderer`, `#hud`) are assumptions. Rename them to match your code.

## 0. Council corrections (apply before anything else)

- **Gemma and Qwen misread the telemetry.** `obs[1,61] → act[1,14]` is tensor shape (61-dim observation, 14-dim action), not latency. Don't show `1.6ms / 14.2ms`, which would be invented data. Show the shapes as `obs 61 → act 14`, and add a real `step` time only if you measure it.
- **DeepSeek's canvas formula is wrong.** `canvas.height = innerWidth * (innerHeight - headerHeight)` multiplies width by height. Don't resize the canvas for the header. Let the header overlay the canvas, or put the canvas in a flex/grid row below it.
- **Drop `backdrop-filter` over the WebGL canvas.** It forces a re-blur every frame. Use a translucent gradient instead.

## 1. Title and subtitle

| Element | Final copy |
|---|---|
| **Title** | **MicroDuck** |
| **Subtitle** | **Bipedal Digital Twin** |
| **Version chip** | `v0.3` (muted, small) |
| **`<title>` tag** | `MicroDuck — Bipedal Digital Twin` |

"Alpha" and "3D" are gone. Maturity lives in the chip, not the brand.

## 2. Header and badges

**web/index.html**
```html
<title>MicroDuck — Bipedal Digital Twin</title>

<header class="hud-header">
  <div class="brand">
    <h1>MicroDuck<span class="ver">v0.3</span></h1>
    <p class="tagline">Bipedal Digital Twin</p>
  </div>

  <div class="badges" role="status" aria-live="off">
    <span class="badge badge--live" id="b-hz"><i class="dot"></i><b>50</b> Hz</span>
    <span class="badge" id="b-io">obs <b>61</b> → act <b>14</b></span>
    <span class="badge" id="b-hw"><b>15</b>× XL330</span>
  </div>
</header>
```

**web/styles.css**
```css
:root{
  --bg-glass: linear-gradient(180deg, rgba(10,14,24,.78), rgba(10,14,24,.35) 70%, transparent);
  --fg: #e8ecf7; --fg-dim: #8b95ad;
  --accent: #ffcf3d;           /* duck yellow */
  --ok: #3ddc97; --warn: #ffb020; --bad: #ff5d6c;
  --ease: cubic-bezier(.2,.8,.2,1);
}

.hud-header{
  position: fixed; inset: 0 0 auto 0; z-index: 10;
  display: flex; justify-content: space-between; align-items: center;
  padding: max(12px, env(safe-area-inset-top)) 20px 18px;
  background: var(--bg-glass);
  color: var(--fg);
  font: 500 14px/1.2 Inter, system-ui, sans-serif;
  pointer-events: none;                 /* canvas stays draggable under header */
}
.hud-header button, .hud-header a { pointer-events: auto; }

.brand h1{ margin:0; font-size:1.35rem; font-weight:700; letter-spacing:-.02em; }
.brand .ver{
  margin-left:8px; padding:2px 6px; border-radius:6px;
  font: 600 .65rem/1 ui-monospace, "JetBrains Mono", monospace;
  color: var(--accent); border:1px solid color-mix(in srgb, var(--accent) 40%, transparent);
  vertical-align: middle;
}
.tagline{ margin:3px 0 0; font-size:.75rem; letter-spacing:.08em; text-transform:uppercase; color:var(--fg-dim); }

.badges{ display:flex; gap:8px; }
.badge{
  min-width: 5.5ch; padding:6px 10px; border-radius:999px;
  background: rgba(255,255,255,.06);
  border: 1px solid rgba(255,255,255,.10);
  font: 500 .75rem/1 ui-monospace, "JetBrains Mono", monospace;
  font-variant-numeric: tabular-nums;   /* no digit jitter */
  color: var(--fg-dim); white-space: nowrap;
}
.badge b{ color: var(--fg); font-weight:600; }
.badge--live .dot{
  display:inline-block; width:6px; height:6px; margin-right:6px; border-radius:50%;
  background: var(--ok); box-shadow: 0 0 8px var(--ok);
  animation: pulse 2s ease-in-out infinite;
}
.badge--live[data-health="warn"] .dot{ background:var(--warn); box-shadow:0 0 8px var(--warn); }
.badge--live[data-health="bad"]  .dot{ background:var(--bad);  box-shadow:0 0 8px var(--bad); }
@keyframes pulse{ 50%{ opacity:.45; transform:scale(.8);} }

@media (max-width:768px){
  .hud-header{ padding:8px 14px 14px; }
  .tagline, #b-hw{ display:none; }
}
@media (prefers-reduced-motion: reduce){ .badge--live .dot{ animation:none; } }
```

## 3. Top 3 micro-polish touches

1. **Calm telemetry.** Update the badges at about 4 Hz (not per frame), use tabular numerals, and color the status dot by health. The title stays still and only the digits change.
2. **Camera feel.** Use damped orbit (`dampingFactor ≈ 0.08`), clamp the polar angle so the camera never dips under the floor, and set sensible min/max zoom. Add a soft contact shadow and a faint warm rim light on the duck so it separates from the background.
3. **Button states.** Every control gets distinct hover, `:active` (1px press), `:focus-visible` (accent ring), and `[aria-pressed="true"]` (accent fill) states, with 120–160 ms transitions.

## 4. Patches

**web/styles.css** (buttons)
```css
button.ctl{
  padding:8px 14px; border-radius:10px; cursor:pointer;
  background: rgba(255,255,255,.06); color: var(--fg);
  border:1px solid rgba(255,255,255,.12);
  transition: background .14s var(--ease), transform .14s var(--ease), border-color .14s;
}
button.ctl:hover{ background: rgba(255,255,255,.12); border-color: rgba(255,255,255,.22); }
button.ctl:active{ transform: translateY(1px) scale(.98); }
button.ctl:focus-visible{ outline:2px solid var(--accent); outline-offset:2px; }
button.ctl[aria-pressed="true"]{ background: var(--accent); color:#1a1400; border-color: transparent; }
button.ctl:disabled{ opacity:.4; cursor:not-allowed; }
```

**web/simulator.js** (camera, lighting, telemetry)
```js
// --- Camera damping (call controls.update() every frame, or damping does nothing) ---
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.rotateSpeed   = 0.7;
controls.zoomSpeed     = 0.8;
controls.minDistance   = 0.15;
controls.maxDistance   = 2.5;
controls.maxPolarAngle = Math.PI * 0.495;   // never below the floor
// in animate(): controls.update();

// --- Shadow + rim light (Three.js) ---
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.radius = 4;
keyLight.shadow.bias = -0.0002;
const rim = new THREE.DirectionalLight(0xffd88a, 0.6);
rim.position.set(-1, 0.8, -1.2);
scene.add(rim);

// --- Resize: size to container, NOT header math ---
function onResize(){
  const w = innerWidth, h = innerHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', onResize); onResize();

// --- Throttled, real telemetry ---
const $ = id => document.getElementById(id);
const tel = { last: performance.now(), steps: 0, hz: 0 };
function onPolicyStep(){ tel.steps++; }            // call once per policy step

setInterval(() => {                                // ~4
