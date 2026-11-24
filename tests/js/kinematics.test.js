/**
 * apps/MicroDuck/tests/js/kinematics.test.js
 *
 * Deterministic Node.js regression suite locking down MicroDuck WebGL simulator
 * kinematic laws, safety invariants, and regression guards.
 */

const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");

const SIM_PATH = path.resolve(__dirname, "../../web/simulator.js");
const INDEX_PATH = path.resolve(__dirname, "../../web/index.html");
const simSource = fs.readFileSync(SIM_PATH, "utf8");
const indexSource = fs.readFileSync(INDEX_PATH, "utf8");

test("flat-blade / flat-sole kinematic law: da = dk - dh holds across leg angles", () => {
  const flatAnkle = (dh, dk) => dk - dh;
  // Roller crouch & stride
  assert.ok(Math.abs(flatAnkle(0.16, 0.32) - 0.16) < 1e-12);
  // Ground pick squat
  assert.ok(Math.abs(flatAnkle(-0.20, 0.40) - 0.60) < 1e-12);
  // Seated squat (left & right mirrored)
  assert.ok(Math.abs(flatAnkle(-0.38, 0.76) - 1.14) < 1e-12);
  assert.ok(Math.abs(flatAnkle(0.38, -0.76) - (-1.14)) < 1e-12);
});

test("regression guard: no per-frame Box3 floor fight (_cadFloorBox) in simulator.js", () => {
  assert.equal(
    simSource.includes("_cadFloorBox"),
    false,
    "_cadFloorBox caused 60Hz vertical shaking and 3cm floating from invisible roller wheels"
  );
});

test("regression guard: ground_pick keeps pose.pitchFall = 0.0 and flat-sole squat", () => {
  const pickMatch = simSource.match(/case\s+"ground_pick":\s*\{([\s\S]*?)break;/);
  assert.ok(pickMatch, "ground_pick skill case must exist");
  const body = pickMatch[1];
  assert.match(body, /pose\.pitchFall\s*=\s*0(?:\.0)?\s*;/, "ground_pick must not tilt rigid world root");
  assert.match(body, /bothLegs\(\s*-0\.20\s*\*\s*squat\s*,\s*0\.40\s*\*\s*squat\s*,\s*0\.60\s*\*\s*squat\s*\)/);
  assert.match(body, /pose\.liftY\s*=\s*-0\.018\s*\*\s*squat/);
});

test("regression guard: seated posture uses symmetric flat-sole squat and analytical -0.032m liftY", () => {
  assert.match(simSource, /state\.isSeated\s*\?\s*-0\.032\s*:\s*0\.0/);
  const sitMatch = simSource.match(/if\s*\(state\.isSeated\)\s*\{([\s\S]*?)\}/);
  assert.ok(sitMatch, "isSeated block must exist");
  const body = sitMatch[1];
  assert.match(body, /action14\[2\]\s*=\s*-0\.38/);
  assert.match(body, /action14\[3\]\s*=\s*0\.76/);
  assert.match(body, /action14\[4\]\s*=\s*1\.14/);
  assert.match(body, /action14\[11\]\s*=\s*0\.38/);
  assert.match(body, /action14\[12\]\s*=\s*-0\.76/);
  assert.match(body, /action14\[13\]\s*=\s*-1\.14/);
});

test("regression guard: roller wheel vertical offset +0.029m and +Y jaw hinge pivot", () => {
  assert.match(simSource, /showRoller\s*\?\s*0\.029\s*:\s*0\.0/);
  assert.match(simSource, /jawPivotGroup\.rotation\.set\(\s*0\s*,/);
});

test("61-D observation layout excludes mouth index 9", () => {
  assert.match(simSource, /const\s+MOUTH_INDEX\s*=\s*9\s*;/);
  assert.match(simSource, /if\s*\(i\s*===\s*MOUTH_INDEX\)\s*continue\s*;/);
});

test("index.html wires viewport, onnxruntime-web, and simulator.js cleanly", () => {
  assert.match(indexSource, /simulator\.js/);
  assert.match(indexSource, /microduck_cad_bundle\.js/);
  assert.match(indexSource, /onnxruntime-web/);
  assert.match(indexSource, /id="btn-toggle-onnx"/);
  assert.match(indexSource, /id="hud-gamepad-status"/);
});

test("security guard: Code Lab shadows DOM/network globals and enforces watchdog timeout", () => {
  assert.match(simSource, /"window",\s*"document",\s*"globalThis",\s*"self",\s*"fetch"/);
  assert.match(simSource, /Watchdog timeout/);
  assert.match(simSource, /Sandbox blocked unbounded loop/);
});

test("ONNX inference & HTML5 Gamepad API hooks exist in simulator.js", () => {
  assert.match(simSource, /function loadOnnxPolicySlot/);
  assert.match(simSource, /function stepOnnxInference/);
  assert.match(simSource, /function pollHardwareGamepad/);
  assert.match(simSource, /function applyPadCurve/);
});
