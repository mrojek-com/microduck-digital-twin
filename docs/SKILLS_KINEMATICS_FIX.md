```javascript
function applySkillAnimation(name, p, action14, state) {
  // Conventions: negative hip_pitch lifts the thigh forward, positive knee flexes,
  // positive rollFall / right hip_roll lean toward the robot's right.
  // Left-side leans use the opposite sign.

  p = p < 0 ? 0 : (p > 1 ? 1 : p);

  const a = action14;
  for (let i = 0; i < 14; i++) a[i] = 0;

  const pose = state.pose;
  const cmd = state.cmd;
  pose.liftY = 0;
  pose.pitchFall = 0;
  pose.rollFall = 0;
  cmd.mouthOpen = 0;

  // ---------- helpers ----------
  function clamp01(t) { return t < 0 ? 0 : (t > 1 ? 1 : t); }
  // smoothstep: zero velocity at both ends
  function ss(e0, e1, x) {
    const t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3 - 2 * t);
  }
  // ease-out cubic: fast start, soft arrival (whip)
  function eo(e0, e1, x) {
    const t = 1 - clamp01((x - e0) / (e1 - e0));
    return 1 - t * t * t;
  }
  // rise a->b, hold, fall c->d
  function pulse(r0, r1, f0, f1, x) {
    return ss(r0, r1, x) * (1 - ss(f0, f1, x));
  }
  // set both legs symmetric (hip pitch, knee, ankle)
  function bothLegs(hip, knee, ankle) {
    a[2] = hip;  a[3] = knee;  a[4] = ankle;
    a[11] = hip; a[12] = knee; a[13] = ankle;
  }

  // ---------- kick (s = +1 left kick, -1 right kick) ----------
  function kick(s) {
    const sw = s > 0 ? 0 : 9;   // swing leg base index
    const sp = s > 0 ? 9 : 0;   // support leg base index

    // phases: 0-.25 weight shift | .25-.45 windup | .45-.56 whip | .56-.68 hold | .68-1 return
    const lean = pulse(0.0, 0.25, 0.78, 1.0, p);
    const wind = ss(0.25, 0.45, p);
    const whip = eo(0.45, 0.56, p);
    const back = ss(0.68, 1.0, p);

    // swing leg: back-cock, then snap forward, then recover
    const hip = 0.45 * wind - 1.45 * whip + 1.0 * back;
    const knee = 0.9 * wind - 0.8 * whip - 0.1 * back;  // knee snaps straight at impact
    const toe = -0.3 * whip * (1 - ss(0.68, 0.9, p));

    a[sw + 1] = s * 0.08 * lean;   // slight clearance abduction
    a[sw + 2] = hip;
    a[sw + 3] = knee;
    a[sw + 4] = toe;

    // support leg: carries the weight, cushions the impact
    const cushion = lean * (0.6 + 0.4 * pulse(0.4, 0.5, 0.6, 0.8, p));
    a[sp + 1] = s * 0.15 * lean;
    a[sp + 2] = -0.07 * cushion;
    a[sp + 3] = 0.15 * cushion;
    a[sp + 4] = -0.07 * cushion;

    // torso and head
    pose.rollFall = s * 0.12 * lean;
    pose.pitchFall = -0.06 * pulse(0.4, 0.52, 0.62, 0.8, p);  // counter-lean on whip
    a[8] = -s * 0.08 * lean;                                  // keep head level
    a[6] = 0.1 * pulse(0.4, 0.52, 0.62, 0.8, p);
  }

  switch (name) {
    case 'kick_left':
      kick(1);
      break;

    case 'kick_right':
      kick(-1);
      break;

    case 'flamingo': {
      const shift = pulse(0.0, 0.2, 0.82, 0.97, p);    // weight onto right foot
      const lift = pulse(0.15, 0.38, 0.72, 0.92, p);   // left knee up, then down gently
      // subtle balance sway during hold
      const sway = Math.sin(p * Math.PI * 6) * 0.012 * lift;

      pose.rollFall = 0.1 * shift + sway;
      a[10] = 0.15 * shift;                // right hip roll
      a[8] = -0.06 * shift - sway * 2;     // head stays level

      a[2] = -0.7 * lift;                  // left hip pitch
      a[3] = 1.1 * lift;                   // left knee
      a[4] = 0.2 * lift;                   // toe relaxed
      a[1] = -0.04 * lift;

      // right leg: support, absorbs touchdown
      const touch = Math.sin(Math.PI * clamp01((p - 0.82) / 0.16)) * (p > 0.82 ? 1 : 0);
      const sup = 0.5 * shift;
      a[11] = -0.05 * sup - 0.04 * touch;
      a[12] = 0.1 * sup + 0.1 * touch;
      a[13] = -0.05 * sup - 0.04 * touch;
      a[6] = 0.05 * lift;
      break;
    }

    case 'polite_bow': {
      const cush = pulse(0.0, 0.2, 0.8, 1.0, p);    // knees cushion
      const bow = pulse(0.15, 0.45, 0.62, 0.9, p); // neck and head fold forward
      bothLegs(-0.1 * cush, 0.2 * cush, -0.1 * cush);
      a[5] = 0.7 * bow;
      a[6] = 0.45 * bow;
      pose.pitchFall = 0.04 * bow;
      cmd.mouthOpen = 0.25 * pulse(0.4, 0.5, 0.6, 0.75, p);
      break;
    }

    case 'ground_pick': {
      const squat = pulse(0.0, 0.3, 0.7, 1.0, p);
      const neck = pulse(0.2, 0.45, 0.7, 0.92, p);
      pose.liftY = -0.04 * squat;
      bothLegs(-0.35 * squat, 0.7 * squat, -0.35 * squat);
      pose.pitchFall = 0.06 * squat;
      a[5] = 0.9 * neck;
      a[6] = 0.5 * neck;
      // mouth snaps open at p=0.5, then clamps shut
      cmd.mouthOpen = ss(0.46, 0.5, p) * (1 - ss(0.5, 0.56, p));
      break;
    }

    case 'roulade': {
      // spec-driven root motion
      pose.liftY = 0.22 * Math.sin(Math.PI * p);
      pose.pitchFall = p * 2 * Math.PI;

      const crouch = ss(0.0, 0.15, p) * (1 - ss(0.15, 0.24, p)); // spring load, then release
      const tuck = ss(0.2, 0.36, p) * (1 - ss(0.72, 0.85, p));   // knees in midair
      const reach = ss(0.72, 0.85, p) * (1 - ss(0.92, 1.0, p));  // legs extended for landing

      const hip = -0.5 * crouch - 1.4 * tuck - 0.25 * reach;
      const knee = 1.0 * crouch + 2.0 * tuck + 0.08 * reach;
      const ankle = -0.5 * crouch + 0.5 * tuck - 0.1 * reach;
      bothLegs(hip, knee, ankle);

      // chin tucked during roll, head lifts for landing
      a[5] = 0.3 * crouch + 0.6 * tuck;
      a[6] = 0.2 * crouch + 0.4 * tuck - 0.15 * reach;
      break;
    }

    default:
      break;
  }
}
```