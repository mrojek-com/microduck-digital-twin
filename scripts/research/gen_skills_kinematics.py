#!/usr/bin/env python3
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / "tools"))
from claude_bridge import query_claude

prompt = """You are Claude Endgegner: Master Robotics Animator & Kinematician.

Write a high-performance JavaScript function `applySkillAnimation(name, p, action14, state)` for the 15-DOF MicroDuck bipedal robot digital twin.
`p` is normalized progress from 0.0 to 1.0.

The 14 action array slots:
- 0: left_hip_yaw, 1: left_hip_roll, 2: left_hip_pitch, 3: left_knee, 4: left_ankle
- 5: neck_pitch, 6: head_pitch, 7: head_yaw, 8: head_roll
- 9: right_hip_yaw, 10: right_hip_roll, 11: right_hip_pitch, 12: right_knee, 13: right_ankle

Robot root pose:
- state.pose.liftY (meters, positive lifts robot up so roulade/somersault does not penetrate floor!)
- state.pose.pitchFall (radians, forward/backward tumble)
- state.pose.rollFall (radians, lateral body lean)
- state.cmd.mouthOpen (0.0 to 1.0)

Design fluid, physically grounded, expressive animations for all 6 skills:
1. kick_left: Weight transfer (lean body right with rollFall and right hip_roll) -> windup left leg back -> rapid whip kick forward -> return.
2. kick_right: Mirror of kick_left (lean body left, whip right leg forward).
3. flamingo: Shift weight onto right foot (rollFall and right hip_roll) -> lift left knee up (pitch -0.7, knee 1.1) -> hold gracefully -> gentle touchdown.
4. polite_bow: Cushion knees slightly -> bow neck and head forward (neck_pitch 0.7, head_pitch 0.45) -> slight mouth open -> return.
5. ground_pick: Squat down (liftY dips -0.04m, knees bend) -> neck pitches down -> mouth snaps open at p=0.5 then clamps shut -> stand back up.
6. roulade (Somersault): Spring crouch at p=0..0.15 -> jump & tumble forward (liftY = 0.22 * Math.sin(Math.PI * p), pitchFall = p * 2 * Math.PI) -> tuck knees in midair -> extend legs for landing at p=0.85..1.0.

Output ONLY valid JavaScript code for function applySkillAnimation(name, p, action14, state) { ... }.
"""

print("Calling Claude Sonnet 5.5...")
res = query_claude(prompt, model="sonnet", max_tokens=8192)
code = res.get("text", "")
print(f"Generated {len(code)} characters.")

out_file = Path(__file__).resolve().parents[1] / "docs" / "SKILLS_KINEMATICS_FIX.md"
out_file.write_text(code, encoding="utf-8")
print(f"Saved to {out_file}")
