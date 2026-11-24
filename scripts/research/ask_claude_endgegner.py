#!/usr/bin/env python3
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / "tools"))
from claude_bridge import query_claude

context_file = Path(__file__).resolve().parents[1] / "docs" / "LIVE_CODE_SANDBOX_DIRECTIVES.md"
context = context_file.read_text(encoding="utf-8")

prompt = f"""You are the Claude Endgegner: Frontier Mastermind & Principal Robotics Simulation Architect.

The user asked:
"but how does it work now.... use the local llms? lets say i want to test something or upgrade something how can i test it in the simulator? is there also a way to paste python or whatever into a input/textarea to test something? or is it naive to think that? use the endgegner model.... local llms and claude to amaze me"

Here is the local Tri-Council deliberation (Gemma 4 & Qwen 3 MoE):
{context}

Now deliver your definitive Endgegner Verdict and Blueprint:
1. Explain passionately and clearly why this is NOT naive at all, but the frontier standard for modern robotics simulation (interactive in-browser live code + LLM copilot).
2. Outline the execution engine:
   - Live Python-like & JavaScript kinematic execution loop at 50 Hz (`onStep(t, dt, duck)`)
   - Natural Language Prompt Bar connecting directly to local Ollama (e.g. qwen3:30b-a3b) or Claude with 1-click execution.
3. Define the exact, tactile, ultra-clean `duck` API exposed to the user:
   - `duck.setMouth(ratio)`, `duck.setJoint(name, angle_rad)`, `duck.setTwist(vx, vyaw)`, `duck.setHead(yaw, pitch, roll)`
   - `duck.readTof()`, `duck.playQuack()`, `duck.applyImpulse(rad_s)`
4. Provide 3 killer motion presets ready to paste and run:
   - Preset 1: Drunken Sailor Wobble (asymmetric gait with dynamic balance compensation)
   - Preset 2: ToF Theremin DJ / Beatbox (mouth & head synced to ToF distance + procedural vocal chirps)
   - Preset 3: Self-Righting Acrobat (detects fall angle and executes coordinated leg tuck & stand recovery)
5. Provide the exact HTML/CSS/JS patch for the simulator.
"""

print("[1/2] Calling Claude Sonnet 5.5 Endgegner...")
res = query_claude(prompt, model="sonnet", max_tokens=3500)
output_text = res.get("text", "")
print(f"[2/2] Received {len(output_text)} characters from Claude.")

with open(context_file, "a", encoding="utf-8") as f:
    f.write("\n\n## 👑 Claude Endgegner: The Frontier Mastermind Breakthrough\n\n" + output_text)

print("[✓] Appended Claude's response to docs/LIVE_CODE_SANDBOX_DIRECTIVES.md")
