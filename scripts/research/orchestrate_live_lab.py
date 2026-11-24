#!/usr/bin/env python3
"""
apps/MicroDuck/src/orchestrate_live_lab.py

Orchestrates the local Tri-Council (Gemma 4, Qwen 3 MoE) and Claude Endgegner
to design a mind-blowing Live In-Browser Python / Code Experimentation Lab:
- Allows the user to paste Python (or natural language prompts) directly into a textarea in the simulator.
- Runs it in real-time on the 15-DOF MicroDuck digital twin.
- Connects local Ollama (Qwen 3 MoE) and Claude to generate motion routines and test new firmware ideas live.
"""

import json
import subprocess
import sys
import urllib.request
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
WORKSPACE_ROOT = PROJECT_ROOT.parent.parent
DOCS_DIR = PROJECT_ROOT / "docs"
OLLAMA_URL = "http://localhost:11434/api/generate"


def evict_ollama() -> None:
    payload = json.dumps({"model": "", "keep_alive": 0}).encode("utf-8")
    req = urllib.request.Request(OLLAMA_URL, data=payload, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5):
            pass
    except Exception:
        pass


def query_ollama(model: str, prompt: str) -> str:
    evict_ollama()
    body = {
        "model": model,
        "prompt": prompt,
        "stream": False,
        "think": False,
        "keep_alive": 0,
        "options": {
            "num_ctx": 4096,
            "num_predict": 1200,
            "temperature": 0.25,
        },
    }
    req = urllib.request.Request(
        OLLAMA_URL,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=300) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            evict_ollama()
            res = data.get("response", "").strip()
            if "</think>" in res:
                res = res.split("</think>", 1)[1].strip()
            return res
    except Exception as e:
        evict_ollama()
        return f"[Ollama Error: {e}]"


def run_claude_endgegner(context: str) -> str:
    bridge_script = WORKSPACE_ROOT / "tools" / "claude_bridge.py"
    prompt = (
        "You are the Claude Endgegner: Frontier Mastermind & Principal Robotics Simulation Architect.\\n\\n"
        "The user asked:\\n"
        "'but how does it work now.... use the local llms? lets say i want to test something or upgrade something how can i test it in the simulator? "
        "is there also a way to paste python or whatever into a input/textarea to test something? or is it naive to think that? "
        "use the endgegner model.... local llms and claude to amaze me'\\n\\n"
        "Here is the local Tri-Council's technical proposal:\\n"
        f"{context}\\n\\n"
        "Your task is to AMAZE the user with an uncompromising, breathtaking architecture and concrete implementation design:\\n"
        "1. Confirm why this is NOT naive at all, but rather the frontier standard for robotics digital twins (Live Code Sandbox + Local AI Copilot).\\n"
        "2. Detail the exact dual-execution engine: In-browser Pyodide (real WebAssembly Python) + ultra-fast zero-latency JS Kinematics Sandbox + Live Ollama AI Generator.\\n"
        "3. Provide the exact API/sandbox contract exposed to user code (e.g. `duck.set_joints()`, `duck.walk()`, `duck.beak()`, `duck.step()`, `duck.read_tof()`).\\n"
        "4. Provide 3 killer ready-to-run motion/behavior presets that demonstrate the power:\\n"
        "   - Preset A: 'Drunken Sailor Gait' (asymmetric limp + dynamic balance wobble)\\n"
        "   - Preset B: 'ToF Theremin Beatbox' (beak opens & head tracks hand distance)\\n"
        "   - Preset C: 'Impact Guard Self-Righting' (detects tip-over and activates emergency push recovery)\\n"
        "5. Provide concrete implementation patches for web/index.html, web/styles.css, and web/simulator.js.\\n"
    )
    cmd = [sys.executable, str(bridge_script), "prompt", prompt, "--model", "sonnet"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return proc.stdout.strip()
    except subprocess.CalledProcessError as e:
        return f"[Claude Bridge Error: {e.stderr or e.stdout}]"


def main() -> None:
    print("=== [1/3] Gemma 4: Live Code Execution Sandbox Architecture ===")
    prompt_gemma = (
        "How should a browser-based 50 Hz robotics digital twin (Three.js + 15-DOF kinematics) "
        "enable a user to write and execute live Python or custom logic from a textarea? "
        "Detail the runtime architecture: Pyodide WASM vs JS sandbox, safety sandboxing, exposed duck API, "
        "and integration with local Ollama for natural-language-to-motion synthesis."
    )
    gemma_out = query_ollama("gemma4:26b", prompt_gemma)
    print(f"Gemma 4 done ({len(gemma_out)} chars)")

    print("=== [2/3] Qwen 3 MoE: AST & Interactive Editor Implementation ===")
    prompt_qwen = (
        f"Based on Gemma 4's architecture:\\n{gemma_out}\\n\\n"
        "Synthesize the concrete front-end components: "
        "1. A collapsible 'Live Motion Lab & Python REPL' modal/drawer in web/index.html with textarea, Run/Stop, Presets, and 'Ask Local AI' prompt bar. "
        "2. The JS execution wrapper that calls the user script at 50 Hz (`onStep(t, dt, duck)`) with zero GC overhead. "
        "3. Error boundary that catches exceptions and highlights the error in the console without crashing the simulator."
    )
    qwen_out = query_ollama("qwen3:30b-a3b", prompt_qwen)
    print(f"Qwen 3 MoE done ({len(qwen_out)} chars)")

    council_context = f"### Gemma 4 Architecture:\n{gemma_out}\n\n### Qwen 3 MoE Implementation:\n{qwen_out}"

    print("=== [3/3] Claude Endgegner: The Frontier Mastermind Breakthrough ===")
    claude_out = run_claude_endgegner(council_context)
    print(f"Claude Endgegner done ({len(claude_out)} chars)")

    report = (
        "# 🦆 MicroDuck Live Code Sandbox & AI Motion Copilot — Architecture & Directives\n\n"
        f"{council_context}\n\n"
        f"## 👑 Claude Endgegner Verdict & Implementation\n\n{claude_out}\n"
    )
    out_file = DOCS_DIR / "LIVE_CODE_SANDBOX_DIRECTIVES.md"
    out_file.write_text(report, encoding="utf-8")
    print(f"[✓] Architecture saved to {out_file}")


if __name__ == "__main__":
    main()
