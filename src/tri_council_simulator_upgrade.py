#!/usr/bin/env python3
"""
apps/MicroDuck/src/tri_council_simulator_upgrade.py

Orchestrates the Local Tri-Council Swarm (Gemma 4, Qwen 3 MoE, DeepSeek-R1)
and the Claude Endgegner Bridge to elevate the MicroDuck WebGL Simulator:
1. Refines the awkward title ("MicroDuck Alpha 3D Digital Twin") to a sleek,
   authoritative "MicroDuck 50 Hz Digital Twin" with refined typography & badges.
2. Identifies UI/UX, interaction, visual shader, and telemetry improvements.
3. Captures Claude's definitive "Finishing Touch" directives.
4. Outputs the unified upgrade spec to `docs/TRI_COUNCIL_UPGRADE_SPEC.md`.

Enforces Apple Silicon 32 GB Unified Memory invariants:
- Sequential model execution with `keep_alive: 0`
- `think: false` for Qwen 3 MoE
- `num_ctx: 4096`, `num_predict: 1400`
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
            "temperature": 0.2,
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


def run_claude_finishing_touch(context_text: str) -> str:
    bridge_script = WORKSPACE_ROOT / "tools" / "claude_bridge.py"
    prompt = (
        "You are the Claude Endgegner (Frontier Mastermind & Lead UX/Graphics Architect). "
        "The user requested: 'orchestrate the local llms and claude to improve the microduck simulator..... "
        "the title in the simulator is tsrange MicroDuck Alpha 3D Digital Twin.... its ok when claude gives its finishing touch....'\\n\\n"
        f"Here is the local Tri-Council evaluation and proposed design:\\n{context_text}\\n\\n"
        "Deliver your definitive FINISHING TOUCH directives:\\n"
        "1. Exact Title & Subtitle phrasing (eliminate 'Alpha 3D' or clunky words; make it sleek and premium).\\n"
        "2. Exact UI Header & Badging tweaks (CSS/HTML).\\n"
        "3. Top 3 aesthetic & interaction micro-polish touches (e.g. subtle shadow bloom, smoother camera damping, button states, telemetry layout).\\n"
        "4. Precise code patch recommendations for web/index.html, web/styles.css, and web/simulator.js.\\n"
        "Be concise, surgical, and authoritative."
    )
    cmd = [sys.executable, str(bridge_script), "prompt", prompt, "--model", "sonnet"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return proc.stdout.strip()
    except subprocess.CalledProcessError as e:
        return f"[Claude Bridge Error: {e.stderr or e.stdout}]"


def main() -> None:
    print("=== [Phase 1/4] Gemma 4 (26B): Structural UX & Title Analysis ===")
    prompt_gemma = (
        "Analyze the current MicroDuck WebGL simulator title and topbar UX. "
        "Currently it has titles like 'MicroDuck Alpha 3D Digital Twin' or 'MicroDuck Alpha Digital Twin' "
        "with badges '50 Hz · obs[1,61] → act[1,14] · 15× XL330'. "
        "What is awkward about this title? Recommend the ideal, professional title, subtitle, and badge hierarchy "
        "for an authentic bipedal robotics digital twin."
    )
    gemma_out = query_ollama("gemma4:26b", prompt_gemma)
    print(f"Gemma 4 response length: {len(gemma_out)}")

    print("=== [Phase 2/4] Qwen 3 MoE (30.5B): Front-End & Visual Shader Proposals ===")
    prompt_qwen = (
        "Based on Gemma 4's architectural critique:\\n"
        f"{gemma_out}\\n\\n"
        "Formulate specific HTML, CSS, and Three.js visual improvements for the simulator: "
        "1. Modern header styling (glassmorphism topbar, clean typography, badge layout). "
        "2. Viewport HUD chips: clean alignment, hover polish, non-overlapping controls. "
        "3. Three.js ground grid, lighting sheen, and DoubleSide rendering polish. "
        "Give concrete CSS properties and HTML markup."
    )
    qwen_out = query_ollama("qwen3:30b-a3b", prompt_qwen)
    print(f"Qwen 3 MoE response length: {len(qwen_out)}")

    print("=== [Phase 3/4] DeepSeek-R1 (14.8B): Adversarial Usability & Robustness Audit ===")
    prompt_r1 = (
        "Review the proposed UI and header updates from Gemma 4 and Qwen 3:\\n"
        f"{qwen_out}\\n\\n"
        "Audit for: layout shift regressions on mobile/compact viewports, button event collisions, "
        "and WebGL canvas resize behavior. List any potential pitfalls and confirm the safest implementation."
    )
    r1_out = query_ollama("deepseek-r1:14b", prompt_r1)
    print(f"DeepSeek-R1 response length: {len(r1_out)}")

    council_summary = (
        f"### Gemma 4 (Structural UX & Title Analysis)\n{gemma_out}\n\n"
        f"### Qwen 3 MoE (Front-End & Shader Directives)\n{qwen_out}\n\n"
        f"### DeepSeek-R1 (Adversarial Usability & Stability Audit)\n{r1_out}\n"
    )

    print("=== [Phase 4/4] Claude Endgegner: The Finishing Touch ===")
    claude_touch = run_claude_finishing_touch(council_summary)
    print(f"Claude Finishing Touch response length: {len(claude_touch)}")

    report = (
        "# 🦆 MicroDuck Simulator — Tri-Council & Claude Upgrade Specification\n\n"
        "## 🏛️ Local Tri-Council Swarm Deliberation\n\n"
        f"{council_summary}\n\n"
        "## 👑 Claude Endgegner: The Finishing Touch\n\n"
        f"{claude_touch}\n"
    )

    out_file = DOCS_DIR / "TRI_COUNCIL_UPGRADE_SPEC.md"
    out_file.write_text(report, encoding="utf-8")
    print(f"[✓] Upgrade specification saved to {out_file}")


if __name__ == "__main__":
    main()
