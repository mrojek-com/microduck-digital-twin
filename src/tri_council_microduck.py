#!/usr/bin/env python3
"""
apps/MicroDuck/src/tri_council_microduck.py

Sequential Local Ollama Tri-Council Codebase, Cheatsheet & Innovation Investigator
for `pollen-robotics/microduck`.

Strictly enforces Apple Silicon 32 GB Unified Memory invariants:
- Sequential model execution (never concurrent)
- keep_alive: 0 on every request (immediate Metal VRAM eviction)
- think: false at top level (INV-020 for qwen3:30b-a3b)
- num_ctx: 4096, num_predict: 1400
"""

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Dict, List
import urllib.error
import urllib.request

OLLAMA_GENERATE_URL = "http://localhost:11434/api/generate"
PROJECT_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = PROJECT_ROOT / "repo"
DOCS_DIR = PROJECT_ROOT / "docs"


def read_excerpt(rel_path: str, max_chars: int = 2400) -> str:
    p = REPO_ROOT / rel_path
    if not p.exists():
        return f"[Missing {rel_path}]"
    text = p.read_text(encoding="utf-8", errors="replace")
    if len(text) <= max_chars:
        return text
    return text[:max_chars] + "\n...[truncated]..."


def evict_ollama_vram(url: str = OLLAMA_GENERATE_URL) -> None:
    """Explicitly unload any resident model from Apple Silicon Unified Memory."""
    payload = json.dumps({"model": "", "keep_alive": 0}).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5):
            pass
    except Exception:
        pass


def _strip_think_tags(text: str) -> str:
    if "</think>" in text:
        after = text.split("</think>", 1)[1].strip()
        if after:
            return after
    return text.replace("<think>", "").replace("</think>", "").strip()


def query_ollama_sequential(
    model: str,
    prompt: str,
    url: str = OLLAMA_GENERATE_URL,
    timeout_seconds: int = 300,
    num_predict: int = 2400,
) -> str:
    """Query one local Ollama model sequentially with keep_alive=0 and think=false."""
    body: Dict[str, Any] = {
        "model": model,
        "prompt": prompt,
        "stream": False,
        "think": False,
        "keep_alive": 0,
        "options": {
            "num_ctx": 4096,
            "num_predict": num_predict,
            "temperature": 0.25,
        },
    }
    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout_seconds) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            text = _strip_think_tags((data.get("response") or "").strip())
            if not text and data.get("thinking"):
                text = _strip_think_tags(str(data.get("thinking")).strip())
            return text or "[Empty response returned by model]"
    except urllib.error.URLError as exc:
        return f"[Ollama connection error for {model}: {exc}]"
    except Exception as exc:
        return f"[Inference error for {model}: {exc}]"
    finally:
        evict_ollama_vram(url)


def build_council_jobs() -> List[Dict[str, str]]:
    arch_excerpt = read_excerpt("docs/design/architecture.md", 3200)
    cheat_excerpt = read_excerpt("docs/robot/cheatsheet.md", 3600)
    cheat_dev_excerpt = read_excerpt("docs/robot/cheatsheet-dev.md", 1800)
    sim_excerpt = read_excerpt("docs/robot/simulation.md", 2000)
    obs_excerpt = read_excerpt("duck-control/src/obs.rs", 2500)
    model_excerpt = read_excerpt("duck-control/src/model.rs", 2200)
    safety_excerpt = read_excerpt("duck-control/src/safety.rs", 1800)
    pickup_excerpt = read_excerpt("duck-control/src/pickup.rs", 1800)
    kin_excerpt = read_excerpt("kinematics/src/lib.rs", 1800)
    synth_excerpt = read_excerpt("sounds/src/synth.rs", 1800)
    autod_excerpt = read_excerpt("autod/src/brain.rs", 1800)

    return [
        {
            "id": "gemma_arch",
            "model": "gemma4:26b",
            "role": "Structural & Daemon Systems Architect",
            "prompt": (
                "You are analyzing the `pollen-robotics/microduck` Rust workspace.\n\n"
                f"=== architecture.md ===\n{arch_excerpt}\n\n"
                f"=== cheatsheet-dev.md ===\n{cheat_dev_excerpt}\n\n"
                f"=== simulation.md ===\n{sim_excerpt}\n\n"
                "Provide a deep technical analysis covering:\n"
                "1. The multi-daemon Unix socket JSON-RPC 2.0 NDJSON architecture (`robotd`, `configd`, `updaterd`, `btd`, `padd`, `netpadd`, `autod`, `mediad`, `tofd`, `mapd`) and why `configd`/`updaterd`/`btd` form a zero-dependency recovery triad.\n"
                "2. Two-layer `/etc/robot/robotd.toml` vs `/var/lib/robot/robotd.toml` state ownership for A/B rootfs OS updates.\n"
                "3. How `scripts/duck-sim` creates a full multi-duck twin against MuJoCo and why real-time factor < 1.0x triggers the 45 Hz health gate rollback."
            ),
        },
        {
            "id": "mistral_quant",
            "model": "mistral-small:24b",
            "role": "Quantitative Control, Kinematics & DSP Strategist",
            "prompt": (
                "You are analyzing Microduck's 50 Hz RL control loop, kinematics, and 48 kHz audio synthesis.\n\n"
                f"=== duck-control/src/obs.rs ===\n{obs_excerpt}\n\n"
                f"=== duck-control/src/model.rs ===\n{model_excerpt}\n\n"
                f"=== kinematics/src/lib.rs ===\n{kin_excerpt}\n\n"
                f"=== sounds/src/synth.rs ===\n{synth_excerpt}\n\n"
                "Derive and explain concisely:\n"
                "1. The exact 61-D observation vector layout (`obs[1,61] -> actions[1,14]`), why mouth index 9 is excluded, and why `[body.z, body.roll, body.pitch]` ordering matters.\n"
                "2. How `kinematics` flattens MJCF root-to-site chains for zero-allocation 50 Hz FK (`left_foot`, `right_foot`, `tof`, `head_camera`).\n"
                "3. Why `sounds` synthesizes at 48 kHz (Radxa I2S clock tree) and rescales time-domain DSP constants from 22.05 kHz, plus how the BLE Chorale achieves ±20 ms ensemble sync without NTP."
            ),
        },
        {
            "id": "qwen_innovate",
            "model": "qwen3:30b-a3b",
            "role": "Master Implementer & Product Innovator (What Else Is Possible)",
            "prompt": (
                "Output ONLY the final numbered Markdown report (no planning notes or preamble).\n\n"
                "Based on `pollen-robotics/microduck`'s capabilities:\n"
                f"=== cheatsheet.md (excerpt) ===\n{cheat_excerpt}\n\n"
                f"=== autod/src/brain.rs ===\n{autod_excerpt}\n\n"
                "Propose exactly 10 high-leverage, 'deeper not bigger' engineering projects and applications we can build with MicroDuck using its existing JSON-RPC sockets, rendezvous control lane, ONNX runtime, 8x8 ToF, WebRTC camera, BLE GATT/Chorale, NFC tags, and 48 kHz voice synth.\n"
                "For each of items 1..10, concisely provide: (a) Concrete Codebase Hook, (b) Technical Architecture, and (c) Extraordinary Capability Unlocked."
            ),
        },
        {
            "id": "deepseek_audit",
            "model": "deepseek-r1:14b",
            "role": "Adversarial & Forensic Systems Auditor",
            "prompt": (
                "Perform a forensic safety & failure-mode audit of MicroDuck's control and safety subsystems.\n\n"
                f"=== duck-control/src/safety.rs ===\n{safety_excerpt}\n\n"
                f"=== duck-control/src/pickup.rs ===\n{pickup_excerpt}\n\n"
                f"=== cheatsheet.md (Impact Guard & Monitoring) ===\n{cheat_excerpt}\n\n"
                "Identify and explain the 6 sharpest failure modes and edge cases documented or implicit in the code:\n"
                "1. Why ROBOTIS Impact Guard Factory Level 5 causes stance-knee collapse (~6 trips/sec) while Setting R succeeds.\n"
                "2. Silent observation-index drift in `obs.rs` and why `Safety` only enforces actuator `[-pi, pi]` rather than per-joint MJCF limits.\n"
                "3. Deadman stop-vs-limp invariant and `fall_limp` / `FallPredictor` interplay.\n"
                "4. Pickup detector (`pickup.rs`) v1 -> v2 edge cases (head grips, 180 deg inversions, sim-to-real current discrepancy).\n"
                "5. VL53L8CX ToF status codes (`5,9` vs `4,13` consistency failed) causing 30 cm theremin cutoff.\n"
                "6. `duck-sim` wall-clock <1.0x real-time triggering the 45 Hz health gate and false update rollbacks."
            ),
        },
        {
            "id": "gemma_cheatsheet",
            "model": "gemma4:26b",
            "role": "Operations & Cheatsheet Distiller",
            "prompt": (
                f"Distill `docs/robot/cheatsheet.md` and `docs/robot/cheatsheet-dev.md` into a crisp, high-signal Operator & Developer Master Cheatsheet.\n\n"
                f"=== cheatsheet.md ===\n{cheat_excerpt}\n\n"
                f"=== cheatsheet-dev.md ===\n{cheat_dev_excerpt}\n\n"
                "Organize cleanly into Markdown tables & command blocks covering: Live Monitoring (`robotctl monitor`), Configuration Layers, Policies & Skills (`load`, `add`, `pad bind`), Gamepad Button & D-Pad Modes, Audio/Chorale/Theremin, Laser & Soccer & `autod`, Power/Servo Auto-Adoption, and Updates/Rescue/Dev-Push."
            ),
        },
    ]


def run_investigation() -> None:
    DOCS_DIR.mkdir(parents=True, exist_ok=True)
    evict_ollama_vram()

    jobs = build_council_jobs()
    results: Dict[str, str] = {}
    for job in jobs:
        print(f"[Tri-Council] Querying {job['model']} ({job['role']})...", flush=True)
        ans = query_ollama_sequential(model=job["model"], prompt=job["prompt"])
        results[job["id"]] = ans
        print(f"[Tri-Council] Completed {job['id']} ({len(ans)} chars).", flush=True)

    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

    report_md = "\n".join(
        [
            "# MicroDuck Deep Codebase & Systems Analysis — Local Tri-Council Report\n",
            f"- **Generated:** {ts}",
            "- **Target Repository:** `https://github.com/pollen-robotics/microduck` (cloned to `apps/MicroDuck/repo`)",
            "- **Execution Law:** Sequential Apple Silicon Metal inference (`keep_alive: 0`, `num_ctx: 4096`, `think: false`)\n",
            "---\n",
            "## 1. Structural & Multi-Daemon Architecture — `gemma4:26b`\n",
            results["gemma_arch"],
            "\n---\n",
            "## 2. Quantitative Control, Kinematics & 48 kHz DSP — `mistral-small:24b`\n",
            results["mistral_quant"],
            "\n---\n",
            "## 3. Forensic Safety & Failure-Mode Audit — `deepseek-r1:14b`\n",
            results["deepseek_audit"],
            "",
        ]
    )
    (DOCS_DIR / "TRI_COUNCIL_MICRODUCK_REPORT.md").write_text(report_md, encoding="utf-8")

    rfc_md = "\n".join(
        [
            "# What Else Is Possible With MicroDuck? — Sovereign Expansion RFC\n",
            f"- **Author Model:** `qwen3:30b-a3b` (Local Master Implementer)",
            f"- **Generated:** {ts}\n",
            "---\n",
            results["qwen_innovate"],
            "",
        ]
    )
    (DOCS_DIR / "WHAT_ELSE_IS_POSSIBLE_RFC.md").write_text(rfc_md, encoding="utf-8")

    cheat_md = "\n".join(
        [
            "# MicroDuck Operator & Developer Master Cheatsheet\n",
            f"- **Compiled By:** `gemma4:26b` from `repo/docs/robot/cheatsheet.md` & `cheatsheet-dev.md`",
            f"- **Generated:** {ts}\n",
            "---\n",
            results["gemma_cheatsheet"],
            "",
        ]
    )
    (DOCS_DIR / "CHEATSHEET_MASTER_GUIDE.md").write_text(cheat_md, encoding="utf-8")
    print("[Tri-Council] All 3 markdown reports written to apps/MicroDuck/docs/.")


if __name__ == "__main__":
    run_investigation()
