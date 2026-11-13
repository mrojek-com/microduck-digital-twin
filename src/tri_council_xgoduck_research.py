#!/usr/bin/env python3
"""
apps/MicroDuck/src/tri_council_xgoduck_research.py

Sequential Local Ollama Tri-Council Hardware & 97-STL Research Pipeline
for `LuwuDynamics/xgoduck_hardware` vs `pollen-robotics/microduck`.

Strictly enforces Apple Silicon 32 GB Unified Memory invariants:
- Sequential model execution (never concurrent)
- keep_alive: 0 on every request (immediate Metal VRAM eviction)
- think: false at top level
- num_ctx: 4096, num_predict: 2400
"""

from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from typing import Any, Dict

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tri_council_microduck import evict_ollama_vram, query_ollama_sequential  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parent.parent
HW_REPO = PROJECT_ROOT / "xgoduck_hardware"
DOCS_DIR = PROJECT_ROOT / "docs"
WEB_ASSETS = PROJECT_ROOT / "web" / "assets"
CATALOG_PATH = WEB_ASSETS / "models" / "xgoduck_stl_catalog.json"


def read_hw_excerpt(rel_path: str, max_chars: int = 2400) -> str:
    p = HW_REPO / rel_path
    if not p.exists():
        return f"[Missing {rel_path}]"
    text = p.read_text(encoding="utf-8", errors="replace")
    return text if len(text) <= max_chars else text[:max_chars] + "\n...[truncated]..."


def summarize_stl_catalog() -> str:
    data: Dict[str, Any] = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
    summary = data["summary"]
    v5_sample = [
        f"- {e['filename']} ({e['category']}): {e['faces']} tris, {e['extents_mm']} mm, watertight={e['watertight']}, ~{e['approx_volume_cm3']} cm3"
        for e in data["v5_audio_video"][:22]
    ]
    v2_sample = [
        f"- {e['filename']} ({e['category']}): {e['faces']} tris, {e['extents_mm']} mm, watertight={e['watertight']}"
        for e in data["v2_standard"][:12]
    ]
    return (
        f"Catalog Summary: {summary['total_stl_count']} total STLs "
        f"({summary['v2_standard_count']} V2_STANDARD [{summary['total_v2_faces']:,} tris], "
        f"{summary['v5_audio_video_count']} V5_AUDIO_VIDEO [{summary['total_v5_faces']:,} tris])\n"
        f"Exports: {json.dumps(data['exports'], indent=2)}\n\n"
        f"Sample V5_AUDIO_VIDEO STLs:\n" + "\n".join(v5_sample) + "\n\n"
        f"Sample V2_STANDARD STLs:\n" + "\n".join(v2_sample)
    )


def run_council() -> None:
    DOCS_DIR.mkdir(parents=True, exist_ok=True)
    evict_ollama_vram()

    stl_summary = summarize_stl_catalog()
    upstream_doc = read_hw_excerpt("UPSTREAM.md", 2200)
    bom_doc = read_hw_excerpt("BOM.md", 2200)
    printing_doc = read_hw_excerpt("docs/PRINTING.md", 2200)
    build_doc = read_hw_excerpt("docs/BUILD_GUIDE.md", 2400)
    v5_readme = read_hw_excerpt("structure/V5_AUDIO_VIDEO_/README.md", 1600)
    readme_doc = read_hw_excerpt("readme.md", 1800)

    print("[Tri-Council Hardware] 1/4 Querying gemma4:26b (Hardware & CAD Topology Architect)...")
    gemma_prompt = (
        "You are gemma4:26b acting as Principal Mechanical & CAD Architect.\n"
        "Analyze the real `LuwuDynamics/xgoduck_hardware` STL repository using ONLY the inspected facts below.\n"
        "Cover:\n"
        "1. V2_STANDARD (47 STLs, XGO_duckV2.3mf) vs V5_AUDIO_VIDEO (50 STLs, xgoduckV5.3mf) structural delta (face camera/light/ToF cutouts, V1 vs V1.1 backpack enclosures).\n"
        "2. Rigid PLA/PETG shells vs TPU flexible parts (`MOUTH_UP_TPU`, `MOUTH_BOTTOM_TPU`, `FOOT_BOTTOM_TPU`) & 9-plate Bambu Studio organization.\n"
        "3. The 10-part Roller Skate add-on (`2x_tire_support`, `4x_TIRE`, `4x_tire_axis`, plus 10x15x3mm bearings & M2x6 screws) and dual bearing system (10x15x3 & 16x22x4 mm).\n"
        "Output ONLY clean concise Markdown sections without preamble.\n\n"
        f"=== STL CATALOG & MEASUREMENTS ===\n{stl_summary}\n\n"
        f"=== PRINTING.md ===\n{printing_doc}\n\n"
        f"=== V5 README ===\n{v5_readme}\n"
    )
    gemma_hw = query_ollama_sequential("gemma4:26b", gemma_prompt, num_predict=2400)
    print(f"[Tri-Council Hardware] Completed gemma4:26b ({len(gemma_hw)} chars).")

    print("[Tri-Council Hardware] 2/4 Querying mistral-small:24b (Actuator, Electronics & Control Comparativist)...")
    mistral_prompt = (
        "You are mistral-small:24b acting as Robotics Electronics & Control Comparator.\n"
        "Compare Pollen Robotics MicroDuck Alpha (`RK3566` + `15x Dynamixel XL330-M288-T` + `BNO085/BMI088` + `VL53L8CX 8x8 ToF`, IDs 11-15/20-24/30-34) "
        "against Luwu Dynamics XGO-Duck (`Arduino UNO Q` Linux+STM32 Bridge + `Luwu V1.1 PCBA` + `15x Feetech HLS1910` + `QMI8658A 6-axis IMU`, IDs 10-14/20-24/30-34, center 2047, 8.4V 2S 18650 >3500mAh >3C).\n"
        "Explain the compute-split difference (`robotd` 8-daemon Rust IPC vs UNO Q Linux->STM32 Bridge), battery/power delivery, fastener/bearing BOM, and PCBA V1.1 specs.\n"
        "Output ONLY a crisp technical comparison Markdown report with tables.\n\n"
        f"=== UPSTREAM.md ===\n{upstream_doc}\n\n"
        f"=== BOM.md & README ===\n{bom_doc}\n\n{readme_doc}\n"
    )
    mistral_hw = query_ollama_sequential("mistral-small:24b", mistral_prompt, num_predict=2400)
    print(f"[Tri-Council Hardware] Completed mistral-small:24b ({len(mistral_hw)} chars).")

    print("[Tri-Council Hardware] 3/4 Querying deepseek-r1:14b (Sim-to-Real & Mechanical Failure Auditor)...")
    deepseek_prompt = (
        "You are deepseek-r1:14b acting as Adversarial Sim-to-Real & Mechanical Failure Auditor.\n"
        "Based strictly on `UPSTREAM.md`, `BUILD_GUIDE.md`, and `PRINTING.md` below, audit EVERY critical failure mode:\n"
        "1. Why upstream MicroDuck ONNX policies (`obs[1,61] -> act[1,14]`) CANNOT be blindly dropped onto XGO-Duck without Rhoban BAM actuator identification (XL330 vs Feetech 1910 friction/armature/latency) and `data/zero_pos.json` calibration.\n"
        "2. Neck/Head assembly trap around IDs 32-34 (pinched cables inside enclosed links, adhesive step p.21, secondary shaft retention on IDs 30-31 vs optional removal on 10/20/32).\n"
        "3. Slicing/tolerance hazards (`2x`/`4x` double-printing trap vs single-instance STL numbering, unspecified PLA/TPU Shore hardness profile, reverse battery polarity / >3C sag, torque-off collapse during calibration).\n"
        "Output ONLY numbered forensic risk items with root cause and mitigation.\n\n"
        f"=== UPSTREAM.md ===\n{upstream_doc}\n\n"
        f"=== BUILD_GUIDE.md ===\n{build_doc}\n"
    )
    deepseek_hw = query_ollama_sequential("deepseek-r1:14b", deepseek_prompt, num_predict=2400)
    print(f"[Tri-Council Hardware] Completed deepseek-r1:14b ({len(deepseek_hw)} chars).")

    print("[Tri-Council Hardware] 4/4 Querying qwen3:30b-a3b (Hardware Mod & Custom STL Innovator)...")
    qwen_prompt = (
        "You are qwen3:30b-a3b acting as Principal Mechatronics & STL Mod Innovator.\n"
        "Propose 8 concrete, buildable custom STL / electronics modifications combining `pollen-robotics/microduck` software (ToF Theremin, BLE Chorale, Pickup Detector, Setting R Impact Guard) with `LuwuDynamics/xgoduck_hardware` (97 STLs, Arduino UNO Q, V5 Audio/Video faceplate, Roller Skates).\n"
        "For each of the 8 proposals give exact STL/software filenames touched, CAD/wiring modification, and user capability unlocked.\n"
        "Output ONLY the final numbered Markdown list (1 through 8) concisely.\n\n"
        f"=== V5 AUDIO/VIDEO HARDWARE SUMMARY ===\n{stl_summary[:2600]}\n"
    )
    qwen_hw = query_ollama_sequential("qwen3:30b-a3b", qwen_prompt, num_predict=2400)
    print(f"[Tri-Council Hardware] Completed qwen3:30b-a3b ({len(qwen_hw)} chars).")

    now_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    report_md = f"""# XGO-Duck (`LuwuDynamics/xgoduck_hardware`) 97-STL & Hardware Research Report

- **Generated:** {now_utc}
- **Hardware Repo Cloned:** `apps/MicroDuck/xgoduck_hardware` (`V2_STANDARD_`: 47 STLs, `V5_AUDIO_VIDEO_`: 50 STLs)
- **Local LLM Council:** Sequential Metal inference (`gemma4:26b`, `mistral-small:24b`, `deepseek-r1:14b`, `qwen3:30b-a3b`) with `keep_alive: 0`

---

## 1. CAD Topology & 97-STL Architecture — `gemma4:26b`

{gemma_hw}

---

## 2. MicroDuck Alpha (`pollen-robotics`) vs XGO-Duck (`LuwuDynamics`) — `mistral-small:24b`

{mistral_hw}

---

## 3. Sim-to-Real & Mechanical Failure Audit — `deepseek-r1:14b`

{deepseek_hw}

---

## 4. Hybrid MicroDuck × XGO-Duck STL & Hardware Mods — `qwen3:30b-a3b`

{qwen_hw}
"""
    out_report = DOCS_DIR / "XGODUCK_HARDWARE_STL_REPORT.md"
    out_report.write_text(report_md, encoding="utf-8")

    # Append STL & XGO-Duck quick-reference section to CHEATSHEET_MASTER_GUIDE.md and sync to web/assets/
    cheat_path = DOCS_DIR / "CHEATSHEET_MASTER_GUIDE.md"
    existing_cheat = cheat_path.read_text(encoding="utf-8") if cheat_path.exists() else ""
    stl_appendix = """
---

## 🖨 XGO-Duck 97-STL Printable Hardware Kit (`LuwuDynamics/xgoduck_hardware`)

| Kit Variant | STL Count | Source `.3mf` | Key Parts & Hardware |
| :--- | :---: | :--- | :--- |
| `V2_STANDARD_` | 47 STLs | `XGO_duckV2.3mf` | Standard biped shells + optional 10-part roller skate kit + V1/V1.1 backpacks |
| `V5_AUDIO_VIDEO_` | 50 STLs | `xgoduckV5.3mf` | Adds `FACE`, `FACE_CAMERA`, `FACE_LIGHT`, `FACE_TOF`, audio/video head shells |
| Flexible `TPU` Parts | 3–4 STLs | Same `.3mf` | `MOUTH_UP_TPU`, `MOUTH_BOTTOM_TPU`, `FOOT_BOTTOM_TPU_01..02` — print in flexible TPU |
| Roller Skates | 10 STLs | Included in V2 & V5 | `2x_tire_support_01..02`, `4x TIRE_01..04`, `4x_tire_axis_01..04` + `8× 10×15×3mm` bearings |

### Critical Print & Assembly Invariants
1. **Single-Instance Rule:** Never multiply STL files containing `2x` or `4x` in the filename — each `.stl` is already exported once per physical instance (`mm` scale, 100%).
2. **Enclosure Choice:** Match `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1.stl` vs `..._FOR_V1P1.stl` to your Luwu expansion PCB revision (`ArduinoUnoQV1.1` recommended).
3. **Feetech 1910 Servo IDs & Zeroing:** Configure one servo at a time: IDs `10–14` (Left Leg), `20–24` (Right Leg), `30–34` (Neck/Head, `34` = Mouth), centered at raw `2047`. Retain secondary shafts ONLY on `30` and `31`. Back up `data/zero_pos.json`.
"""
    if "XGO-Duck 97-STL Printable Hardware Kit" not in existing_cheat:
        updated_cheat = existing_cheat.rstrip() + "\n" + stl_appendix
        cheat_path.write_text(updated_cheat, encoding="utf-8")
    else:
        updated_cheat = existing_cheat

    (WEB_ASSETS / "CHEATSHEET_MASTER_GUIDE.md").write_text(updated_cheat, encoding="utf-8")
    print(f"[✓] Wrote {out_report} and synced {WEB_ASSETS / 'CHEATSHEET_MASTER_GUIDE.md'}.")


if __name__ == "__main__":
    run_council()
