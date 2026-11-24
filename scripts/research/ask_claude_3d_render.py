#!/usr/bin/env python3
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / "tools"))
from claude_bridge import query_claude

prompt = """You are Claude Endgegner: World-Class Three.js / WebGL Graphics Architect & Technical Artist.

The user asked: "claude can you imorove the 3d rendering itself?"

In the 15-DOF MicroDuck digital twin (`web/simulator.js`):
Currently, the 3D scene uses basic Three.js lighting and generic material properties on the 59-part CAD rig (`duck.bin`).
We want to dramatically elevate the visual fidelity to look like a high-end AAA industrial product / robotics studio:

1. Procedural Studio Environment Map / PBR IBL (Image-Based Lighting):
   - Generate a procedural high-dynamic-range studio cubemap / equirectangular canvas texture (warm key softbox, cool rim softbox, neutral fill) and set `scene.environment = envTexture`.
   - This creates gorgeous specular reflections on the injection-molded duck plastic and anodized aluminum servo horns!

2. Authentic Multi-Material PBR Shading per Part:
   - Body & Head Shells: High-grade injection molded ABS (`roughness: 0.32, metalness: 0.04, clearcoat: 0.3, clearcoatRoughness: 0.25`).
   - Beak (Mouth & Jaw): Satin TPU / matte polymer (`roughness: 0.45, metalness: 0.0`).
   - Servos & Skeleton Brackets: Anodized matte aluminum (`roughness: 0.26, metalness: 0.78`).
   - Roller-Skate Wheels: Polyurethane tread with subtle specular sheen (`roughness: 0.18, metalness: 0.15`).

3. Ground & Soft Contact Shadow Decal:
   - Replace the crude hard black circle with a smooth radial gradient Gaussian contact shadow canvas texture that tracks the robot root and dynamically scales with foot distance/altitude.
   - Studio floor: dark slate radial vignette with subtle hexagonal/grid lines and soft specular reflection.

4. Cinematic Studio Lighting & ACES Tone Mapping:
   - 3-point key/fill/rim balance with warm key (0xfff5e6) and cyan/lavender rim (0x7dd3fc).
   - Tone mapping exposure, shadow bias, and soft PCF shadow filtering.

Provide the exact, complete JavaScript code patches for `web/simulator.js` that implement these visual upgrades cleanly.
"""

print("[1/2] Calling Claude Sonnet 5.5 for 3D Rendering Overhaul...")
res = query_claude(prompt, model="sonnet", max_tokens=5000)
code = res.get("text", "")
print(f"[2/2] Generated {len(code)} characters.")

out_file = Path(__file__).resolve().parents[1] / "docs" / "CLAUDE_3D_RENDERING_SPEC.md"
out_file.write_text(code, encoding="utf-8")
print(f"[✓] Saved to {out_file}")
