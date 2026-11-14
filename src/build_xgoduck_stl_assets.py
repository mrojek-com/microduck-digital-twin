#!/usr/bin/env python3
"""
apps/MicroDuck/src/build_xgoduck_stl_assets.py

Processes all 97 real CAD `.stl` files from `LuwuDynamics/xgoduck_hardware`:
1. Audits all 47 `V2_STANDARD_` STLs and 50 `V5_AUDIO_VIDEO_` STLs -> `web/assets/models/xgoduck_stl_catalog.json`.
2. Replaces `web/assets/models/microduck_3d_print.stl` with the real articulated Home-Pose CAD assembly (mm slicer scale).
3. Builds a multi-part colored GLB platter `web/assets/models/xgoduck_v5_stl_kit.glb` (and copies to `microduck_flux_ai.glb`)
   from the 12 core `V5_AUDIO_VIDEO_*.stl` printable shells.
4. Exports the real roller-skate wheel `V5_AUDIO_VIDEO_TIRE_01.stl` centered at origin (meters) -> `web/assets/models/xgoduck_v5_tire.json`.
5. Copies key printable `.stl` files into `web/assets/models/stl/` and hardware photos into `web/assets/flux/`.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import trimesh


PROJECT_ROOT = Path(__file__).resolve().parent.parent
HW_REPO = PROJECT_ROOT / "xgoduck_hardware"
MODELS_DIR = PROJECT_ROOT / "web" / "assets" / "models"
STL_EXPORT_DIR = MODELS_DIR / "stl"
FLUX_DIR = PROJECT_ROOT / "web" / "assets" / "flux"


def classify_stl_part(filename: str) -> str:
    upper = filename.upper()
    if "TIRE" in upper:
        return "roller_skates"
    if "TPU" in upper:
        return "tpu_flexible"
    if any(k in upper for k in ("FACE", "HEAD", "EYE", "JAW", "MOUTH", "NECK")):
        return "head_beak"
    if any(k in upper for k in ("THIGH", "SHANK", "FOOT", "ANKLE", "HIP")):
        return "legs_feet"
    return "body_torso"


def scan_stl_directory(stl_dir: Path, kit_name: str) -> List[Dict[str, Any]]:
    entries: List[Dict[str, Any]] = []
    for stl_path in sorted(stl_dir.glob("*.stl")):
        mesh = trimesh.load_mesh(stl_path, force="mesh", process=False)
        extents = [round(float(x), 2) for x in mesh.extents]
        bounds = mesh.bounds
        vol_cm3 = 0.0
        try:
            if mesh.is_watertight:
                vol_cm3 = abs(float(mesh.volume)) / 1000.0
            else:
                vol_cm3 = abs(float(mesh.convex_hull.volume)) / 1000.0
        except Exception:
            vol_cm3 = 0.0
        entries.append(
            {
                "kit": kit_name,
                "filename": stl_path.name,
                "category": classify_stl_part(stl_path.name),
                "size_bytes": stl_path.stat().st_size,
                "vertices": int(len(mesh.vertices)),
                "faces": int(len(mesh.faces)),
                "extents_mm": extents,
                "bounds_min_mm": [round(float(x), 2) for x in bounds[0]],
                "bounds_max_mm": [round(float(x), 2) for x in bounds[1]],
                "watertight": bool(mesh.is_watertight),
                "approx_volume_cm3": round(vol_cm3, 2),
            }
        )
    return entries


def export_real_cad_slicer_stl() -> Dict[str, Any]:
    """Export real articulated CAD Home-Pose duck (`microduck_cad_articulated.glb`) in mm Z-up slicer space."""
    glb_path = MODELS_DIR / "microduck_cad_articulated.glb"
    scene = trimesh.load(glb_path)
    combined = scene.to_mesh() if hasattr(scene, "to_mesh") else scene
    # Convert glTF Y-up (m) -> Slicer Z-up (mm): (x, y, z) -> (x, -z, y) * 1000
    v = combined.vertices.copy()
    v_zup = np.column_stack([v[:, 0], -v[:, 2], v[:, 1]]) * 1000.0
    # Center X/Y at 0 and sit lowest Z vertex cleanly on build plate Z = 0
    cx = 0.5 * (v_zup[:, 0].min() + v_zup[:, 0].max())
    cy = 0.5 * (v_zup[:, 1].min() + v_zup[:, 1].max())
    zmin = v_zup[:, 2].min()
    v_zup[:, 0] -= cx
    v_zup[:, 1] -= cy
    v_zup[:, 2] -= zmin
    stl_mesh = trimesh.Trimesh(vertices=v_zup, faces=combined.faces.copy(), process=False)
    trimesh.repair.fix_normals(stl_mesh)

    out_stl = MODELS_DIR / "microduck_3d_print.stl"
    stl_mesh.export(out_stl)
    return {
        "path": str(out_stl.relative_to(PROJECT_ROOT)),
        "faces": int(len(stl_mesh.faces)),
        "extents_mm": [round(float(x), 2) for x in stl_mesh.extents],
        "size_bytes": out_stl.stat().st_size,
    }


def _decimate_if_huge(mesh: trimesh.Trimesh, max_faces: int = 5000) -> trimesh.Trimesh:
    if len(mesh.faces) <= max_faces:
        return mesh
    step = int(np.ceil(len(mesh.faces) / max_faces))
    return trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces[::step], process=False)


def build_v5_stl_kit_glb() -> Dict[str, Any]:
    """Assemble the 12 hero `V5_AUDIO_VIDEO_*.stl` printable shells into a colored multi-part GLB platter."""
    v5_dir = HW_REPO / "structure" / "V5_AUDIO_VIDEO_"
    zup_to_yup = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    kit_parts: List[Tuple[str, Tuple[int, int, int, int]]] = [
        ("V5_AUDIO_VIDEO_FACE.stl", (244, 241, 234, 255)),
        ("V5_AUDIO_VIDEO_EYE.stl", (250, 204, 21, 255)),
        ("V5_AUDIO_VIDEO_JAW.stl", (249, 115, 22, 255)),
        ("V5_AUDIO_VIDEO_HEAD_BOTTOM.stl", (244, 241, 234, 255)),
        ("V5_AUDIO_VIDEO_BODY_SHELL_LEFT.stl", (181, 232, 247, 255)),
        ("V5_AUDIO_VIDEO_BODY_SHELL_RIGHT.stl", (181, 232, 247, 255)),
        ("V5_AUDIO_VIDEO_THIGH_LEFT_SHELL.stl", (244, 241, 234, 255)),
        ("V5_AUDIO_VIDEO_THIGH_RIGHT_SHELL.stl", (244, 241, 234, 255)),
        ("V5_AUDIO_VIDEO_LEFT_FOOT.stl", (249, 115, 22, 255)),
        ("V5_AUDIO_VIDEO_RIGHT_FOOT.stl", (249, 115, 22, 255)),
        ("V5_AUDIO_VIDEO_FOOT_BOTTOM_TPU_01.stl", (250, 204, 21, 255)),
        ("V5_AUDIO_VIDEO_TIRE_01.stl", (51, 65, 85, 255)),
    ]

    scene = trimesh.Scene()
    cols = 4
    spacing_x = 0.145
    spacing_z = 0.145
    total_faces = 0

    for idx, (fname, rgba) in enumerate(kit_parts):
        path = v5_dir / fname
        tm = trimesh.load_mesh(path, force="mesh", process=False)
        tm = _decimate_if_huge(tm, max_faces=18000)
        tm.apply_scale(0.001)  # mm -> meters
        tm.apply_transform(zup_to_yup)

        bounds = tm.bounds
        cx = 0.5 * (bounds[0][0] + bounds[1][0])
        cz = 0.5 * (bounds[0][2] + bounds[1][2])
        ymin = bounds[0][1]

        col = idx % cols
        row = idx // cols
        slot_x = (col - (cols - 1) / 2.0) * spacing_x
        slot_z = (row - 1.0) * spacing_z
        tm.apply_translation([slot_x - cx, -ymin, slot_z - cz])

        color_arr = np.tile(np.array(rgba, dtype=np.uint8), (len(tm.faces), 1))
        tm.visual = trimesh.visual.ColorVisuals(mesh=tm, face_colors=color_arr)
        scene.add_geometry(tm, node_name=fname.replace(".stl", ""))
        total_faces += len(tm.faces)

    kit_glb = MODELS_DIR / "xgoduck_v5_stl_kit.glb"
    compat_glb = MODELS_DIR / "microduck_flux_ai.glb"
    scene.export(kit_glb)
    shutil.copyfile(kit_glb, compat_glb)
    return {
        "kit_glb": str(kit_glb.relative_to(PROJECT_ROOT)),
        "compat_glb": str(compat_glb.relative_to(PROJECT_ROOT)),
        "parts_included": len(kit_parts),
        "total_faces": total_faces,
        "size_bytes": kit_glb.stat().st_size,
    }


def export_roller_tire_json() -> Dict[str, Any]:
    """Export centered `V5_AUDIO_VIDEO_TIRE_01.stl` in meters (axle along X) for live WebGL Roller mode."""
    tire_stl = HW_REPO / "structure" / "V5_AUDIO_VIDEO_" / "V5_AUDIO_VIDEO_TIRE_01.stl"
    tm = trimesh.load_mesh(tire_stl, force="mesh", process=False)
    # Decimate slightly for smooth 60fps 4-wheel instancing
    if len(tm.faces) > 6000:
        try:
            tm = tm.simplify_quadric_decimation(face_count=4500)
        except Exception:
            tm = _decimate_if_huge(tm, max_faces=4500)
    tm.apply_scale(0.001)  # mm -> meters
    # Orient thinnest bounding dimension (wheel thickness) along X axis so rotation around X rolls forward
    ext = tm.extents
    thin_axis = int(np.argmin(ext))
    verts = tm.vertices.copy()
    if thin_axis == 1:
        verts = np.column_stack([verts[:, 1], verts[:, 2], verts[:, 0]])
    elif thin_axis == 2:
        verts = np.column_stack([verts[:, 2], verts[:, 1], verts[:, 0]])
    # Center around origin
    center = 0.5 * (verts.min(axis=0) + verts.max(axis=0))
    verts -= center

    out_json = MODELS_DIR / "xgoduck_v5_tire.json"
    payload = {
        "source": "xgoduck_hardware/structure/V5_AUDIO_VIDEO_/V5_AUDIO_VIDEO_TIRE_01.stl",
        "radius_m": round(float(max(verts[:, 1].max(), verts[:, 2].max())), 5),
        "width_m": round(float(verts[:, 0].max() - verts[:, 0].min()), 5),
        "verts": np.round(verts, 5).tolist(),
        "faces": tm.faces.astype(int).tolist(),
    }
    out_json.write_text(json.dumps(payload), encoding="utf-8")
    return {
        "path": str(out_json.relative_to(PROJECT_ROOT)),
        "faces": len(payload["faces"]),
        "radius_m": payload["radius_m"],
        "width_m": payload["width_m"],
    }


def copy_curated_stls_and_media() -> Dict[str, int]:
    STL_EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    FLUX_DIR.mkdir(parents=True, exist_ok=True)

    curated_v5 = [
        "V5_AUDIO_VIDEO_FACE.stl",
        "V5_AUDIO_VIDEO_JAW.stl",
        "V5_AUDIO_VIDEO_BODY_SHELL_LEFT.stl",
        "V5_AUDIO_VIDEO_THIGH_LEFT_SHELL.stl",
        "V5_AUDIO_VIDEO_LEFT_FOOT.stl",
        "V5_AUDIO_VIDEO_FOOT_BOTTOM_TPU_01.stl",
        "V5_AUDIO_VIDEO_TIRE_01.stl",
    ]
    copied_stl = 0
    for f in curated_v5:
        src = HW_REPO / "structure" / "V5_AUDIO_VIDEO_" / f
        if src.exists():
            shutil.copyfile(src, STL_EXPORT_DIR / f)
            copied_stl += 1

    v2_cover = HW_REPO / "structure" / "V2_STANDARD_" / "V2_STANDARD_HEAD_COVER.stl"
    if v2_cover.exists():
        shutil.copyfile(v2_cover, STL_EXPORT_DIR / "V2_STANDARD_HEAD_COVER.stl")
        copied_stl += 1

    copied_media = 0
    for m_name in ["xgo-duck-hero.jpg", "xgo-duck-open-beak.jpg", "xgo-duck-rear.jpg", "xgo-duck-somersault.webp"]:
        src = HW_REPO / "media" / m_name
        if src.exists():
            shutil.copyfile(src, FLUX_DIR / m_name)
            copied_media += 1

    return {"copied_stl": copied_stl, "copied_media": copied_media}


def main() -> None:
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    v2_entries = scan_stl_directory(HW_REPO / "structure" / "V2_STANDARD_", "V2_STANDARD")
    v5_entries = scan_stl_directory(HW_REPO / "structure" / "V5_AUDIO_VIDEO_", "V5_AUDIO_VIDEO")

    slicer_info = export_real_cad_slicer_stl()
    kit_info = build_v5_stl_kit_glb()
    tire_info = export_roller_tire_json()
    media_info = copy_curated_stls_and_media()

    catalog = {
        "source_repo": "https://github.com/LuwuDynamics/xgoduck_hardware",
        "summary": {
            "v2_standard_count": len(v2_entries),
            "v5_audio_video_count": len(v5_entries),
            "total_stl_count": len(v2_entries) + len(v5_entries),
            "total_v2_faces": sum(e["faces"] for e in v2_entries),
            "total_v5_faces": sum(e["faces"] for e in v5_entries),
        },
        "exports": {
            "slicer_full_cad_stl": slicer_info,
            "v5_stl_kit_glb": kit_info,
            "roller_tire_json": tire_info,
            **media_info,
        },
        "v2_standard": v2_entries,
        "v5_audio_video": v5_entries,
    }
    catalog_path = MODELS_DIR / "xgoduck_stl_catalog.json"
    catalog_path.write_text(json.dumps(catalog, indent=2), encoding="utf-8")

    # Emit synchronous JS bundle (`microduck_cad_bundle.js`) so the 3D CAD rig & V5 roller tires
    # load in 0 ms on frame 1 even when `web/index.html` is opened directly via `file://`
    rig_json_path = MODELS_DIR / "microduck_cad_rig.json"
    tire_json_path = MODELS_DIR / "xgoduck_v5_tire.json"
    if rig_json_path.exists() and tire_json_path.exists():
        bundle_js = MODELS_DIR / "microduck_cad_bundle.js"
        bundle_js.write_text(
            f"window.__MICRODUCK_CAD_RIG__ = {rig_json_path.read_text(encoding='utf-8').strip()};\n"
            f"window.__MICRODUCK_V5_TIRE__ = {tire_json_path.read_text(encoding='utf-8').strip()};\n",
            encoding="utf-8",
        )
        print(f"[✓] Synchronous CAD + Tire JS bundle: {bundle_js.relative_to(PROJECT_ROOT)} ({bundle_js.stat().st_size // 1024} KB)")

    print(f"[✓] Cataloged {catalog['summary']['total_stl_count']} real STLs ({len(v2_entries)} V2 + {len(v5_entries)} V5)")
    print(f"[✓] Real CAD slicer STL: {slicer_info['path']} ({slicer_info['faces']} tris, extents {slicer_info['extents_mm']} mm)")
    print(f"[✓] V5 STL Kit GLB: {kit_info['kit_glb']} ({kit_info['total_faces']} tris, {kit_info['size_bytes']//1024} KB)")
    print(f"[✓] Roller Tire JSON: {tire_info['path']} ({tire_info['faces']} tris, r={tire_info['radius_m']}m)")


if __name__ == "__main__":
    main()
