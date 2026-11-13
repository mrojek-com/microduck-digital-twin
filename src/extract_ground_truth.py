#!/usr/bin/env python3
"""
apps/MicroDuck/src/extract_ground_truth.py

Extracts ground-truth geometry, kinematics, emotions, scores, and ONNX models
from the cloned `pollen-robotics/microduck` repository into `web/assets/`.

Outputs:
  - web/assets/models/microduck_cad_rig.json  (Articulated 15-DOF CAD rig parsed from robotctl/assets/duck.bin)
  - web/assets/models/microduck_cad_articulated.glb (Static Home-Pose GLB export via trimesh)
  - web/assets/models/microduck_mjcf.json     (Parsed MuJoCo MJCF joints, ranges, inertias, sensor sites)
  - web/assets/emotions/*                     (All 13 emotion JSON keyframes + WAV audio)
  - web/assets/scores/*                       (wistful.duckscore, duck_strut.mid, outer_wilds.mid)
  - web/assets/policies/*                     (pickup_detector.onnx, pet_detect.onnx, official manifest + ONNX)
"""

import json
import math
import shutil
import struct
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = PROJECT_ROOT / "repo"
WEB_ASSETS = PROJECT_ROOT / "web" / "assets"

JOINT_NAMES = [
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "mouth",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
]

DEFAULT_POSITION = [
    0.0,
    -0.0873,
    -0.4579,
    -0.0049,
    0.4530,
    0.3491,
    0.3491,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0873,
    0.4579,
    0.0049,
    -0.4530,
]


def quat_to_mat(q: List[float]) -> np.ndarray:
    """Convert MJCF quaternion (w, x, y, z) to 3x3 rotation matrix."""
    arr = np.array(q, dtype=np.float64)
    n = np.linalg.norm(arr)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = arr / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def axis_angle_mat(axis: List[float], angle: float) -> np.ndarray:
    """Rodrigues rotation matrix for a unit axis and angle in radians."""
    ax = np.array(axis, dtype=np.float64)
    n = np.linalg.norm(ax)
    if n < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z = ax / n
    s, c = math.sin(angle), math.cos(angle)
    ic = 1.0 - c
    return np.array(
        [
            [c + x * x * ic, x * y * ic - z * s, x * z * ic + y * s],
            [y * x * ic + z * s, c + y * y * ic, y * z * ic - x * s],
            [z * x * ic - y * s, z * y * ic + x * s, c + z * z * ic],
        ],
        dtype=np.float64,
    )


def pose_to_mat4(rot: np.ndarray, trans: List[float]) -> np.ndarray:
    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = rot
    m[:3, 3] = np.array(trans, dtype=np.float64)
    return m


def parse_duck_bin(bin_path: Path) -> Dict[str, Any]:
    """Parse `robotctl/assets/duck.bin` (DUCK format v1)."""
    data = bin_path.read_bytes()
    offset = 0

    magic, version, n_meshes, n_bodies, n_parts = struct.unpack_from("<4sIHHH", data, offset)
    offset += struct.calcsize("<4sIHHH")
    if magic != b"DUCK" or version != 1:
        raise ValueError(f"Unsupported duck.bin header: magic={magic!r}, version={version}")

    meshes: List[Dict[str, Any]] = []
    for _ in range(n_meshes):
        n_verts, n_tris = struct.unpack_from("<HH", data, offset)
        offset += 4
        v_bytes = n_verts * 3 * 4
        t_bytes = n_tris * 3 * 2
        verts = np.frombuffer(data, dtype="<f4", count=n_verts * 3, offset=offset).reshape(n_verts, 3)
        offset += v_bytes
        tris = np.frombuffer(data, dtype="<u2", count=n_tris * 3, offset=offset).reshape(n_tris, 3)
        offset += t_bytes
        meshes.append(
            {
                "verts": np.round(verts, 6).tolist(),
                "tris": tris.astype(int).tolist(),
            }
        )

    bodies: List[Dict[str, Any]] = []
    body_fmt = "<hh10f"
    body_size = struct.calcsize(body_fmt)
    for _ in range(n_bodies):
        vals = struct.unpack_from(body_fmt, data, offset)
        offset += body_size
        parent, joint = vals[0], vals[1]
        pos = list(vals[2:5])
        quat = list(vals[5:9])  # w, x, y, z
        axis = list(vals[9:12])
        bodies.append(
            {
                "parent": int(parent),
                "joint": int(joint),
                "joint_name": JOINT_NAMES[joint] if 0 <= joint < len(JOINT_NAMES) else None,
                "pos": [round(float(x), 6) for x in pos],
                "quat": [round(float(x), 6) for x in quat],
                "axis": [round(float(x), 6) for x in axis],
            }
        )

    parts: List[Dict[str, Any]] = []
    part_fmt = "<HHBBBB7f"
    part_size = struct.calcsize(part_fmt)
    for _ in range(n_parts):
        vals = struct.unpack_from(part_fmt, data, offset)
        offset += part_size
        body_idx, mesh_idx = vals[0], vals[1]
        rgb = [int(vals[2]), int(vals[3]), int(vals[4])]
        pos = list(vals[6:9])
        quat = list(vals[9:13])  # w, x, y, z
        parts.append(
            {
                "body": int(body_idx),
                "mesh": int(mesh_idx),
                "rgb": rgb,
                "pos": [round(float(x), 6) for x in pos],
                "quat": [round(float(x), 6) for x in quat],
            }
        )

    return {
        "format": "DUCK_V1",
        "joint_names": JOINT_NAMES,
        "default_position": DEFAULT_POSITION,
        "meshes": meshes,
        "bodies": bodies,
        "parts": parts,
    }


def export_home_pose_glb(rig: Dict[str, Any], out_glb: Path) -> Dict[str, Any]:
    """Assemble the articulated CAD parts in Home Pose and export a Y-up GLB."""
    bodies = rig["bodies"]
    meshes = rig["meshes"]
    parts = rig["parts"]

    world_mats: List[np.ndarray] = []
    for b in bodies:
        parent_mat = world_mats[b["parent"]] if b["parent"] >= 0 else np.eye(4, dtype=np.float64)
        local_mat = pose_to_mat4(quat_to_mat(b["quat"]), b["pos"])
        mat = parent_mat @ local_mat
        if 0 <= b["joint"] < len(DEFAULT_POSITION):
            angle = DEFAULT_POSITION[b["joint"]]
            mat = mat @ pose_to_mat4(axis_angle_mat(b["axis"], angle), [0.0, 0.0, 0.0])
        world_mats.append(mat)

    # Convert Z-up (robot frame) -> glTF Y-up: (x, y, z) -> (x, z, -y)
    zup_to_yup = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    scene = trimesh.Scene()
    min_y = float("inf")
    max_y = float("-inf")
    total_tris = 0

    for idx, p in enumerate(parts):
        m_data = meshes[p["mesh"]]
        verts = np.array(m_data["verts"], dtype=np.float64)
        faces = np.array(m_data["tris"], dtype=np.int64)
        if len(verts) == 0 or len(faces) == 0:
            continue
        part_local = pose_to_mat4(quat_to_mat(p["quat"]), p["pos"])
        full_mat = zup_to_yup @ world_mats[p["body"]] @ part_local

        tm = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        tm.apply_transform(full_mat)
        rgba = np.array([p["rgb"][0], p["rgb"][1], p["rgb"][2], 255], dtype=np.uint8)
        tm.visual = trimesh.visual.ColorVisuals(mesh=tm, face_colors=np.tile(rgba, (len(faces), 1)))
        scene.add_geometry(tm, node_name=f"part_{idx:03d}")
        min_y = min(min_y, float(tm.bounds[0][1]))
        max_y = max(max_y, float(tm.bounds[1][1]))
        total_tris += len(faces)

    # Shift all geometries so the lowest foot vertex sits cleanly at Y = 0
    if math.isfinite(min_y):
        for geom in scene.geometry.values():
            geom.apply_translation([0.0, -min_y, 0.0])

    out_glb.parent.mkdir(parents=True, exist_ok=True)
    scene.export(str(out_glb))
    return {
        "glb_path": str(out_glb.relative_to(PROJECT_ROOT)),
        "parts": len(parts),
        "total_triangles": total_tris,
        "height_m": round(max_y - min_y, 4) if math.isfinite(min_y) else 0.0,
        "foot_offset_y": round(-min_y, 5) if math.isfinite(min_y) else 0.0,
    }


def parse_mjcf(mjcf_path: Path) -> Dict[str, Any]:
    """Parse `kinematics/assets/alpha/robot_walk.xml` for joint limits, inertias, and sites."""
    root = ET.parse(mjcf_path).getroot()
    bodies: List[Dict[str, Any]] = []
    joints: List[Dict[str, Any]] = []
    sites: List[Dict[str, Any]] = []

    def parse_vec(val: str | None, default: List[float]) -> List[float]:
        if not val:
            return default
        return [float(x) for x in val.split()]

    def walk(elem: ET.Element, parent_name: str | None) -> None:
        b_name = elem.get("name", "unnamed")
        pos = parse_vec(elem.get("pos"), [0.0, 0.0, 0.0])
        quat = parse_vec(elem.get("quat"), [1.0, 0.0, 0.0, 0.0])
        inertial = elem.find("inertial")
        mass = float(inertial.get("mass")) if inertial is not None and inertial.get("mass") else 0.0

        j_elem = elem.find("joint")
        j_info = None
        if j_elem is not None and j_elem.get("type", "hinge") == "hinge":
            j_name = j_elem.get("name")
            rng_raw = j_elem.get("range")
            rng = [float(x) for x in rng_raw.split()] if rng_raw else None
            axis = parse_vec(j_elem.get("axis"), [0.0, 0.0, 1.0])
            j_info = {"name": j_name, "axis": axis, "range_rad": rng}
            joints.append(j_info)

        bodies.append(
            {
                "name": b_name,
                "parent": parent_name,
                "pos": pos,
                "quat": quat,
                "mass_kg": mass,
                "joint": j_info,
            }
        )

        for s_elem in elem.findall("site"):
            sites.append(
                {
                    "name": s_elem.get("name"),
                    "body": b_name,
                    "pos": parse_vec(s_elem.get("pos"), [0.0, 0.0, 0.0]),
                    "quat": parse_vec(s_elem.get("quat"), [1.0, 0.0, 0.0, 0.0]),
                }
            )

        for child in elem.findall("body"):
            walk(child, b_name)

    for top in root.find("worldbody").findall("body"):
        walk(top, None)

    total_mass = sum(b["mass_kg"] for b in bodies)
    return {
        "mjcf_source": str(mjcf_path.relative_to(REPO_ROOT)),
        "total_mass_kg": round(total_mass, 4),
        "body_count": len(bodies),
        "joint_count": len(joints),
        "joints": joints,
        "sites": sites,
        "bodies": bodies,
    }


def copy_repo_assets() -> Dict[str, Any]:
    """Copy real emotion keyframes/WAVs, chorale scores, and ONNX models into web/assets/."""
    emotions_src = REPO_ROOT / "robotd" / "emotions"
    emotions_dst = WEB_ASSETS / "emotions"
    emotions_dst.mkdir(parents=True, exist_ok=True)
    emotion_names = sorted({p.stem for p in emotions_src.glob("*.json")})
    for p in emotions_src.iterdir():
        if p.suffix in {".json", ".wav"}:
            shutil.copy2(p, emotions_dst / p.name)
    (emotions_dst / "index.json").write_text(json.dumps(emotion_names, indent=2), encoding="utf-8")

    scores_src = REPO_ROOT / "sounds" / "scores"
    scores_dst = WEB_ASSETS / "scores"
    scores_dst.mkdir(parents=True, exist_ok=True)
    score_files: List[str] = []
    for p in sorted(scores_src.iterdir()):
        if p.is_file():
            shutil.copy2(p, scores_dst / p.name)
            score_files.append(p.name)

    policies_dst = WEB_ASSETS / "policies"
    policies_dst.mkdir(parents=True, exist_ok=True)
    for src in [
        REPO_ROOT / "duck-control" / "models" / "pickup_detector.onnx",
        REPO_ROOT / "pet-detect" / "models" / "pet_detect.onnx",
    ]:
        if src.exists():
            shutil.copy2(src, policies_dst / src.name)

    # Fetch official Hugging Face policy manifest & velstand.onnx if reachable
    hf_base = "https://huggingface.co/pollen-robotics/microduck-policies/resolve/main"
    fetched = []
    for fname in ["manifest.json", "velstand.onnx"]:
        target = policies_dst / fname
        if not target.exists():
            try:
                urllib.request.urlretrieve(f"{hf_base}/{fname}", target)
                fetched.append(fname)
            except Exception as exc:
                print(f"[warn] Could not fetch {fname} from HF: {exc}")

    return {
        "emotions": emotion_names,
        "scores": score_files,
        "fetched_hf_policies": fetched,
    }


def main() -> None:
    models_dir = WEB_ASSETS / "models"
    models_dir.mkdir(parents=True, exist_ok=True)

    duck_bin = REPO_ROOT / "robotctl" / "assets" / "duck.bin"
    rig = parse_duck_bin(duck_bin)
    glb_stats = export_home_pose_glb(rig, models_dir / "microduck_cad_articulated.glb")
    rig["home_pose_stats"] = glb_stats
    (models_dir / "microduck_cad_rig.json").write_text(json.dumps(rig), encoding="utf-8")

    mjcf_path = REPO_ROOT / "kinematics" / "assets" / "alpha" / "robot_walk.xml"
    mjcf_data = parse_mjcf(mjcf_path)
    (models_dir / "microduck_mjcf.json").write_text(json.dumps(mjcf_data, indent=2), encoding="utf-8")

    asset_summary = copy_repo_assets()

    summary = {
        "cad_rig": {
            "meshes": len(rig["meshes"]),
            "bodies": len(rig["bodies"]),
            "parts": len(rig["parts"]),
            **glb_stats,
        },
        "mjcf": {
            "total_mass_kg": mjcf_data["total_mass_kg"],
            "joints": mjcf_data["joint_count"],
            "sites": [s["name"] for s in mjcf_data["sites"]],
        },
        "assets": asset_summary,
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
