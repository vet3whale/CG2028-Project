import csv
import sys
from pathlib import Path

import sys
from pathlib import Path
import numpy as np
import imageio.v2 as imageio

import trimesh
import pyrender
from scipy.spatial.transform import Rotation as R

W, H = 512, 512

def load_trimesh_obj(path: Path) -> trimesh.Trimesh:
    tm = trimesh.load(str(path), force="mesh")
    if isinstance(tm, trimesh.Scene):
        tm = trimesh.util.concatenate(list(tm.geometry.values()))
    # center + scale for stable camera framing
    tm.vertices -= tm.vertices.mean(axis=0)
    tm.vertices /= np.max(np.linalg.norm(tm.vertices, axis=1))
    return tm

def load_rows(in_csv: Path):
    def remap(ax, ay, az):
        # X' = -Z, Y' = -X, Z' = -Y
        return ay, -az, ax
    rows = []
    with open(in_csv, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            ax, ay, az = float(row["ax"]), float(row["ay"]), float(row["az"])
            gx, gy, gz = float(row["gx"]), float(row["gy"]), float(row["gz"])

            ax, ay, az = remap(ax, ay, az)
            gx, gy, gz = remap(gx, gy, gz)

            rows.append({
                "t_ms": int(row["t_ms"]),
                "ax": ax, "ay": ay, "az": az,
                "gx": gx, "gy": gy, "gz": gz,
            })
    if not rows:
        raise RuntimeError(f"No data rows in {in_csv}")
    return rows


def main(csv_path, out_dir):
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)

    # Load mesh -> pyrender mesh
    tm = load_trimesh_obj(Path("human.obj"))
    pr_mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)

    scene = pyrender.Scene(bg_color=[0, 0, 0, 255], ambient_light=[0.2, 0.2, 0.2, 1.0])
    mesh_node = scene.add(pr_mesh)

    camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0)
    cam_pos = np.array([1.5, 1.5, 2.5], dtype=float)   # top-right
    target  = np.array([0.0, 0.0, 0.0], dtype=float)
    up_hint = np.array([0.0, 1.0, 0.0], dtype=float)

    forward = (target - cam_pos)
    forward = forward / np.linalg.norm(forward)

    right = np.cross(forward, up_hint)
    right = right / np.linalg.norm(right)

    up = np.cross(right, forward)

    cam_pose = np.eye(4, dtype=float)
    cam_pose[:3, 0] = right
    cam_pose[:3, 1] = up
    cam_pose[:3, 2] = -forward   # OpenGL camera +Z points “back”
    cam_pose[:3, 3] = cam_pos

    scene.add(camera, pose=cam_pose)


    light = pyrender.DirectionalLight(color=np.ones(3), intensity=2.0)
    scene.add(light, pose=cam_pose)

    r = pyrender.OffscreenRenderer(viewport_width=W, viewport_height=H)

    rows = load_rows(Path(csv_path))

    R_cur = R.identity()
    for i in range(len(rows)):
        dt = 0.0 if i == 0 else (rows[i]["t_ms"] - rows[i-1]["t_ms"]) / 1000.0
        angle_deg = np.array([rows[i]["gx"], rows[i]["gy"], rows[i]["gz"]]) * dt
        dR = R.from_euler("xyz", angle_deg, degrees=True)
        R_cur = dR * R_cur

        # pose for pyrender node (4x4)
        pose = np.eye(4)
        pose[:3, :3] = R_cur.as_matrix()
        scene.set_pose(mesh_node, pose=pose)  # Scene can update node pose

        color, depth = r.render(scene)  # returns image arrays
        imageio.imwrite(out_dir / f"frame_{i:03d}.png", color)

    r.delete()

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
