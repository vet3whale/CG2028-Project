import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def load_rows(in_csv: Path):
    rows = []
    with open(in_csv, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append({
                "t_ms": int(row["t_ms"]),
                "ax": float(row["ax"]), "ay": float(row["ay"]), "az": float(row["az"]),
                "gx": float(row["gx"]), "gy": float(row["gy"]), "gz": float(row["gz"]),
            })
    if not rows:
        raise RuntimeError(f"No data rows in {in_csv}")
    return rows

def set_black_background(fig, ax):
    fig.patch.set_facecolor("black")
    ax.set_facecolor("black")
    ax.set_axis_off()

    ax.grid(False)


def make_cuboid_vertices(w=0.5, d=0.2, h=1.5):
    # half-dimensions
    x, y, z = w/2, d/2, h/2
    V = np.array([
        [-x, -y, -z],
        [ x, -y, -z],
        [ x,  y, -z],
        [-x,  y, -z],
        [-x, -y,  z],
        [ x, -y,  z],
        [ x,  y,  z],
        [-x,  y,  z],
    ])
    # faces as lists of 4 vertices (quads)
    faces = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # side
        [1, 2, 6, 5],  # side
        [2, 3, 7, 6],  # side
        [3, 0, 4, 7],  # side
    ]
    return V, faces


def draw_cuboid(ax, verts, faces):
    polys = [[verts[idx] for idx in face] for face in faces]

    face_colors = [
        "#ff3b30",  # bottom - red
        "#34c759",  # top - green
        "#0a84ff",  # side - blue
        "#ff9f0a",  # side - orange
        "#bf5af2",  # side - purple
        "#ffd60a",  # side - yellow
    ]

    pc = Poly3DCollection(
        polys,
        facecolors=face_colors,
        linewidths=1.0,
    )
    ax.add_collection3d(pc)


def main():
    if len(sys.argv) < 3:
        print("Usage: python csv_to_frames.py <input_csv> <output_frames_dir>")
        sys.exit(2)

    in_csv = Path(sys.argv[1])
    out_dir = Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = load_rows(in_csv)

    # Base cuboid geometry
    base_verts, faces = make_cuboid_vertices(w=0.6, d=0.2, h=1.6)

    # Initial orientation: identity
    R_cur = R.identity()

    # Fix plot limits so cuboid doesn't rescale each frame
    axis_lim = 1.2 

    for i in range(len(rows)):
        if i == 0:
            dt = 0.0
        else:
            dt = (rows[i]["t_ms"] - rows[i - 1]["t_ms"]) / 1000.0

        # Angular velocity vector (deg/s) -> small rotation over dt (deg)
        gx = rows[i]["gx"]
        gy = rows[i]["gy"]
        gz = rows[i]["gz"]
        angle_deg = np.array([gx, gy, gz]) * dt

        # Incremental rotation; choose axis order to match your IMU convention
        dR = R.from_euler("xyz", angle_deg, degrees=True)
        R_cur = dR * R_cur  # compose new orientation

        # Rotate cuboid vertices
        verts_rot = R_cur.apply(base_verts)

        # Draw cuboid
        fig = plt.figure(figsize=(5, 5))
        ax = fig.add_subplot(111, projection="3d")
        set_black_background(fig, ax)
        draw_cuboid(ax, verts_rot, faces)

        # Fix axes for consistency across frames
        ax.set_xlim(-axis_lim, axis_lim)
        ax.set_ylim(-axis_lim, axis_lim)
        ax.set_zlim(-axis_lim, axis_lim)
        ax.set_box_aspect([1, 1, 1])

        out_path = out_dir / f"frame_{i:03d}.png"
        plt.tight_layout()
        plt.savefig(out_path, dpi=120)
        plt.close()

    print(f"Wrote {len(rows)} 3D cube frames to {out_dir}")


if __name__ == "__main__":
    main()
