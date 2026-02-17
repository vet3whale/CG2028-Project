
import csv
import os
from collections import deque
from datetime import datetime

import serial 
import subprocess
import sys
from pathlib import Path

PORT = "COM7"
BAUD = 115200
TIMEOUT_S = 1

TRIGGER_TEXT = "FALL DETECTED"
BUFFER_LEN = 75

OUTPUT_DIR = "captures"
FIELDNAMES = ["t_ms", "ax", "ay", "az", "gx", "gy", "gz"]

ROOT = Path(__file__).parent

def run_postprocess(csv_file: str):
    csv_file = Path(csv_file)

    frames_dir = (ROOT / "frames_out" / csv_file.stem)
    videos_dir = (ROOT / "videos_out")
    videos_dir.mkdir(parents=True, exist_ok=True)

    mp4_out = (videos_dir / f"{csv_file.stem}.mp4")

    csv_to_frames = (ROOT / "csvToFrames.py")

    # 1) CSV -> frames
    subprocess.run(
        [sys.executable, str(csv_to_frames), str(csv_file), str(frames_dir)],
        check=True
    )

    # 2) frames -> mp4
    subprocess.run(
        ["ffmpeg", "-y", "-framerate", "25",
         "-i", str(frames_dir / "frame_%03d.png"),
         "-c:v", "libx264", "-pix_fmt", "yuv420p", str(mp4_out)],
        check=True
    )

    return str(mp4_out)

def parse_imu_csv(line: str):
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 7:
        return None

    try:
        return {
            "t_ms": int(parts[0]),
            "ax": float(parts[1]), "ay": float(parts[2]), "az": float(parts[3]),
            "gx": float(parts[4]), "gy": float(parts[5]), "gz": float(parts[6]),
        }
    except ValueError:
        return None


def write_buffer_to_csv(rows, out_path: str):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)


def main():
    buf = deque(maxlen=BUFFER_LEN)

    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S)
    print(f"Listening on {ser.port} @ {BAUD}...")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if TRIGGER_TEXT in line:
                print("Fall Detected...Creating Video")
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                out_path = os.path.join(OUTPUT_DIR, f"fall_{ts}.csv")
                write_buffer_to_csv(list(buf), out_path)
                mp4 = run_postprocess(out_path)
                print(f"Saved {BUFFER_LEN} samples -> {out_path}")
                print(f"Rendered video -> {mp4}")
                continue

            sample = parse_imu_csv(line)
            if sample is not None:
                buf.append(sample)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
