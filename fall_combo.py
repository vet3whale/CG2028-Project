# -*- coding: utf-8 -*-
import csv
import json
import os
import re
import subprocess
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from serial.tools import list_ports

import requests
import serial

# ===================== CONFIG =====================
PORT = "COM3"
BAUD = 115200
TIMEOUT_S = 1

BOT_TOKEN = "8463120288:AAEjTsTXQ1YChPB53w92LNjCf_6rKWwlPJo"
CHAT_ID = "-5171478433"  # can keep as string, grpchat ID

NIC_NUMBER = "+6588181916"

# How many IMU samples to keep for video
BUFFER_LEN = 75

# Trigger texts that may appear from STM32
TRIGGER_TEXTS = [
    "FALL DETECTED",          # his prints
    "!!! FALL DETECTED !!!",  # his prints
    "FALL axis=",             # your machine line
]

# Where to save outputs
ROOT = Path(__file__).parent
OUTPUT_DIR = ROOT / "captures"
FRAMES_ROOT = ROOT / "frames_out"
VIDEOS_ROOT = ROOT / "videos_out"
CSV_TO_FRAMES = ROOT / "csvtoframes.py"  # match your friend filename exactly

# CSV format from STM32 stream
FIELDNAMES = ["t_ms", "ax", "ay", "az", "gx", "gy", "gz"]

# Telegram cooldown so you don't spam
TELEGRAM_COOLDOWN_S = 10


# ========================= PORT DETECTION =======================
def auto_detect_port():
    ports = list_ports.comports()
    for p in ports:
        desc = p.description.lower()
        if "stm" in desc or "usb serial" in desc:
            print(f"Auto-detected port: {p.device} ({p.description})")
            return p.device

    # fallback if nothing matched
    if ports:
        print("No STM match found. Using first available port:", ports[0].device)
        return ports[0].device

    raise RuntimeError("No serial ports found.")

# ========================= TELEGRAM API ========================

API = f"https://api.telegram.org/bot{BOT_TOKEN}"

last_telegram_sent = 0.0


# ---------------- Telegram (HTTP API) ----------------
def send_message(text: str, reply_markup=None):
    payload = {"chat_id": CHAT_ID, "text": text}
    if reply_markup is not None:
        payload["reply_markup"] = json.dumps(reply_markup)

    r = requests.post(f"{API}/sendMessage", data=payload, timeout=15)
    if not r.ok:
        print("Telegram sendMessage failed:", r.status_code, r.text)
        r.raise_for_status()
    return r.json()

def send_video(video_path: str, caption: str | None = None):
    with open(video_path, "rb") as f:
        files = {"video": f}
        data = {"chat_id": CHAT_ID}
        if caption:
            data["caption"] = caption

        r = requests.post(f"{API}/sendVideo", data=data, files=files, timeout=60)

    if not r.ok:
        print("Telegram sendVideo failed:", r.status_code, r.text)
        r.raise_for_status()

    return r.json()

def edit_message_text(chat_id: int, message_id: int, text: str, reply_markup=None):
    payload = {"chat_id": chat_id, "message_id": message_id, "text": text}
    if reply_markup is not None:
        payload["reply_markup"] = json.dumps(reply_markup)
    else:
        payload["reply_markup"] = json.dumps({"inline_keyboard": []})

    r = requests.post(f"{API}/editMessageText", data=payload, timeout=15)
    if not r.ok:
        print("Telegram editMessageText failed:", r.status_code, r.text)
        r.raise_for_status()
    return r.json()


def answer_callback_query(callback_query_id: str):
    try:
        r = requests.post(
            f"{API}/answerCallbackQuery",
            data={"callback_query_id": callback_query_id},
            timeout=10
        )
        # If it's an old query, just ignore it
        if not r.ok:
            print("Ignoring old/invalid callback:", r.text)
    except Exception as e:
        print("Callback answer failed (ignored):", e)


def build_keyboard():
    return {
        "inline_keyboard": [
            [{"text": "False Alarm! Don’t Take Action", "callback_data": "false_alarm"}],
            [{"text": "Contact Nic", "callback_data": "contact_nic"}],
        ]
    }


def build_alert_text(axis_from, axis_to, peak_g, lying_s):
    return (
        "User has fallen at home!\n\n"
        f"Details:\n"
        f"- Rotation detected.\n"
        f"- Fall axis: {axis_from} -> {axis_to}\n"
        f"- Peak acceleration: {peak_g:.2f} g\n"
        f"- Lying duration: {lying_s} seconds\n"
    )


def parse_machine_fall_line(line: str):
    """
    Parse:
      FALL axis=0->2 peak_g=1.24 lying_s=5
    Returns dict or None
    """
    m = re.search(r"FALL\s+axis=(\d+)->(\d+)\s+peak_g=([0-9.]+)(?:\s+lying_s=(\d+))?", line)
    if not m:
        return None
    return {
        "axis_from": int(m.group(1)),
        "axis_to": int(m.group(2)),
        "peak_g": float(m.group(3)),
        "lying_s": int(m.group(4)) if m.group(4) else None,
    }


def telegram_updates_loop(shared_state):
    """
    Poll Telegram callbacks to handle buttons.
    shared_state holds:
      - last_alert: {"chat_id": int, "message_id": int, "text": str} or None
    """
    offset = None
    while True:
        params = {"timeout": 30}
        if offset is not None:
            params["offset"] = offset

        r = requests.get(f"{API}/getUpdates", params=params, timeout=35)
        if not r.ok:
            print("getUpdates failed:", r.status_code, r.text)
            time.sleep(1)
            continue

        updates = r.json().get("result", [])
        for u in updates:
            offset = u["update_id"] + 1
            cq = u.get("callback_query")
            if not cq:
                continue

            cq_id = cq["id"]
            data = cq.get("data", "")
            answer_callback_query(cq_id)

            msg = cq.get("message", {})
            chat = msg.get("chat", {})
            chat_id = chat.get("id")
            message_id = msg.get("message_id")
            original_text = msg.get("text", "")

            if chat_id is None or message_id is None:
                continue

            if data == "false_alarm":
                new_text = original_text.rstrip() + "\n\nUser pressed False Alarm. No action will be done."
                try:
                    edit_message_text(chat_id, message_id, new_text, reply_markup={"inline_keyboard": []})
                except Exception:
                    pass

            elif data == "contact_nic":
                new_text = original_text.rstrip() + f"\n\nIf you need to call, call this number: {NIC_NUMBER}"
                try:
                    edit_message_text(chat_id, message_id, new_text, reply_markup={"inline_keyboard": []})
                except Exception:
                    pass

        time.sleep(0.1)


# ---------------- UART + Video pipeline ----------------
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


def write_buffer_to_csv(rows, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)


def run_postprocess(csv_file: Path):
    """
    CSV -> frames -> mp4
    Returns mp4 path as string, or None if failed.
    """
    try:
        frames_dir = (FRAMES_ROOT / csv_file.stem)
        VIDEOS_ROOT.mkdir(parents=True, exist_ok=True)
        mp4_out = (VIDEOS_ROOT / f"{csv_file.stem}.mp4")

        # 1) CSV -> frames
        subprocess.run(
            [sys.executable, str(CSV_TO_FRAMES), str(csv_file), str(frames_dir)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"Frames saved -> {frames_dir}")

        # 2) frames -> mp4
        subprocess.run(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostats", "-y", "-framerate", "25", "-i", str(frames_dir / "frame_%03d.png"),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", str(mp4_out)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"Rendered video -> {mp4_out}")
        return str(mp4_out)
    except Exception as e:
        print("Video postprocess failed:", e)
        return None


def uart_loop():
    global last_telegram_sent

    buf = deque(maxlen=BUFFER_LEN)
    port = auto_detect_port()
    ser = serial.Serial(port, BAUD, timeout=TIMEOUT_S)
    print(f"Listening on {ser.port} @ {BAUD}...")

    last_video_path = None  # IMPORTANT

    while True:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        # 1️⃣ Always try to parse IMU CSV samples for video
        sample = parse_imu_csv(line)
        if sample is not None:
            buf.append(sample)
            continue

        # 2️⃣ Handle trigger lines
        is_video_trigger = "!!! FALL DETECTED !!!" in line
        is_data_trigger = "FALL axis=" in line

        if not (is_video_trigger or is_data_trigger):
            continue

        print("Trigger line:", line)

        # --- VIDEO TRIGGER ---
        if is_video_trigger:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            csv_path = OUTPUT_DIR / f"fall_{ts}.csv"

            write_buffer_to_csv(list(buf), csv_path)
            last_video_path = run_postprocess(csv_path)

        # --- DATA TRIGGER (send Telegram here) ---
        if is_data_trigger:
            fall_info = parse_machine_fall_line(line)
            if not fall_info:
                continue

            now = time.time()
            if now - last_telegram_sent >= TELEGRAM_COOLDOWN_S:

                text = build_alert_text(
                    fall_info["axis_from"],
                    fall_info["axis_to"],
                    fall_info["peak_g"],
                    fall_info["lying_s"] or 0
                )

                # Send text first
                send_message(text, reply_markup=build_keyboard())

                # Then upload video
                if last_video_path and os.path.exists(last_video_path):
                    send_video(last_video_path)

                last_telegram_sent = now

# ---------------- main ----------------
shared_state = {"last_alert": None}

def main():
    import threading
    t = threading.Thread(target=telegram_updates_loop, args=(shared_state,), daemon=True)
    t.start()
    uart_loop()


if __name__ == "__main__":
    main()
