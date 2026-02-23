# -*- coding: utf-8 -*-
import csv
import json
import os
import re
import subprocess
import sys
import time
import threading
import random
import requests
import serial
 
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from serial.tools import list_ports

# ===================== CONFIG =====================
PORT = "COM3"
BAUD = 115200
TIMEOUT_S = 1

SER = None
SER_LOCK = threading.Lock()

BOT_TOKEN = "8463120288:AAEjTsTXQ1YChPB53w92LNjCf_6rKWwlPJo"
CHAT_ID = "-1003576715549"  # supergroup chat id

AMBULANCE_NUMBER = "995"

# How many IMU samples to keep for video
SAMPLE_RATE = 50
CLIP_SECONDS = 3
BUFFER_LEN = SAMPLE_RATE * CLIP_SECONDS

# NOTE: Unused in current logic, kept for reference.
TRIGGER_TEXTS = [
    "FALL DETECTED",
    "!!! FALL DETECTED !!!",
    "FALL axis=",
]

SESSION = requests.Session()
_retry = Retry(
    total=5,
    connect=5,
    read=5,
    status=5,
    backoff_factor=0.5,
    status_forcelist=(429, 500, 502, 503, 504),
    allowed_methods=frozenset(["GET", "POST"]),
    raise_on_status=False,
)
adapter = HTTPAdapter(max_retries=_retry, pool_connections=10, pool_maxsize=10)
SESSION.mount("https://", adapter)
SESSION.mount("http://", adapter)

ASSIST_TEXT = "Not a False Alarm! User needs assistance!"

# Where to save outputs
ROOT = Path(__file__).parent
OUTPUT_DIR = ROOT / "captures"
FRAMES_ROOT = ROOT / "frames_out"
VIDEOS_ROOT = ROOT / "videos_out"

CSV_TO_FRAMES_CANDIDATES = [
    ROOT / "csvToFrames.py",
    ROOT / "csvtoframes.py",
]
CSV_TO_FRAMES = next(
    (p for p in CSV_TO_FRAMES_CANDIDATES if p.exists()),
    CSV_TO_FRAMES_CANDIDATES[0],
)

# CSV format from STM32 stream
FIELDNAMES = ["t_ms", "ax", "ay", "az", "gx", "gy", "gz", "fall_alt"]

# Telegram cooldown so you don't spam initial alerts
TELEGRAM_COOLDOWN_S = 1
assist_last_seen_ts = 0.0
ASSIST_MIN_INTERVAL = 1.0

# ===== Reminder settings =====
REMIND_EVERY_SECONDS = 15
MAX_REMINDERS = 999999

POST_CAPTURE_TIMEOUT_S = 2.0  # finalize even if we don't get enough samples

# ===================== RUNTIME STATE =====================
API = f"https://api.telegram.org/bot{BOT_TOKEN}"
last_telegram_sent = 0.0

post_capture_started_ts = None
pending_video_upload = False  # set True when we send alert but mp4 not ready yet
pending_reminder_start = None


# ========================= PORT DETECTION =======================
def auto_detect_port() -> str:
    ports = list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        if "stm" in desc or "usb serial" in desc:
            print(f"Auto-detected port: {p.device} ({p.description})")
            return p.device

    if ports:
        print("No STM match found. Using first available port:", ports[0].device)
        return ports[0].device

    raise RuntimeError("No serial ports found.")

# ======================= STM32 Buzzer Helpers ========================
def get_ser():
    global SER
    if SER is None or not SER.is_open:
        raise RuntimeError("Serial not initialized yet. uart_loop() must start first.")
    return SER

def send_to_stm32(cmd: str):
    global SER
    try:
        if SER is None or not SER.is_open:
            raise RuntimeError("Serial not open")

        data = (cmd.strip() + "\n").encode("utf-8")
        with SER_LOCK:
            SER.write(data)
            SER.flush()
        print("Sent:", cmd)
    except Exception as e:
        print("Failed to send to STM32:", e)


# ========================= TELEGRAM HELPERS ========================
def _tg_post(method: str, *, data=None, files=None, timeout=15):
    url = f"{API}/{method}"
    r = SESSION.post(url, data=data, files=files, timeout=timeout)
    if not r.ok:
        print(f"Telegram {method} failed:", r.status_code, r.text)
        r.raise_for_status()
    return r.json()

def _tg_get(method: str, *, params=None, timeout=35):
    url = f"{API}/{method}"
    r = SESSION.get(url, params=params, timeout=timeout)
    if not r.ok:
        print(f"Telegram {method} failed:", r.status_code, r.text)
    return r

def send_message(text: str, reply_markup=None):
    payload = {"chat_id": CHAT_ID, "text": text}
    if reply_markup is not None:
        payload["reply_markup"] = json.dumps(reply_markup)

    resp = _tg_post("sendMessage", data=payload, timeout=15)
    print("✅ Telegram message sent!")
    return resp

def send_video(video_path: str, caption: str | None = None):
    with open(video_path, "rb") as f:
        files = {"video": f}
        data = {"chat_id": CHAT_ID}
        if caption:
            data["caption"] = caption

        return _tg_post("sendVideo", data=data, files=files, timeout=60)

def edit_message_text(chat_id: int, message_id: int, text: str, reply_markup=None):
    payload = {"chat_id": chat_id, "message_id": message_id, "text": text}
    if reply_markup is not None:
        payload["reply_markup"] = json.dumps(reply_markup)
    else:
        payload["reply_markup"] = json.dumps({"inline_keyboard": []})

    return _tg_post("editMessageText", data=payload, timeout=15)

def edit_message_reply_markup(chat_id: int, message_id: int, reply_markup=None):
    payload = {"chat_id": chat_id, "message_id": message_id}
    if reply_markup is not None:
        payload["reply_markup"] = json.dumps(reply_markup)
    else:
        payload["reply_markup"] = json.dumps({"inline_keyboard": []})

    try:
        return _tg_post("editMessageReplyMarkup", data=payload, timeout=15)
    except Exception as e:
        # match your original "print but don't crash" behavior
        print("Telegram editMessageReplyMarkup failed:", e)
        return None

def delete_message(chat_id: int, message_id: int) -> bool:
    try:
        _tg_post("deleteMessage", data={"chat_id": chat_id, "message_id": message_id}, timeout=15)
        return True
    except Exception as e:
        print("Telegram deleteMessage failed:", e)
        return False

def answer_callback_query(callback_query_id: str):
    try:
        _tg_post("answerCallbackQuery", data={"callback_query_id": callback_query_id}, timeout=10)
    except Exception as e:
        # keep ignore behavior
        print("Callback answer failed (ignored):", e)


# ========================= UI BUILDERS ========================
def build_keyboard():
    return {
        "inline_keyboard": [
            [{"text": "✅ False Alarm! Don't Take Action!", "callback_data": "false_alarm"}],
            [{"text": "🚑 Call Ambulance!", "callback_data": "call_ambulance"}],
        ]
    }

def build_alert_text(axis_change, peak_g, lying_s, fall_alt):
    return (
        "🚨 *FALL DETECTED!*\n\n"
        f"📌 *Details:*\n"
        f"- 🔄 Rotation detected.\n"
        f"- 🧭 Fall axis: {axis_change}\n"
        f"- 💥 Peak acceleration: {peak_g:.2f} g\n"
        f"- 🧍‍♂️Lying duration: {lying_s} seconds\n"
        f"- 📏 Fall distance: {fall_alt}m\n"
    )


# ========================= PARSERS ========================
def parse_machine_fall_line(line: str):
    m = re.search(
        r"FALL\s+axis=([+-][XYZ])\s*->\s*([+-][XYZ])\s+"
        r"peak_g=([0-9]*\.?[0-9]+)\s+lying_s=(\d+)\s+"
        r"fall_alt=([-+]?(?:nan|[0-9]*\.?[0-9]+))",
        line, re.IGNORECASE
    )
    if not m:
        return None

    axis_from = m.group(1)
    axis_to = m.group(2)

    fall_alt_str = m.group(5).lower()
    fall_alt = float("nan") if fall_alt_str == "nan" else float(fall_alt_str)

    return {
        "axis_change": f"{axis_from} -> {axis_to}",
        "peak_g": float(m.group(3)),
        "lying_s": int(m.group(4)),
        "fall_alt": fall_alt,
    }

def parse_imu_csv(line: str):
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 8:
        return None
    try:
        return {
            "t_ms": int(parts[0]),
            "ax": float(parts[1]), "ay": float(parts[2]), "az": float(parts[3]),
            "gx": float(parts[4]), "gy": float(parts[5]), "gz": float(parts[6]),
            "fall_alt": float(parts[7]),
        }
    except ValueError:
        return None


# ========================= Reminder State ========================
@dataclass
class PendingAlert:
    chat_id: int
    original_message_id: int
    original_text: str
    created_ts: float
    cancel_event: threading.Event
    base_lying_s: int
    reminder_count: int = 0
    reminder_message_ids: list[int] | None = None

PENDING_LOCK = threading.Lock()
PENDING_ALERTS: dict[int, PendingAlert] = {}

def start_reminder_loop(chat_id: int, message_id: int, original_text: str, base_lying_s: int, created_ts: float | None = None):
    cancel_event = threading.Event()
    alert = PendingAlert(
        chat_id=chat_id,
        original_message_id=message_id,
        original_text=original_text,
        created_ts=(created_ts if created_ts is not None else time.time()),
        cancel_event=cancel_event,
        base_lying_s=base_lying_s,
        reminder_count=0,
        reminder_message_ids=[],
    )

    with PENDING_LOCK:
        prev = PENDING_ALERTS.get(chat_id)
        if prev:
            prev.cancel_event.set()
        PENDING_ALERTS[chat_id] = alert

    def worker():
        while not cancel_event.is_set():
            time.sleep(REMIND_EVERY_SECONDS)
            if cancel_event.is_set():
                break

            with PENDING_LOCK:
                current = PENDING_ALERTS.get(chat_id)
                if current is None or current.original_message_id != message_id:
                    return

                current.reminder_count += 1
                rc = current.reminder_count
                floor_s = int(current.base_lying_s + (time.time() - current.created_ts))

            if rc > MAX_REMINDERS:
                cancel_event.set()
                try:
                    send_message("⛔ No response received. Reminders stopped.")
                except Exception:
                    pass
                return

            try:
                resp = send_message(
                    f"⏰ Reminder: no response yet.\n"
                    f"🧍‍♂️Time on floor: {floor_s} seconds\n\n"
                    "Please select a button:\n- ✅ False Alarm!\n- 🚑 Call Ambulance",
                    reply_markup=build_keyboard()
                )
                reminder_mid = resp["result"]["message_id"]

                with PENDING_LOCK:
                    current = PENDING_ALERTS.get(chat_id)
                    if current is not None and current.original_message_id == message_id:
                        current.reminder_message_ids.append(reminder_mid)

            except Exception as e:
                print("Reminder send failed:", e)

    t = threading.Thread(target=worker, daemon=True)
    t.start()

def resolve_pending_alert(chat_id: int, resolution_text: str):
    with PENDING_LOCK:
        alert = PENDING_ALERTS.get(chat_id)
        if not alert:
            return None
        alert.cancel_event.set()
        PENDING_ALERTS.pop(chat_id, None)

    try:
        edit_message_text(
            alert.chat_id,
            alert.original_message_id,
            alert.original_text.rstrip() + "\n\n" + resolution_text,
            reply_markup={"inline_keyboard": []}
        )
    except Exception as e:
        print("Failed to edit original alert message:", e)

    for mid in (alert.reminder_message_ids or []):
        ok = delete_message(alert.chat_id, mid)
        if not ok:
            try:
                edit_message_reply_markup(alert.chat_id, mid, reply_markup={"inline_keyboard": []})
            except Exception:
                pass

    return alert


# ========================= TELEGRAM UPDATES LOOP ========================
def telegram_updates_loop():
    offset = None
    backoff = 1.0   # ← ADD THIS LINE (before while True)

    while True:
        params = {"timeout": 30}
        if offset is not None:
            params["offset"] = offset

        try:
            r = SESSION.get(f"{API}/getUpdates", params=params, timeout=35)
            backoff = 1.0  # reset backoff after success

        except Exception as e:
            print("getUpdates network error:", e)
            time.sleep(backoff + random.uniform(0, 0.25))
            backoff = min(backoff * 2, 30.0)  # exponential backoff (max 30s)
            continue

        if not r.ok:
            time.sleep(backoff)
            backoff = min(backoff * 2, 30.0)
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
            if chat_id is None:
                continue

            clicked_mid = msg.get("message_id")

            # ---- EXPIRY GATE ----
            with PENDING_LOCK:
                active = PENDING_ALERTS.get(chat_id)

            if not active:
                try:
                    if clicked_mid is not None:
                        edit_message_reply_markup(chat_id, clicked_mid, reply_markup={"inline_keyboard": []})
                except Exception:
                    pass
                continue

            valid_ids = {active.original_message_id} | set(active.reminder_message_ids or [])
            if clicked_mid is not None and clicked_mid not in valid_ids:
                try:
                    edit_message_reply_markup(chat_id, clicked_mid, reply_markup={"inline_keyboard": []})
                except Exception:
                    pass
                continue
            # ----------------------

            if data == "false_alarm":
                resolve_pending_alert(chat_id, "✅ User pressed False Alarm. No action will be done.")
                send_to_stm32("BEEP FA")
                try:
                    send_message("✅ Acknowledged: False alarm.")
                except Exception:
                    pass

            elif data == "call_ambulance":
                resolve_pending_alert(chat_id, f"🚑 User requested contact. Call: {AMBULANCE_NUMBER}")
                send_to_stm32("BEEP 995")
                try:
                    send_message(f"🚑 Call the Ambulance: {AMBULANCE_NUMBER}")
                except Exception:
                    pass

        time.sleep(0.1)


# ================= UART + Video pipeline =================
def write_buffer_to_csv(rows, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)

def run_postprocess(csv_file: Path):
    try:
        frames_dir = (FRAMES_ROOT / csv_file.stem)
        VIDEOS_ROOT.mkdir(parents=True, exist_ok=True)
        mp4_out = (VIDEOS_ROOT / f"{csv_file.stem}.mp4")

        subprocess.run(
            [sys.executable, str(CSV_TO_FRAMES), str(csv_file), str(frames_dir)],
            check=True,
        )
        print(f"Frames saved -> {frames_dir}")

        mp4_from_csvtoframes = frames_dir / "out.mp4"
        if mp4_from_csvtoframes.exists():
            # move/copy into videos_out with the right name
            VIDEOS_ROOT.mkdir(parents=True, exist_ok=True)
            mp4_out = (VIDEOS_ROOT / f"{csv_file.stem}.mp4")
            try:
                os.replace(mp4_from_csvtoframes, mp4_out)  # atomic move if same drive
            except Exception:
                import shutil
                shutil.copy2(mp4_from_csvtoframes, mp4_out)
            print(f"Rendered video -> {mp4_out}")
            return str(mp4_out)

        print("Video postprocess failed: out.mp4 not found")
        return None

    except Exception as e:
        print("Video postprocess failed:", e)
        return None

def uart_loop():
    global last_telegram_sent, pending_video_upload, post_capture_started_ts

    # Capture half pre-trigger + half post-trigger:
    pre_trigger_len = BUFFER_LEN // 2
    post_trigger_samples_needed = BUFFER_LEN - pre_trigger_len

    buf = deque(maxlen=pre_trigger_len)
    post_trigger_buffer = []
    capturing_post = False

    global SER
    ser = SER
    print(f"Listening on {ser.port} @ {BAUD}...")
    time.sleep(5)
    send_to_stm32("BEEP FA")
    last_video_path = None

    while True:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        # 1) Parse IMU CSV samples for video buffer
        sample = parse_imu_csv(line)
        if sample is not None:
            buf.append(sample)

            if capturing_post:
                post_trigger_buffer.append(sample)

                enough = (len(post_trigger_buffer) >= post_trigger_samples_needed)
                timed_out = (
                    post_capture_started_ts is not None
                    and (time.time() - post_capture_started_ts) >= POST_CAPTURE_TIMEOUT_S
                )

                if enough or timed_out:
                    combined = list(buf) + post_trigger_buffer

                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
                    csv_path = OUTPUT_DIR / f"fall_{ts}.csv"

                    write_buffer_to_csv(combined, csv_path)
                    last_video_path = run_postprocess(csv_path)

                    capturing_post = False
                    post_trigger_buffer = []
                    post_capture_started_ts = None

                    # if alert already sent earlier, upload video now
                    if pending_video_upload and last_video_path and os.path.exists(last_video_path):
                        try:
                            send_video(last_video_path)
                        except Exception as e:
                            print("Video upload failed (delayed):", e)
                        pending_video_upload = False
                        
                        global pending_reminder_start
                        if pending_reminder_start is not None:
                            chat_id, message_id, alert_text, base_lying, alert_created_ts = pending_reminder_start
                            start_reminder_loop(chat_id, message_id, alert_text, base_lying, created_ts=alert_created_ts)
                            pending_reminder_start = None

            continue  # important: don't treat IMU lines as triggers

        # Assist text trigger
        if ASSIST_TEXT in line:
            now = time.time()

            global assist_last_seen_ts
            if now - assist_last_seen_ts >= ASSIST_MIN_INTERVAL:
                send_message("🚨 " + ASSIST_TEXT)
                assist_last_seen_ts = now
            else:
                print("[ASSIST] Suppressed duplicate press")

            continue

        # 2) Trigger handling (non-CSV lines)
        is_video_trigger = ("!!! FALL DETECTED !!!" in line) or ("FALL DETECTED" == line)
        is_data_trigger = ("FALL axis=" in line)

        if not (is_video_trigger or is_data_trigger):
            print("[STM32]", line)
            continue

        print("Trigger line:", line)

        # VIDEO TRIGGER: start post-capture
        if is_video_trigger:
            capturing_post = True
            post_trigger_buffer = []
            post_capture_started_ts = time.time()
            continue

        # DATA TRIGGER: send Telegram
        if is_data_trigger:
            fall_info = parse_machine_fall_line(line)
            if not fall_info:
                continue

            now = time.time()
            if now - last_telegram_sent < TELEGRAM_COOLDOWN_S:
                continue

            alert_text = build_alert_text(
                fall_info["axis_change"],
                fall_info["peak_g"],
                fall_info["lying_s"],
                fall_info["fall_alt"] or 0
            )

            resp = send_message(alert_text, reply_markup=build_keyboard())

            try:
                chat_id = resp["result"]["chat"]["id"]
                message_id = resp["result"]["message_id"]
                base_lying = fall_info["lying_s"] or 0
                start_reminder_loop(chat_id, message_id, alert_text, base_lying)
            except Exception as e:
                print("Failed to start reminder loop (ignored):", e)

            if last_video_path and os.path.exists(last_video_path):
                try:
                    send_video(last_video_path)
                except Exception as e:
                    print("Video upload failed:", e)
            else:
                # mp4 not ready yet -> upload when post-capture finishes
                pending_video_upload = True

            last_telegram_sent = now


def main():
    global SER
    port = auto_detect_port()
    SER = serial.Serial(port, BAUD, timeout=TIMEOUT_S)

    t = threading.Thread(target=telegram_updates_loop, daemon=True)
    t.start()

    try:
        uart_loop()   # blocking loop
    except KeyboardInterrupt:
        print("\nCtrl-C received, exiting...")
    finally:
        # Close serial if open
        try:
            if SER is not None and SER.is_open:
                SER.close()
        except Exception:
            pass

        # Close HTTP session (free sockets)
        try:
            SESSION.close()
        except Exception:
            pass

        print("Shutdown complete.")


if __name__ == "__main__":
    main()
