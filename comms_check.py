# ========== This is a serial handshake test between PC and STM32 ==========
import serial
import time
from serial.tools import list_ports

PORT = "COM3"
BAUD = 115200

# Auto detects the port
def auto_detect_port() -> str:
    ports = list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        if "stm" in desc or "usb serial" in desc:
            print(f"Auto-detected port: {p.device} ({p.description})")
            return p.device
    if ports:
        print("No STM match. Using first port:", ports[0].device)
        return ports[0].device
    raise RuntimeError("No serial ports found.")

# Sends a line of tet to STM32
def send(ser, msg):
    ser.write((msg.strip() + "\n").encode("utf-8"))
    ser.flush()
    print(f"[PC → STM32] {msg.strip()}")

def main():
    port = auto_detect_port()
    ser = serial.Serial(port, BAUD, timeout=10)  # 10s timeout to catch startup message
    print(f"Opened {port} @ {BAUD}")
    print("Waiting for STM32 startup message...")

    # Step 1: Wait for initial message from STM32
    while True:
        raw = ser.readline()
        if not raw:
            print("Timed out waiting for STM32. Is it powered?")
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(f"[STM32 → PC] {line}")

        # Step 2: Acknowledge the startup message
        if "ready" in line.lower():
            send(ser, "ACK: Python ready")
            break  # move on to sending data

    # Small gap before sending data to prevent overlapping UART message
    time.sleep(0.5)

    # Step 3: Send actual data
    send(ser, "BEEP FA")

    # Step 4: Listen for STM32's acknowledgement of the data
    print("Waiting for STM32 acknowledgement...")
    ser.timeout = 5  # tighter timeout now
    while True:
        raw = ser.readline()
        if not raw:
            print("No response from STM32.")
            break

        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(f"[STM32 → PC] {line}")
            if "ACK" in line:
                print("Handshake complete.")
                break

if __name__ == "__main__":
    main()
