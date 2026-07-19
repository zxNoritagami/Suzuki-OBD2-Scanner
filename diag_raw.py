import serial
import time
import sys

PORT = "COM4"
BAUD = 38400

def send_cmd(ser, cmd, timeout=3.0):
    ser.write((cmd + "\r").encode())
    ser.flush()
    lines = []
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode("ascii", errors="ignore")
            lines.append(line)
            if ">" in line:
                break
        time.sleep(0.01)
    return "".join(lines)

def main():
    print(f"=== DIAG RAW BYTES - {PORT} @ {BAUD} ===")
    print("Conectando...")

    ser = serial.Serial(PORT, BAUD, timeout=1, write_timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Init
    for cmd in ["ATZ", "ATE0", "ATL1", "ATS1", "ATH1", "ATSP6", "ATCAF1", "ATCFC1", "ATSTFF"]:
        r = send_cmd(ser, cmd, timeout=2.0)
        print(f"  {cmd}: {r.strip()[:80]}")
        time.sleep(0.1)

    print("\n--- Leyendo 2100 (3 veces con motor OFF) ---")
    for i in range(3):
        r = send_cmd(ser, "2100", timeout=5.0)
        print(f"\n[Iter {i+1}] Response ({len(r)} chars):")
        print(r.strip())
        time.sleep(1)

    print("\n=== Ahora enciende el motor y espera 5 segundos, luego presiona ENTER ===")
    input(">>> ")

    print("--- Leyendo 2100 con motor ON (5 veces) ---")
    for i in range(5):
        r = send_cmd(ser, "2100", timeout=5.0)
        print(f"\n[Motor ON {i+1}] Response:")
        print(r.strip())
        time.sleep(1)

    # Parsear bytes
    print("\n=== PARSEO DE BYTES ===")
    r = send_cmd(ser, "2100", timeout=5.0)
    # Extract hex bytes from response
    all_hex = ""
    for line in r.strip().split("\r"):
        line = line.strip().replace(" ", "")
        if not line or line == ">":
            continue
        # Lines starting with 7E8 are CAN frames
        if line.startswith("7E8"):
            # Format: 7E8[nn] [data bytes...]
            # With ATH1, format is like: 7E8 06 61 00 ...
            parts = line[3:].strip()
            all_hex += parts
        elif line.startswith("61"):
            all_hex += line

    if all_hex:
        raw = bytes.fromhex(all_hex)
        print(f"Total bytes: {len(raw)}")
        for idx, b in enumerate(raw):
            label = ""
            if idx == 0: label = " <- SID"
            elif idx == 1: label = " <- PID"
            else:
                d = idx - 2
                if d == 0: label = " <- Data[0]"
                else: label = f" <- Data[{d}]"
            print(f"  [{idx:3d}] 0x{b:02X} = {b:3d}{label}")

    print("\n=== PRUEBA DE OTROS PIDs ===")
    for pid in ["2101", "2102", "2103", "2104", "2105"]:
        r = send_cmd(ser, pid, timeout=3.0)
        print(f"\n{pid}: {r.strip()[:200]}")
        time.sleep(0.5)

    ser.close()
    print("\n=== DIAG COMPLETADO ===")

if __name__ == "__main__":
    main()
