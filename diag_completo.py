#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DIAGNOSTICO COMPLETO - Suzuki Baleno 2019+
==========================================
Captura datos crudos del ECU para determinar:
  1. Byte mapping correcto de Mode 21 0x00
  2. Que PIDs individuales 21xx retornan datos
  3. Reading de DTCs via multiples metodos
  4. Formato de respuesta CAN

USAGE:
  1. Conecta el adaptador ELM327 al auto (sin encender motor)
  2. Ejecuta: python diag_completo.py
  3. Sigue las instrucciones en pantalla
  4. Comparte el output completo
"""

import serial
import serial.tools.list_ports
import time
import sys
import os

PORT = "COM4"
BAUD = 38400

def send_cmd(ser, cmd, timeout=3.0):
    """Envia comando y lee respuesta completa."""
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

def hex_dump(data, prefix=""):
    """Dump hex con offset labels."""
    hex_str = " ".join(f"{b:02X}" for b in data)
    return f"{prefix}{hex_str}"

def parse_isotp(raw_response):
    """Parse ISO-TP response from ELM327."""
    frames = []
    for line in raw_response.replace("\r", "\n").split("\n"):
        line = line.strip().replace(">", "").strip()
        if not line:
            continue
        parts = line.split()
        if not parts:
            continue
        if parts[0].upper() == "7E8":
            hex_bytes = "".join(parts[1:])
            if hex_bytes:
                try:
                    frame_data = bytes.fromhex(hex_bytes)
                    frames.append(frame_data)
                except ValueError:
                    pass

    if not frames:
        # Try raw hex without headers
        clean = raw_response.replace(">", "").replace("\r", " ").replace("\n", " ").strip()
        tokens = clean.split()
        all_hex = "".join(tokens)
        try:
            return bytes.fromhex(all_hex)
        except ValueError:
            return b""

    first = frames[0]
    pci_type = (first[0] >> 4) & 0x0F

    if pci_type == 0:
        sf_len = first[0] & 0x0F
        return first[1:1 + sf_len]
    elif pci_type == 1:
        total_len = ((first[0] & 0x0F) << 8) | first[1]
        payload = bytearray(first[2:])
        for cf in frames[1:]:
            if (cf[0] >> 4) & 0x0F == 2:
                payload.extend(cf[1:])
            if len(payload) >= total_len:
                break
        return bytes(payload[:total_len])
    elif pci_type == 2:
        payload = bytearray(first[1:])
        for cf in frames[1:]:
            if (cf[0] >> 4) & 0x0F == 2:
                payload.extend(cf[1:])
        return bytes(payload)

    return b"".join(frames)

def main():
    print("=" * 70)
    print("  DIAGNOSTICO COMPLETO - Suzuki Baleno 2019+ (Bosch MEDC17)")
    print("=" * 70)

    # Buscar adaptador
    ports = serial.tools.list_ports.comports()
    print("\n[STEP 0] Puertos COM disponibles:")
    for p in ports:
        marker = " <--" if p.device == PORT else ""
        print(f"  {p.device}: {p.description}{marker}")

    print(f"\n[STEP 1] Conectando a {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1, write_timeout=1)
    except Exception as e:
        print(f"  ERROR: {e}")
        print("  Verifica que el adaptador este conectado.")
        sys.exit(1)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("  OK")

    # ====== FASE 1: INIT ======
    print("\n" + "=" * 70)
    print("  FASE 1: INICIALIZACION ELM327")
    print("=" * 70)

    init_cmds = [
        ("ATZ", "Reset"),
        (None, "Espera 2s post-reset"),
        ("ATE0", "Echo OFF"),
        ("ATL1", "Linefeeds ON"),
        ("ATS1", "Spaces ON"),
        ("ATH1", "Headers ON (CRITICAL para ISO-TP)"),
        ("ATSP6", "Protocol CAN 11bit 500k"),
        ("ATCAF1", "CAN Auto Format ON"),
        ("ATCFC1", "CAN Flow Control ON"),
        ("ATST96", "Timeout 960ms"),
        ("ATSH7E0", "CAN Header = ECM (7E0)"),
        ("ATI", "Device ID"),
        ("ATRV", "Battery Voltage"),
    ]

    for cmd, desc in init_cmds:
        if cmd is None:
            print(f"  {desc}...")
            time.sleep(2.0)
            continue
        r = send_cmd(ser, cmd, timeout=2.0)
        status = "OK" if ("OK" in r or "ELM" in r or "V" in r) else "FAIL"
        print(f"  {cmd:12s} ({desc}): [{status}] {r.strip()[:60]}")
        time.sleep(0.1)

    # ====== FASE 2: TEST 2100 ENGINE OFF ======
    print("\n" + "=" * 70)
    print("  FASE 2: MODE 21 0x00 - MOTOR APAGADO")
    print("  >>>>>> Asegurate de que el motor este APAGADO <<<<<<")
    print("=" * 70)

    input("  Presiona ENTER cuando el motor este APAGADO...")

    for attempt in range(3):
        print(f"\n  --- Intento {attempt+1}/3 ---")
        r = send_cmd(ser, "2100", timeout=5.0)
        print(f"  Response raw ({len(r)} chars): {r.strip()[:200]}")

        data = parse_isotp(r)
        if data and len(data) > 2:
            print(f"  ISO-TP payload: {len(data)} bytes")
            print(f"  SID: 0x{data[0]:02X} (esperado: 0x61)")
            print(f"  PID: 0x{data[1]:02X} (esperado: 0x00)")
            print(f"\n  {'Offset':>6s}  {'Hex':>4s}  {'Dec':>5s}  {'Bin':>8s}  Note")
            print(f"  {'-'*6}  {'-'*4}  {'-'*5}  {'-'*8}  ----")
            for i, b in enumerate(data[2:], start=2):
                note = ""
                if b == 0xFF:
                    note = "(default/unused)"
                elif i == 6:
                    note = "<<< RPM_HIGH (SDL addr 0x04)"
                elif i == 7:
                    note = "<<< RPM_LOW (SDL addr 0x05)"
                elif i == 8:
                    note = "<<< TARGET_IDLE (SDL addr 0x06)"
                elif i == 9:
                    note = "<<< VSS/SPEED (SDL addr 0x07)"
                elif i == 10:
                    note = "<<< ECT (SDL addr 0x08)"
                elif i == 11:
                    note = "<<< IAT (SDL addr 0x09)"
                elif i == 12:
                    note = "<<< TPS_ANGLE (SDL addr 0x0A)"
                elif i == 13:
                    note = "<<< TPS_VOLTAGE (SDL addr 0x0B)"
                elif i == 15:
                    note = "<<< INJ_HI (SDL addr 0x0D)"
                elif i == 16:
                    note = "<<< INJ_LO (SDL addr 0x0E)"
                elif i == 17:
                    note = "<<< IGNITION (SDL addr 0x0F)"
                elif i == 18:
                    note = "<<< MAP (SDL addr 0x10)"
                elif i == 19:
                    note = "<<< BARO (SDL addr 0x11)"
                elif i == 20:
                    note = "<<< ISC (SDL addr 0x12)"
                elif i == 24:
                    note = "<<< BATTERY (SDL addr 0x16)"
                elif i == 27:
                    note = "<<< RADIATOR_FAN (SDL addr 0x19)"
                elif i == 28:
                    note = "<<< STATUS_FLAGS (SDL addr 0x1A)"
                print(f"  {i:6d}  0x{b:02X}  {b:5d}  {b:08b}  {note}")
        else:
            print(f"  ISO-TP parse failed. Raw tokens:")
            for line in r.strip().split("\r"):
                line = line.strip()
                if line and line != ">":
                    print(f"    {line}")

    # ====== FASE 3: TEST 2100 MOTOR ON ======
    print("\n" + "=" * 70)
    print("  FASE 3: MODE 21 0x00 - MOTOR ENCENDIDO")
    print("  >>>>>> Ahora enciende el motor y deja idle <<<<<<")
    print("=" * 70)

    input("  Presiona ENTER despues de encender el motor y esperar 5 segundos...")

    for attempt in range(5):
        r = send_cmd(ser, "2100", timeout=5.0)
        data = parse_isotp(r)
        if data and len(data) > 2:
            print(f"\n  [Motor ON {attempt+1}] {len(data)} bytes")
            # Mostrar solo los bytes que cambiaron de 0xFF
            for i, b in enumerate(data[2:], start=2):
                note = ""
                if i == 6: note = "RPM_HIGH"
                elif i == 7: note = "RPM_LOW"
                elif i == 9: note = "SPEED"
                elif i == 10: note = "ECT"
                elif i == 11: note = "IAT"
                elif i == 12: note = "TPS"
                elif i == 17: note = "IGN"
                elif i == 18: note = "MAP"
                elif i == 24: note = "BAT"
                if b != 0xFF or (i >= 6 and i <= 28):
                    print(f"    [{i:3d}] 0x{b:02X} = {b:3d}  {note}")
        time.sleep(1.0)

    # ====== FASE 4: SCAN DE PIDs Mode 21 INDIVIDUALES ======
    print("\n" + "=" * 70)
    print("  FASE 4: SCAN DE PIDs Mode 21 INDIVIDUALES")
    print("=" * 70)

    pids_to_test = [
        "2100", "2101", "2102", "2103", "2104", "2105",
        "2106", "2107", "2108", "2109", "210A", "210B",
        "210C", "210D", "210E", "210F",
        "2127", "2128", "2129", "212A", "212B", "212C",
        "2130", "2131", "2132", "2133", "2134", "2135", "2136",
        "213C", "213D", "213E", "213F",
        "2140", "2141", "2142", "2143", "2144", "2145",
        "2146", "2147", "2148", "2149",
        "21A2", "21D0",
    ]

    print("  Probando cada PID...")
    for pid in pids_to_test:
        r = send_cmd(ser, pid, timeout=2.0)
        data = parse_isotp(r)
        if data and len(data) >= 2:
            print(f"  {pid}: OK - {len(data)} bytes - SID=0x{data[0]:02X} PID=0x{data[1]:02X} - {data[2:10].hex(' ').upper()}")
        else:
            has_7f = "7F" in r
            has_no_data = "NO DATA" in r
            status = "NRC(7F)" if has_7f else ("NO DATA" if has_no_data else "NO RESPONSE")
            print(f"  {pid}: {status}")
        time.sleep(0.3)

    # ====== FASE 5: DTCs ======
    print("\n" + "=" * 70)
    print("  FASE 5: LECTURA DE DTCs (multiples metodos)")
    print("=" * 70)

    dtc_methods = [
        ("03", "Mode 03 - OBD2 standard DTCs"),
        ("07", "Mode 07 - Pending DTCs"),
        ("0A", "Mode 0A - Permanent DTCs"),
        ("190200", "UDS 0x19 0x02 - ReportDTCByStatusMask (all)"),
        ("190A00", "UDS 0x19 0x0A - ReportSupportedDTC"),
        ("1800FF", "Mode 18 - Suzuki extended DTCs (sub 00, status FF)"),
        ("1802FF", "Mode 18 - Suzuki extended DTCs (sub 02, snapshot)"),
    ]

    for cmd, desc in dtc_methods:
        print(f"\n  --- {desc} ---")
        print(f"  Sending: {cmd}")
        r = send_cmd(ser, cmd, timeout=3.0)
        print(f"  Response: {r.strip()[:300]}")

        data = parse_isotp(r)
        if data and len(data) > 0:
            print(f"  Parsed: {len(data)} bytes - {data.hex(' ').upper()}")
            # Try to decode DTCs from the response
            if len(data) >= 3:
                resp_mode = data[0]
                if resp_mode == 0x43:
                    # DTC response
                    dtc_data = data[2:]  # Skip mode + count
                    for i in range(0, len(dtc_data) - 1, 2):
                        b1, b2 = dtc_data[i], dtc_data[i+1]
                        if b1 == 0 and b2 == 0:
                            continue
                        cat = {0x00: "P", 0x40: "C", 0x80: "B", 0xC0: "U"}.get(b1 & 0xC0, "?")
                        d2 = (b1 >> 4) & 0x03
                        d3 = b1 & 0x0F
                        d4 = (b2 >> 4) & 0x0F
                        d5 = b2 & 0x0F
                        code = f"{cat}{d2}{d3:X}{d4:X}{d5:X}"
                        print(f"    DTC: {code}")
                elif resp_mode == 0x59:
                    print(f"    UDS Response sub={data[1]:02X}")
                    if len(data) > 4:
                        for i in range(4, len(data) - 1, 2):
                            b1, b2 = data[i], data[i+1]
                            if b1 == 0 and b2 == 0:
                                continue
                            cat = {0x00: "P", 0x40: "C", 0x80: "B", 0xC0: "U"}.get(b1 & 0xC0, "?")
                            d2 = (b1 >> 4) & 0x03
                            d3 = b1 & 0x0F
                            d4 = (b2 >> 4) & 0x0F
                            d5 = b2 & 0x0F
                            code = f"{cat}{d2}{d3:X}{d4:X}{d5:X}"
                            print(f"    DTC: {code}")
        time.sleep(0.5)

    # ====== FASE 6: ECU ID ======
    print("\n" + "=" * 70)
    print("  FASE 6: IDENTIFICACION ECU")
    print("=" * 70)

    ecu_cmds = [
        ("21D0", "Mode 21 0xD0 - ECU Identification"),
        ("0902", "Mode 09 0x02 - VIN"),
        ("1003", "UDS 0x10 0x03 - Extended Diagnostic Session"),
        ("22F187", "UDS 0x22 - ECU Serial Number (F187)"),
        ("22F189", "UDS 0x22 - Vehicle Manufacturer ECU HW Number (F189)"),
        ("22F18A", "UDS 0x22 - Vehicle Manufacturer ECU SW Number (F18A)"),
    ]

    for cmd, desc in ecu_cmds:
        r = send_cmd(ser, cmd, timeout=3.0)
        data = parse_isotp(r)
        if data and len(data) > 2:
            # Try to decode as ASCII
            ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in data[2:])
            print(f"  {desc}: {ascii_str[:80]}")
            print(f"    Raw: {data[2:20].hex(' ').upper()}")
        else:
            print(f"  {desc}: {r.strip()[:100]}")
        time.sleep(0.3)

    # ====== FASE 7: TEST ATSH PARA OTROS MODULOS ======
    print("\n" + "=" * 70)
    print("  FASE 7: SCAN DE MODULOS (ATSH switching)")
    print("=" * 70)

    modules = [
        (0x7E0, "ECM (Motor)"),
        (0x7E1, "TCM (Transmision)"),
        (0x7E2, "ABS/ESP"),
        (0x7E3, "BCM (Carroceria)"),
        (0x7E4, "EPS (Direccion)"),
        (0x7E5, "SRS (Airbags)"),
        (0x7D0, "Inmovilizador"),
    ]

    for tx_id, name in modules:
        send_cmd(ser, f"ATSH{tx_id:03X}", timeout=1.0)
        time.sleep(0.1)
        r = send_cmd(ser, "21D0", timeout=2.0)
        data = parse_isotp(r)
        if data and len(data) > 2:
            ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
            print(f"  [0x{tx_id:03X}] {name}: RESPONDE - {ascii_str[:60]}")
        else:
            no_data = "NO DATA" in r or "7F" in r
            print(f"  [0x{tx_id:03X}] {name}: {'no responde' if no_data else 'respuesta corta'}")
        time.sleep(0.2)

    # Restore ECM header
    send_cmd(ser, "ATSH7E0", timeout=1.0)

    ser.close()
    print("\n" + "=" * 70)
    print("  DIAGNOSTICO COMPLETADO")
    print("  Copia TODO el output y compartelo.")
    print("=" * 70)


if __name__ == "__main__":
    main()
