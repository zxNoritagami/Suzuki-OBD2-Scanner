#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
SUZUKI OBD2 DIAGNOSTIC SCANNER PRO v1.0.0
================================================================================
Aplicacion de diagnostico OBD2 avanzada para vehiculos Suzuki (Baleno 2019+)
Soporte: CAN (ISO 15765-4), UDS (ISO 14229), ELM327 USB (CH341 driver)

DEPENDENCIAS:
    pip install pyserial customtkinter pillow

USO:
    python suzuki_obd2_scanner.py
    python suzuki_obd2_scanner.py --simulate
================================================================================
"""

import sys
import os
import time
import threading
import queue
import json
import re
import struct
import argparse
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Callable, Tuple, Any, Union
from enum import Enum, auto
from collections import deque
import traceback

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial no instalado. Ejecuta: pip install pyserial")
    sys.exit(1)

try:
    import customtkinter as ctk
    from customtkinter import CTk, CTkFrame, CTkLabel, CTkButton, CTkEntry
    from customtkinter import CTkTextbox, CTkOptionMenu, CTkSwitch, CTkProgressBar
    from customtkinter import CTkTabview, CTkScrollableFrame, CTkCheckBox
except ImportError:
    print("[ERROR] customtkinter no instalado. Ejecuta: pip install customtkinter")
    sys.exit(1)

APP_NAME = "Suzuki OBD2 Scanner Pro"
APP_VERSION = "1.0.0"
BAUD_RATES = [38400, 115200, 9600, 57600, 230400]
DEFAULT_BAUD = 38400
SERIAL_TIMEOUT = 2.0
COMMAND_TIMEOUT = 5.0

# ==============================================================================
# COLORES DE MARCA (Suzuki Racing Red)
# ==============================================================================
SUZUKI_RED = "#C8102E"
SUZUKI_RED_HOVER = "#A00D24"
SUZUKI_RED_DIM = "#8B0A1F"
COLOR_BG_DARK = "#1A1A2E"
COLOR_BG_CARD = "#16213E"
COLOR_BG_SIDEBAR = "#0F3460"
COLOR_TEXT_PRIMARY = "#E8E8E8"
COLOR_TEXT_SECONDARY = "#8899AA"
COLOR_GREEN = "#2ECC71"
COLOR_YELLOW = "#F1C40F"
COLOR_RED = "#E74C3C"
COLOR_ORANGE = "#E67E22"
COLOR_BORDER = "#2A2A4A"

class OBDProtocol(Enum):
    AUTO = "0"
    SAE_J1850_PWM = "1"
    SAE_J1850_VPW = "2"
    ISO_9141_2 = "3"
    ISO_14230_4_5BAUD = "4"
    ISO_14230_4_FAST = "5"
    ISO_15765_4_11BIT_500K = "6"
    ISO_15765_4_29BIT_500K = "7"
    ISO_15765_4_11BIT_250K = "8"
    ISO_15765_4_29BIT_250K = "9"
    SAE_J1939 = "A"
    USER1_CAN_11BIT_125K = "B"
    USER2_CAN_11BIT_50K = "C"

class SuzukiModule(Enum):
    ECM = (0x7E0, 0x7E8, "Motor", "Powertrain CAN")
    TCM = (0x7E1, 0x7E9, "Transmision", "Powertrain CAN")
    ABS = (0x7E2, 0x7EA, "ABS/ESP", "Chassis CAN")
    BCM = (0x7E3, 0x7EB, "Carroceria", "Body CAN")
    EPS = (0x7E4, 0x7EC, "Direccion", "Chassis CAN")
    SRS = (0x7E5, 0x7ED, "Airbags", "Safety CAN-UDS")
    HVAC = (0x7E6, 0x7EE, "Climatizador", "Body CAN")
    CLUSTER = (0x7E7, 0x7EF, "Tablero", "Body CAN")
    IMMO = (0x7D0, 0x7D8, "Inmovilizador", "Body CAN")
    TPMS = (0x7D1, 0x7D9, "Presion Neumaticos", "Body CAN")
    ALH = (0x7D2, 0x7DA, "Auto Locking Hub", "Chassis CAN-UDS")
    DSBS = (0x7D4, 0x7DC, "Frenado Autonomo", "Safety CAN-UDS")
    ALC = (0x7D5, 0x7DD, "Nivelado Luces", "Body CAN")
    CSW = (0x7DC, 0x7E4, "Central Switch", "Body CAN")
    CGW = (0x7DF, 0x7E7, "Central Gateway", "Gateway CAN-UDS")
    def __init__(self, tx_id, rx_id, label, bus):
        self.tx_id = tx_id
        self.rx_id = rx_id
        self.label = label
        self.bus = bus

    @classmethod
    def by_tx(cls, tx_id):
        for m in cls:
            if m.tx_id == tx_id:
                return m
        return None

class UDSService(Enum):
    DIAGNOSTIC_SESSION_CONTROL = 0x10
    ECU_RESET = 0x11
    SECURITY_ACCESS = 0x27
    READ_DATA_BY_IDENTIFIER = 0x22
    READ_DTC_INFORMATION = 0x19
    CLEAR_DIAGNOSTIC_INFORMATION = 0x14
    TESTER_PRESENT = 0x3E

class OBDMode(Enum):
    CURRENT_DATA = "01"
    FREEZE_FRAME = "02"
    STORED_DTC = "03"
    PENDING_DTC = "07"
    PERMANENT_DTC = "0A"
    CLEAR_DTC = "04"
    VEHICLE_INFO = "09"

class ConnectionState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    INITIALIZING = auto()
    CONNECTED = auto()
    RECOVERING = auto()
    ERROR = auto()

class CANBusSpeed(Enum):
    HIGH_SPEED = "500"
    MEDIUM_SPEED = "125"

UDS_NEGATIVE_RESPONSE_CODES = {
    0x10: "General Reject", 0x11: "Service Not Supported",
    0x12: "Sub-function Not Supported", 0x13: "Incorrect Message Length",
    0x21: "Busy Repeat Request", 0x22: "Conditions Not Correct",
    0x31: "Request Out Of Range", 0x33: "Security Access Denied",
    0x35: "Invalid Key", 0x36: "Exceed Number Of Attempts",
    0x78: "Response Pending", 0x7E: "Sub-function Not Supported In Active Session",
    0x7F: "Service Not Supported In Active Session",
    0x81: "Rpm Too High", 0x82: "Rpm Too Low",
    0x83: "Engine Is Running", 0x84: "Engine Is Not Running",
    0x86: "Temperature Too High", 0x87: "Temperature Too Low",
    0x88: "Vehicle Speed Too High", 0x89: "Vehicle Speed Too Low",
    0x92: "Voltage Too High", 0x93: "Voltage Too Low",
}

DTC_CATEGORIES = {"P": "Powertrain", "C": "Chassis", "B": "Body", "U": "Network"}
DTC_PREFIXES = {
    "P0": "SAE Generic", "P1": "Suzuki Specific", "P2": "SAE Generic",
    "C0": "SAE Generic", "C1": "Suzuki Specific",
    "B0": "SAE Generic", "B1": "Suzuki Specific",
    "U0": "SAE Generic", "U1": "Suzuki Specific", "U2": "SAE Generic",
}

@dataclass
class PIDDefinition:
    mode: str
    pid: str
    name: str
    description: str
    bytes_count: int
    unit: str
    decode_func: Callable
    category: str = "Generic"
    module: str = "ECM"
    min_val: float = None
    max_val: float = None

@dataclass
class DTCRecord:
    code: str
    description: str
    category: str
    prefix_type: str
    is_pending: bool = False
    is_permanent: bool = False
    module: str = "ECM"
    freeze_frame: Dict = None

@dataclass
class ConnectionConfig:
    port: str = "COM1"
    baudrate: int = 38400
    protocol: OBDProtocol = None
    can_speed: CANBusSpeed = None
    timeout: float = 2.0
    auto_reconnect: bool = True
    def __post_init__(self):
        if self.protocol is None:
            self.protocol = OBDProtocol.AUTO
        if self.can_speed is None:
            self.can_speed = CANBusSpeed.HIGH_SPEED

def decode_percent_a(data): return data[0] * 100.0 / 255.0
def decode_percent_ab(data): return ((data[0] * 256) + data[1]) * 100.0 / 65535.0
def decode_temp_a(data): return data[0] - 40.0
def decode_temp_ab(data): return ((data[0] * 256) + data[1]) / 10.0 - 40.0
def decode_rpm_ab(data): return ((data[0] * 256) + data[1]) / 4.0
def decode_speed_a(data): return float(data[0])
def decode_timing_a(data): return (data[0] - 128) / 2.0
def decode_pressure_a(data): return data[0] * 3.0
def decode_voltage_ab(data): return ((data[0] * 256) + data[1]) / 1000.0
def decode_maf_ab(data): return ((data[0] * 256) + data[1]) / 100.0
def decode_fuel_trim_a(data): return (data[0] - 128) * 100.0 / 128.0
def decode_distance_ab(data): return (data[0] * 256) + data[1]
def decode_raw_hex(data): return " ".join(f"{b:02X}" for b in data)
def decode_fuel_rate_ab(data): return ((data[0] * 256) + data[1]) * 0.05

# ==============================================================================
# SUZUKI MODE 21 - ISO-TP PARSER
# ==============================================================================
# El Baleno 2019+ NO soporta OBD2 estandar (Mode 01).
# Usa Suzuki-proprietary Mode 21 sobre CAN (7E0/7E8) a 500kbps.
# Las respuestas multi-frame usan ISO-TP (ISO 15765-2).

def parse_isotp_response(raw_response):
    """
    Parsea respuesta ISO-TP multi-frame del ELM327.

    Maneja multiples formatos de salida ELM327 v1.5:
      ATS1 (spaces ON):  "7E8 10 92 61 00 FF FF ..."
      ATS0 (spaces OFF): "7E810926100FFFFFFFF..."
      Sin headers:        "61 00 FF FF ..." o "6100FFFFFFFF..."

    ISO-TP PCI types:
      0x0N: Single Frame, N=data length
      0x1N: First Frame, NNN=data length (12-bit)
      0x2N: Consecutive Frame, N=sequence number
      0x3N: Flow Control

    Retorna: bytes con el payload completo (incluyendo SID+PID).
    """
    # Limpiar respuesta: quitar >, \r, \n, espacios extra
    clean = raw_response.replace(">", "").replace("\r", "").replace("\n", " ").strip()

    # Extraer todos los tokens hex (con o sin headers CAN)
    tokens = clean.split()

    if not tokens:
        return b""

    # Buscar donde empiezan los datos ISO-TP
    # Formato con header CAN: [7E8] [PCI] [len...] [61] [00] [data...]
    # Formato sin header: [61] [00] [data...]

    frames = []

    # Estrategia 1: Buscar lineas que empiecen con 7E8 (con ATH1)
    for line in raw_response.replace("\r", "\n").split("\n"):
        line = line.strip().replace(">", "").strip()
        if not line:
            continue
        parts = line.split()
        if not parts:
            continue

        # Si empieza con 7E8, es un frame CAN
        if parts[0].upper() == "7E8":
            hex_bytes = "".join(parts[1:])  # Todo despues del ID
            if hex_bytes:
                try:
                    frame_data = bytes.fromhex(hex_bytes)
                    frames.append(frame_data)
                except ValueError:
                    pass

    if frames:
        return _reassemble_isotp(frames)

    # Estrategia 2: Sin headers CAN - buscar respuesta directa (61 xx ...)
    all_hex = "".join(tokens)
    try:
        raw_bytes = bytes.fromhex(all_hex)
    except ValueError:
        return b""

    # Verificar si empieza con 61 (Mode 21 response) o 43/47/4A (DTC response)
    if raw_bytes and raw_bytes[0] in (0x61, 0x43, 0x47, 0x4A, 0x59):
        # Respuesta simple (single frame sin PCI)
        return raw_bytes

    # Intentar parsear como ISO-TP desde bytes crudos
    if len(raw_bytes) >= 2:
        pci_type = (raw_bytes[0] >> 4) & 0x0F
        if pci_type == 0:
            # Single frame
            sf_len = raw_bytes[0] & 0x0F
            if sf_len > 0 and len(raw_bytes) >= 1 + sf_len:
                return raw_bytes[1:1 + sf_len]
        elif pci_type == 1:
            # First frame: dividir en frames por PCI y reensamblar
            frames = [raw_bytes]
            idx = 2 + (raw_bytes[0] & 0x0F) * 256 + raw_bytes[1]
            idx = min(idx, len(raw_bytes))
            cf_start = 2
            while cf_start < len(raw_bytes):
                cf = raw_bytes[cf_start:]
                if cf and (cf[0] >> 4) & 0x0F == 2:
                    frames.append(cf)
                    cf_start += 1
                    cf_start += cf[0] & 0x0F if (cf[0] & 0x0F) > 0 else 7
                else:
                    break
            return _reassemble_isotp(frames)
        elif pci_type == 2:
            # Consecutive frame directo: tratar como frame unico
            return _reassemble_isotp([raw_bytes])

    return raw_bytes


def _reassemble_isotp(frames):
    """Reensambla frames ISO-TP en payload completo."""
    if not frames:
        return b""

    first = frames[0]
    if not first:
        return b""

    pci_type = (first[0] >> 4) & 0x0F

    if pci_type == 0:
        # Single Frame: PCI byte = 0x0N where N = data length
        sf_len = first[0] & 0x0F
        return first[1:1 + sf_len] if len(first) >= 1 + sf_len else first[1:]

    if pci_type == 1:
        # First Frame: PCI = 0x1N, length = NNN (12-bit)
        total_len = ((first[0] & 0x0F) << 8) | first[1]
        payload = bytearray(first[2:])

        for cf_frame in frames[1:]:
            pci_cf = (cf_frame[0] >> 4) & 0x0F
            if pci_cf == 2:
                # Consecutive Frame: PCI = 0x2N, skip PCI byte
                payload.extend(cf_frame[1:])
            if len(payload) >= total_len:
                break

        return bytes(payload[:total_len])

    if pci_type == 2:
        # Consecutive Frame directo (sin First Frame)
        payload = bytearray(first[1:])
        for cf_frame in frames[1:]:
            pci_cf = (cf_frame[0] >> 4) & 0x0F
            if pci_cf == 2:
                payload.extend(cf_frame[1:])
            elif pci_cf == 0:
                # Otro single frame
                sf_len = cf_frame[0] & 0x0F
                payload.extend(cf_frame[1:1 + sf_len])
        return bytes(payload)

    # Fallback: concatenar todo
    return b"".join(frames)

GENERIC_PIDS = {
    "0100": PIDDefinition("01", "00", "PIDs_Supported_01_20", "PIDs soportados 01-20", 4, "bitmask", decode_raw_hex),
    "0101": PIDDefinition("01", "01", "DTC_Count", "Numero de DTCs y estado MIL", 4, "count/mask", lambda d: f"DTCs:{d[0]&0x7F}, MIL:{bool(d[0]&0x80)}"),
    "0103": PIDDefinition("01", "03", "Fuel_System_Status", "Estado sistema combustible", 2, "status", decode_raw_hex),
    "0104": PIDDefinition("01", "04", "Engine_Load", "Carga calculada motor", 1, "%", decode_percent_a, min_val=0, max_val=100),
    "0105": PIDDefinition("01", "05", "Coolant_Temp", "Temp. refrigerante", 1, "C", decode_temp_a, min_val=-40, max_val=215),
    "0106": PIDDefinition("01", "06", "Short_Fuel_Trim_B1", "Ajuste corto combustible B1", 1, "%", decode_fuel_trim_a),
    "0107": PIDDefinition("01", "07", "Long_Fuel_Trim_B1", "Ajuste largo combustible B1", 1, "%", decode_fuel_trim_a),
    "0108": PIDDefinition("01", "08", "Short_Fuel_Trim_B2", "Ajuste corto combustible B2", 1, "%", decode_fuel_trim_a),
    "0109": PIDDefinition("01", "09", "Long_Fuel_Trim_B2", "Ajuste largo combustible B2", 1, "%", decode_fuel_trim_a),
    "010A": PIDDefinition("01", "0A", "Fuel_Pressure", "Presion combustible", 1, "kPa", decode_pressure_a),
    "010B": PIDDefinition("01", "0B", "Intake_Manifold_Pressure", "Presion multiple admision", 1, "kPa", lambda d: float(d[0]), min_val=0, max_val=255),
    "010C": PIDDefinition("01", "0C", "Engine_RPM", "Revoluciones motor", 2, "rpm", decode_rpm_ab, min_val=0, max_val=16383.75),
    "010D": PIDDefinition("01", "0D", "Vehicle_Speed", "Velocidad vehiculo", 1, "km/h", decode_speed_a, min_val=0, max_val=255),
    "010E": PIDDefinition("01", "0E", "Timing_Advance", "Avance encendido", 1, "deg", decode_timing_a),
    "010F": PIDDefinition("01", "0F", "Intake_Air_Temp", "Temp. aire admision", 1, "C", decode_temp_a, min_val=-40, max_val=215),
    "0110": PIDDefinition("01", "10", "MAF_Rate", "Flujo aire MAF", 2, "g/s", decode_maf_ab, min_val=0, max_val=655.35),
    "0111": PIDDefinition("01", "11", "Throttle_Position", "Posicion acelerador", 1, "%", decode_percent_a, min_val=0, max_val=100),
    "011C": PIDDefinition("01", "1C", "OBD_Standard", "Estandar OBD", 1, "code", lambda d: f"OBD Std: {d[0]}"),
    "011F": PIDDefinition("01", "1F", "Run_Time", "Tiempo desde arranque", 2, "s", decode_distance_ab, min_val=0, max_val=65535),
    "0120": PIDDefinition("01", "20", "PIDs_Supported_21_40", "PIDs soportados 21-40", 4, "bitmask", decode_raw_hex),
    "0121": PIDDefinition("01", "21", "Distance_MIL_On", "Distancia MIL encendido", 2, "km", decode_distance_ab),
    "0122": PIDDefinition("01", "22", "Fuel_Rail_Pressure", "Presion rail combustible", 2, "kPa", lambda d: ((d[0]*256)+d[1])*0.079),
    "012C": PIDDefinition("01", "2C", "Commanded_EGR", "EGR comandado", 1, "%", decode_percent_a),
    "012D": PIDDefinition("01", "2D", "EGR_Error", "Error EGR", 1, "%", decode_fuel_trim_a),
    "012E": PIDDefinition("01", "2E", "Commanded_Evap_Purge", "Purgado evaporativo", 1, "%", decode_percent_a),
    "012F": PIDDefinition("01", "2F", "Fuel_Level", "Nivel combustible", 1, "%", decode_percent_a),
    "0130": PIDDefinition("01", "30", "Warmups_Since_DTC_Clear", "Calentamientos desde borrado", 1, "count", lambda d: float(d[0])),
    "0131": PIDDefinition("01", "31", "Distance_Since_DTC_Clear", "Distancia desde borrado", 2, "km", decode_distance_ab),
    "0133": PIDDefinition("01", "33", "Barometric_Pressure", "Presion barometrica", 1, "kPa", lambda d: float(d[0])),
    "013C": PIDDefinition("01", "3C", "Catalyst_Temp_B1S1", "Temp. catalizador B1S1", 2, "C", decode_temp_ab),
    "013D": PIDDefinition("01", "3D", "Catalyst_Temp_B2S1", "Temp. catalizador B2S1", 2, "C", decode_temp_ab),
    "013E": PIDDefinition("01", "3E", "Catalyst_Temp_B1S2", "Temp. catalizador B1S2", 2, "C", decode_temp_ab),
    "013F": PIDDefinition("01", "3F", "Catalyst_Temp_B2S2", "Temp. catalizador B2S2", 2, "C", decode_temp_ab),
    "0140": PIDDefinition("01", "40", "PIDs_Supported_41_60", "PIDs soportados 41-60", 4, "bitmask", decode_raw_hex),
    "0142": PIDDefinition("01", "42", "Control_Module_Voltage", "Voltaje modulo", 2, "V", decode_voltage_ab),
    "0143": PIDDefinition("01", "43", "Absolute_Load", "Carga absoluta", 2, "%", decode_percent_ab),
    "0144": PIDDefinition("01", "44", "Commanded_Equivalence_Ratio", "Relacion equivalencia", 2, "ratio", lambda d: ((d[0]*256)+d[1])*2.0/65535.0),
    "0145": PIDDefinition("01", "45", "Relative_Throttle_Position", "Posicion relativa acelerador", 1, "%", decode_percent_a),
    "0146": PIDDefinition("01", "46", "Ambient_Air_Temp", "Temp. ambiente", 1, "C", decode_temp_a),
    "0147": PIDDefinition("01", "47", "Absolute_Throttle_B", "Posicion absoluta acelerador B", 1, "%", decode_percent_a),
    "014C": PIDDefinition("01", "4C", "Commanded_Throttle_Actuator", "Actuador acelerador", 1, "%", decode_percent_a),
    "014D": PIDDefinition("01", "4D", "Time_MIL_On", "Tiempo MIL encendido", 2, "min", decode_distance_ab),
    "014E": PIDDefinition("01", "4E", "Time_Since_DTC_Clear", "Tiempo desde borrado", 2, "min", decode_distance_ab),
    "0151": PIDDefinition("01", "51", "Fuel_Type", "Tipo combustible", 1, "code", lambda d: f"Fuel: {d[0]}"),
    "015C": PIDDefinition("01", "5C", "Engine_Oil_Temp", "Temp. aceite motor", 1, "C", decode_temp_a),
    "015D": PIDDefinition("01", "5D", "Fuel_Injection_Timing", "Tiempo inyeccion", 2, "deg", lambda d: (((d[0]*256)+d[1])/128.0)-210.0),
    "015E": PIDDefinition("01", "5E", "Engine_Fuel_Rate", "Tasa consumo", 2, "L/h", decode_fuel_rate_ab),
    "0160": PIDDefinition("01", "60", "PIDs_Supported_61_80", "PIDs soportados 61-80", 4, "bitmask", decode_raw_hex),
    "0161": PIDDefinition("01", "61", "Driver_Demand_Engine_Torque", "Torque demandado", 1, "%", lambda d: d[0]-125),
    "0162": PIDDefinition("01", "62", "Actual_Engine_Torque", "Torque real motor", 1, "%", lambda d: d[0]-125),
    "0163": PIDDefinition("01", "63", "Engine_Reference_Torque", "Torque referencia", 2, "Nm", lambda d: (d[0]*256)+d[1]),
}

SUZUKI_SPECIFIC_PIDS = {
    "0105_E": PIDDefinition("01", "05", "Coolant_Temp_ECM", "Temp. refrigerante ECM precisa", 2, "C", lambda d: ((d[0]*256)+d[1])/10.0-40.0, category="Suzuki", module="ECM"),
    "0121_E": PIDDefinition("01", "21", "Odometer", "Odometro vehiculo", 3, "km", lambda d: (d[0]*65536+d[1]*256+d[2])/10.0, category="Suzuki", module="ECM"),
    "01A0": PIDDefinition("01", "A0", "TCM_ATF_Temp", "Temp. ATF transmision", 2, "C", lambda d: ((d[0]*256)+d[1])/10.0-40.0, category="Suzuki", module="TCM"),
    "01A1": PIDDefinition("01", "A1", "TCM_Gear_Position", "Posicion marcha", 1, "gear", lambda d: f"Gear:{d[0]}", category="Suzuki", module="TCM"),
    "01A2": PIDDefinition("01", "A2", "TCM_TCC_Status", "Estado TCC", 1, "status", lambda d: f"TCC:{d[0]}", category="Suzuki", module="TCM"),
    "01A3": PIDDefinition("01", "A3", "TCM_Input_Shaft_Speed", "RPM eje entrada", 2, "rpm", decode_rpm_ab, category="Suzuki", module="TCM"),
    "01A4": PIDDefinition("01", "A4", "TCM_Output_Shaft_Speed", "RPM eje salida", 2, "rpm", decode_rpm_ab, category="Suzuki", module="TCM"),
    "01A5": PIDDefinition("01", "A5", "TCM_Line_Pressure", "Presion linea hidraulica", 2, "kPa", lambda d: (d[0]*256)+d[1], category="Suzuki", module="TCM"),
    "01A6": PIDDefinition("01", "A6", "TCM_Shift_Solenoid_A", "Solenoide A", 1, "%", decode_percent_a, category="Suzuki", module="TCM"),
    "01A7": PIDDefinition("01", "A7", "TCM_Shift_Solenoid_B", "Solenoide B", 1, "%", decode_percent_a, category="Suzuki", module="TCM"),
    "01A8": PIDDefinition("01", "A8", "TCM_Lockup_Solenoid", "Solenoide lockup", 1, "%", decode_percent_a, category="Suzuki", module="TCM"),
    "01B0": PIDDefinition("01", "B0", "ABS_Wheel_Speed_FL", "Velocidad rueda FL", 2, "km/h", lambda d: ((d[0]*256)+d[1])/100.0, category="Suzuki", module="ABS"),
    "01B1": PIDDefinition("01", "B1", "ABS_Wheel_Speed_FR", "Velocidad rueda FR", 2, "km/h", lambda d: ((d[0]*256)+d[1])/100.0, category="Suzuki", module="ABS"),
    "01B2": PIDDefinition("01", "B2", "ABS_Wheel_Speed_RL", "Velocidad rueda RL", 2, "km/h", lambda d: ((d[0]*256)+d[1])/100.0, category="Suzuki", module="ABS"),
    "01B3": PIDDefinition("01", "B3", "ABS_Wheel_Speed_RR", "Velocidad rueda RR", 2, "km/h", lambda d: ((d[0]*256)+d[1])/100.0, category="Suzuki", module="ABS"),
    "01B4": PIDDefinition("01", "B4", "ABS_Brake_Pressure", "Presion frenos", 2, "bar", lambda d: ((d[0]*256)+d[1])/100.0, category="Suzuki", module="ABS"),
    "01B5": PIDDefinition("01", "B5", "ABS_Yaw_Rate", "Tasa guinada", 2, "deg/s", lambda d: ((d[0]*256)+d[1])/100.0-327.68, category="Suzuki", module="ABS"),
    "01B6": PIDDefinition("01", "B6", "ABS_Lateral_Accel", "Aceleracion lateral", 2, "m/s2", lambda d: ((d[0]*256)+d[1])/100.0-327.68, category="Suzuki", module="ABS"),
    "01B7": PIDDefinition("01", "B7", "ABS_Steering_Angle", "Angulo direccion", 2, "deg", lambda d: ((d[0]*256)+d[1])/10.0-3276.8, category="Suzuki", module="ABS"),
    "01C0": PIDDefinition("01", "C0", "EPS_Motor_Current", "Corriente motor EPS", 2, "A", lambda d: ((d[0]*256)+d[1])/100.0-327.68, category="Suzuki", module="EPS"),
    "01C1": PIDDefinition("01", "C1", "EPS_Torque_Sensor", "Sensor torque direccion", 2, "Nm", lambda d: ((d[0]*256)+d[1])/100.0-327.68, category="Suzuki", module="EPS"),
    "01C2": PIDDefinition("01", "C2", "EPS_Assist_Level", "Nivel asistencia EPS", 1, "%", decode_percent_a, category="Suzuki", module="EPS"),
    "01C3": PIDDefinition("01", "C3", "EPS_Motor_Temp", "Temp. motor EPS", 1, "C", decode_temp_a, category="Suzuki", module="EPS"),
    "01D0": PIDDefinition("01", "D0", "BCM_Battery_Voltage", "Voltaje bateria BCM", 2, "V", decode_voltage_ab, category="Suzuki", module="BCM"),
    "01D1": PIDDefinition("01", "D1", "BCM_Alternator_Load", "Carga alternador", 1, "%", decode_percent_a, category="Suzuki", module="BCM"),
    "01D2": PIDDefinition("01", "D2", "BCM_Door_Status", "Estado puertas", 1, "bitmask", lambda d: f"Doors:{d[0]:08b}", category="Suzuki", module="BCM"),
    "01D3": PIDDefinition("01", "D3", "BCM_Light_Status", "Estado luces", 1, "bitmask", lambda d: f"Lights:{d[0]:08b}", category="Suzuki", module="BCM"),
    "01D4": PIDDefinition("01", "D4", "BCM_Key_Position", "Posicion llave", 1, "status", lambda d: f"KeyPos:{d[0]}", category="Suzuki", module="BCM"),
    "01E0": PIDDefinition("01", "E0", "SRS_Crash_Sensor", "Sensor impacto", 1, "status", lambda d: f"Crash:{d[0]}", category="Suzuki", module="SRS"),
    "01E1": PIDDefinition("01", "E1", "SRS_Seatbelt_Status", "Estado cinturones", 1, "bitmask", lambda d: f"Seatbelts:{d[0]:08b}", category="Suzuki", module="SRS"),
}

# ==============================================================================
# SUZUKI MODE 21 - PIDs PROPRIETARIOS (CAN 7E0/7E8)
# ==============================================================================
# El Baleno 2019+ (ECU 33920-65GP, Bosch MEDC17) responde a Mode 21 sobre CAN.
# Estos PIDs fueron descubiertos escaneando el ECU real.
#
# 2100 = Bloque principal de datos en vivo (141 bytes, ISO-TP multi-frame)
#   Los bytes mapean a parametros internos del ECU.
#   Offset 0-5:   Fault codes 1-6 (FF = sin DTCs)
#   Offset 6-7:   RPM (high/low) - formula: (H*256+L)/5.1
#   Offset 8:     Target idle
#   Offset 9:     Vehicle speed (km/h directo)
#   Offset 10:    Coolant temp - formula: (raw/255)*159-40
#   Offset 11:    Intake air temp - formula: (raw/255)*159-40
#   Offset 12:    TPS angle - formula: (raw/255)*100
#   Offset 13:    TPS voltage - formula: (raw/255)*5
#   Offset 14:    Injector pulse width high
#   Offset 15:    Injector pulse width low
#   Offset 16:    Ignition advance - formula: (raw/255)*90-12
#   Offset 17:    MAP sensor - formula: (raw/255)*166.63-20
#   Offset 18:    Barometric pressure
#   Offset 19:    ISC duty
#   Offset 20:    Battery voltage - formula: raw*0.0787
#   Offset 22:    Status flags 1
#
# NOTA: Estos offsets son INCERTIDUMBRES basadas en suzuki_sdl.
# Se necesita confirmar con el motor encendido. Se provee vista raw.

def _decode_mode21_block(data):
    """Decodifica el bloque 2100 en parametros individuales."""
    if len(data) < 21:
        return {"raw": " ".join(f"{b:02X}" for b in data)}
    result = {}
    if len(data) > 6:
        rpm = int(((data[6] * 256) + data[7]) / 5.1)
        result["RPM"] = rpm
    if len(data) > 9:
        result["Velocidad"] = data[9]
    if len(data) > 10:
        ect = round((data[10] / 255.0) * 159 - 40)
        result["Temp.Ref"] = ect
    if len(data) > 11:
        iat = round((data[11] / 255.0) * 159 - 40)
        result["Temp.Aire"] = iat
    if len(data) > 12:
        tps = round((data[12] / 255.0) * 100)
        result["Acelerador"] = tps
    if len(data) > 16:
        adv = round(((data[16] / 255.0) * 90) - 12)
        result["Avance"] = adv
    if len(data) > 17:
        map_kpa = round((data[17] / 255.0) * 166.63 - 20, 1)
        result["MAP"] = map_kpa
    if len(data) > 20:
        bat = round(data[20] * 0.0787, 2)
        result["Bateria"] = bat
    if len(data) > 22:
        result["Flags"] = f"0x{data[22]:02X}"
    return result

def _extract_rpm_2100(data):
    if len(data) > 7:
        return int(((data[6] * 256) + data[7]) / 5.1)
    return 0

def _extract_speed_2100(data):
    if len(data) > 9:
        return data[9]
    return 0

def _extract_coolant_temp_2100(data):
    if len(data) > 10:
        return round((data[10] / 255.0) * 159 - 40)
    return 0

def _extract_intake_temp_2100(data):
    if len(data) > 11:
        return round((data[11] / 255.0) * 159 - 40)
    return 0

def _extract_tps_2100(data):
    if len(data) > 12:
        return round((data[12] / 255.0) * 100)
    return 0

def _extract_timing_2100(data):
    if len(data) > 16:
        return round(((data[16] / 255.0) * 90) - 12)
    return 0

def _extract_map_2100(data):
    if len(data) > 17:
        return round((data[17] / 255.0) * 166.63 - 20, 1)
    return 0.0

def _extract_battery_2100(data):
    if len(data) > 20:
        return round(data[20] * 0.0787, 2)
    return 0.0

SUZUKI_MODE21_PIDS = {
    "S21_00": PIDDefinition(
        "21", "00", "Mode21_Live_Data",
        "Datos en vivo Suzuki (bloque completo)",
        141, "block", _decode_mode21_block,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_RPM": PIDDefinition(
        "21", "00", "S21_RPM",
        "RPM del motor (2100 offset 6-7)",
        2, "rpm", _extract_rpm_2100,
        min_val=0, max_val=8000,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_SPD": PIDDefinition(
        "21", "00", "S21_Velocidad",
        "Velocidad (2100 offset 9)",
        2, "km/h", _extract_speed_2100,
        min_val=0, max_val=220,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_ECT": PIDDefinition(
        "21", "00", "S21_Coolant_Temp",
        "Temp. refrigerante (2100 offset 10)",
        2, "\u00b0C", _extract_coolant_temp_2100,
        min_val=-40, max_val=120,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_IAT": PIDDefinition(
        "21", "00", "S21_Intake_Temp",
        "Temp. aire admision (2100 offset 11)",
        2, "\u00b0C", _extract_intake_temp_2100,
        min_val=-40, max_val=120,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_TPS": PIDDefinition(
        "21", "00", "S21_Throttle",
        "Pos. acelerador (2100 offset 12)",
        2, "%", _extract_tps_2100,
        min_val=0, max_val=100,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_ADV": PIDDefinition(
        "21", "00", "S21_Timing",
        "Avance encendido (2100 offset 16)",
        2, "\u00b0", _extract_timing_2100,
        min_val=-12, max_val=78,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_MAP": PIDDefinition(
        "21", "00", "S21_MAP",
        "Presion colector (2100 offset 17)",
        2, "kPa", _extract_map_2100,
        min_val=0, max_val=200,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_BAT": PIDDefinition(
        "21", "00", "S21_Battery",
        "Voltaje bateria (2100 offset 20)",
        2, "V", _extract_battery_2100,
        min_val=0, max_val=18,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_27": PIDDefinition(
        "21", "27", "Mode21_Data_Block_27",
        "Bloque de datos adicional 0x27",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_28": PIDDefinition(
        "21", "28", "Mode21_Data_Block_28",
        "Bloque de datos adicional 0x28",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_30": PIDDefinition(
        "21", "30", "Mode21_Data_Block_30",
        "Bloque calibracion 0x30",
        41, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_31": PIDDefinition(
        "21", "31", "Mode21_Data_Block_31",
        "Bloque calibracion 0x31",
        41, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_32": PIDDefinition(
        "21", "32", "Mode21_Status_32",
        "Estado 0x32",
        5, "raw", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_33": PIDDefinition(
        "21", "33", "Mode21_Status_33",
        "Estado 0x33",
        5, "raw", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_34": PIDDefinition(
        "21", "34", "Mode21_Data_Block_34",
        "Bloque de datos 0x34",
        19, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_35": PIDDefinition(
        "21", "35", "Mode21_Data_Block_35",
        "Bloque de datos 0x35",
        26, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_36": PIDDefinition(
        "21", "36", "Mode21_Status_36",
        "Estado 0x36",
        5, "raw", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_3C": PIDDefinition(
        "21", "3C", "Mode21_Data_Block_3C",
        "Bloque de datos 0x3C",
        26, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_3D": PIDDefinition(
        "21", "3D", "Mode21_Status_3D",
        "Estado 0x3D",
        5, "raw", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_40": PIDDefinition(
        "21", "40", "Mode21_Data_Block_40",
        "Bloque de datos 0x40",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_41": PIDDefinition(
        "21", "41", "Mode21_Data_Block_41",
        "Bloque de datos 0x41",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_42": PIDDefinition(
        "21", "42", "Mode21_Data_Block_42",
        "Bloque de datos 0x42",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_43": PIDDefinition(
        "21", "43", "Mode21_Data_Block_43",
        "Bloque de datos 0x43",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_44": PIDDefinition(
        "21", "44", "Mode21_Data_Block_44",
        "Bloque de datos 0x44",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_45": PIDDefinition(
        "21", "45", "Mode21_Data_Block_45",
        "Bloque de datos 0x45",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_46": PIDDefinition(
        "21", "46", "Mode21_Data_Block_46",
        "Bloque de datos 0x46",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_47": PIDDefinition(
        "21", "47", "Mode21_Data_Block_47",
        "Bloque de datos 0x47",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_48": PIDDefinition(
        "21", "48", "Mode21_Data_Block_48",
        "Bloque de datos 0x48",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_49": PIDDefinition(
        "21", "49", "Mode21_Data_Block_49",
        "Bloque de datos 0x49",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_A2": PIDDefinition(
        "21", "A2", "Mode21_Data_Block_A2",
        "Bloque de datos 0xA2",
        69, "block", decode_raw_hex,
        category="Suzuki Mode 21", module="ECM"
    ),
    "S21_D0": PIDDefinition(
        "21", "D0", "Mode21_ECU_ID",
        "Identificacion ECU",
        77, "string",
        lambda d: d[1:76].decode("ascii", errors="replace").rstrip("\x00"),
        category="Suzuki Mode 21", module="ECM"
    ),
}

ALL_PIDS = {**GENERIC_PIDS, **SUZUKI_SPECIFIC_PIDS, **SUZUKI_MODE21_PIDS}

SUZUKI_DTC_DATABASE = {
    "P1105": "Barometric Pressure Circuit Malfunction",
    "P1110": "Intake Air Temperature Circuit High Input",
    "P1115": "Engine Coolant Temperature Circuit High Input",
    "P1120": "Accelerator Pedal Position Sensor 1 Circuit Malfunction",
    "P1121": "Accelerator Pedal Position Sensor 1 Circuit Range/Performance",
    "P1122": "Accelerator Pedal Position Sensor 1 Circuit Low Input",
    "P1125": "Accelerator Pedal Position Sensor 2 Circuit Malfunction",
    "P1130": "HO2S Circuit Malfunction (Bank 1 Sensor 1)",
    "P1135": "HO2S Heater Control Circuit (Bank 1 Sensor 1)",
    "P1140": "MAF Sensor Circuit Range/Performance",
    "P1145": "MAF Sensor Circuit Low Input",
    "P1150": "MAF Sensor Circuit High Input",
    "P1170": "Fuel Trim Malfunction (Bank 1)",
    "P1190": "Fuel Pressure Regulator Control Circuit Malfunction",
    "P1200": "Fuel Injector Circuit Malfunction (Cylinder 1)",
    "P1205": "Fuel Injector Circuit Malfunction (Cylinder 2)",
    "P1210": "Fuel Injector Circuit Malfunction (Cylinder 3)",
    "P1215": "Fuel Injector Circuit Malfunction (Cylinder 4)",
    "P1230": "Fuel Pump Control Circuit Malfunction",
    "P1240": "Intake Air Temperature Circuit Malfunction",
    "P1250": "Fuel Pressure Regulator Control Solenoid Valve Circuit Malfunction",
    "P1260": "Fuel Pump Speed Control Circuit Malfunction",
    "P1270": "Accelerator Pedal Position Sensor 1-2 Correlation",
    "P1280": "Accelerator Pedal Position Sensor 1 Circuit Malfunction",
    "P1290": "Cylinder Head Temperature Sensor Circuit Malfunction",
    "P1300": "Ignition Coil 1 Primary Control Circuit Malfunction",
    "P1305": "Ignition Coil 2 Primary Control Circuit Malfunction",
    "P1310": "Ignition Coil 3 Primary Control Circuit Malfunction",
    "P1315": "Ignition Coil 4 Primary Control Circuit Malfunction",
    "P1320": "Knock Sensor Circuit Malfunction",
    "P1325": "Knock Sensor Circuit Range/Performance",
    "P1330": "Camshaft Position Sensor Circuit Malfunction",
    "P1335": "Crankshaft Position Sensor Circuit Malfunction",
    "P1340": "Camshaft Position Sensor - Crankshaft Position Sensor Correlation",
    "P1350": "Ignition Coil Control Circuit Malfunction",
    "P1400": "EGR Valve Position Sensor Circuit Malfunction",
    "P1405": "EGR Valve Control Circuit Malfunction",
    "P1410": "Secondary Air Injection System Malfunction",
    "P1420": "Fuel Tank Pressure Sensor Circuit Malfunction",
    "P1440": "EVAP System Vent Control Circuit Malfunction",
    "P1450": "EVAP System Pressure Sensor Circuit Malfunction",
    "P1460": "Cooling Fan Control Circuit Malfunction",
    "P1470": "Cooling Fan Relay Control Circuit Malfunction",
    "P1480": "Cooling Fan Speed Sensor Circuit Malfunction",
    "P1490": "EGR Valve Position Sensor Circuit Range/Performance",
    "P1500": "Starter Signal Circuit Malfunction",
    "P1510": "Idle Air Control (IAC) Valve Circuit Malfunction",
    "P1520": "IAC Valve Position Sensor Circuit Malfunction",
    "P1530": "A/C Compressor Relay Control Circuit Malfunction",
    "P1540": "A/C Refrigerant Pressure Sensor Circuit Malfunction",
    "P1550": "Battery Current Sensor Circuit Malfunction",
    "P1560": "Battery Temperature Sensor Circuit Malfunction",
    "P1570": "Immobilizer Communication Line Malfunction",
    "P1575": "Immobilizer - ECM Communication Error",
    "P1580": "Cruise Control System Malfunction",
    "P1590": "Neutral Position Switch Circuit Malfunction",
    "P1600": "ECM Internal Malfunction",
    "P1610": "ECM Programming Error",
    "P1620": "ECM EEPROM Error",
    "P1630": "ECM Communication Error",
    "P1640": "Throttle Valve Control Module Malfunction",
    "P1650": "Power Steering Pressure Switch Circuit Malfunction",
    "P1660": "Cooling Fan Control Circuit Malfunction (High Speed)",
    "P1670": "Cooling Fan Control Circuit Malfunction (Low Speed)",
    "P1680": "Metering Oil Pump Circuit Malfunction",
    "P1690": "Throttle Body Control Circuit Malfunction",
    "P1700": "Transmission Control System Malfunction",
    "P1705": "Transmission Range Sensor Circuit Malfunction",
    "P1710": "Transmission Fluid Temperature Sensor Circuit Malfunction",
    "P1720": "Vehicle Speed Sensor Circuit Malfunction (TCM)",
    "P1730": "Turbine Speed Sensor Circuit Malfunction",
    "P1740": "Torque Converter Clutch Solenoid Circuit Malfunction",
    "P1750": "Shift Solenoid A Circuit Malfunction",
    "P1755": "Shift Solenoid B Circuit Malfunction",
    "P1760": "Shift Solenoid C Circuit Malfunction",
    "P1765": "Pressure Control Solenoid Circuit Malfunction",
    "P1770": "Lock-up Solenoid Circuit Malfunction",
    "P1775": "TCM Communication Error",
    "P1780": "TCM Internal Malfunction",
    "P1790": "TCM Programming Error",
    "C1100": "ABS Wheel Speed Sensor Front Left Circuit Malfunction",
    "C1105": "ABS Wheel Speed Sensor Front Right Circuit Malfunction",
    "C1110": "ABS Wheel Speed Sensor Rear Left Circuit Malfunction",
    "C1115": "ABS Wheel Speed Sensor Rear Right Circuit Malfunction",
    "C1120": "ABS Wheel Speed Sensor Front Left Circuit Range/Performance",
    "C1125": "ABS Wheel Speed Sensor Front Right Circuit Range/Performance",
    "C1130": "ABS Wheel Speed Sensor Rear Left Circuit Range/Performance",
    "C1135": "ABS Wheel Speed Sensor Rear Right Circuit Range/Performance",
    "C1140": "ABS Hydraulic Pump Motor Circuit Malfunction",
    "C1145": "ABS Solenoid Valve Circuit Malfunction",
    "C1150": "ABS Control Module Internal Malfunction",
    "C1155": "ABS Control Module Communication Error",
    "C1160": "Brake Fluid Level Sensor Circuit Malfunction",
    "C1165": "Brake Light Switch Circuit Malfunction",
    "C1170": "Parking Brake Switch Circuit Malfunction",
    "C1175": "Yaw Rate Sensor Circuit Malfunction",
    "C1180": "Lateral Acceleration Sensor Circuit Malfunction",
    "C1185": "Steering Angle Sensor Circuit Malfunction",
    "C1190": "ABS Control Module Programming Error",
    "C1200": "EPS Motor Circuit Malfunction",
    "C1205": "EPS Torque Sensor Circuit Malfunction",
    "C1210": "EPS Control Module Internal Malfunction",
    "C1215": "EPS Control Module Communication Error",
    "C1220": "EPS Control Module Programming Error",
    "B1100": "BCM Internal Malfunction",
    "B1105": "BCM Communication Error",
    "B1110": "Door Lock Actuator Circuit Malfunction",
    "B1115": "Door Unlock Actuator Circuit Malfunction",
    "B1120": "Trunk Lid Actuator Circuit Malfunction",
    "B1125": "Fuel Lid Actuator Circuit Malfunction",
    "B1130": "Horn Circuit Malfunction",
    "B1135": "Wiper Motor Circuit Malfunction",
    "B1140": "Washer Motor Circuit Malfunction",
    "B1145": "Rear Defogger Circuit Malfunction",
    "B1150": "Power Window Motor Circuit Malfunction",
    "B1155": "Sunroof Motor Circuit Malfunction",
    "B1160": "Seat Heater Circuit Malfunction",
    "B1165": "Mirror Heater Circuit Malfunction",
    "B1170": "Keyless Entry System Malfunction",
    "B1175": "Immobilizer Antenna Circuit Malfunction",
    "B1180": "Theft Deterrent System Malfunction",
    "B1185": "BCM Programming Error",
    "B1190": "Instrument Cluster Communication Error",
    "B1195": "HVAC Control Module Communication Error",
    "B1200": "SRS Airbag Sensor Front Circuit Malfunction",
    "B1205": "SRS Airbag Sensor Side Circuit Malfunction",
    "B1210": "SRS Airbag Inflator Circuit Malfunction",
    "B1215": "SRS Control Module Internal Malfunction",
    "B1220": "SRS Control Module Communication Error",
    "B1225": "SRS Control Module Programming Error",
    "B1230": "Seat Belt Pretensioner Circuit Malfunction",
    "B1235": "Occupant Classification System Malfunction",
    "U1100": "CAN Bus Communication Error (ECM)",
    "U1105": "CAN Bus Communication Error (TCM)",
    "U1110": "CAN Bus Communication Error (ABS)",
    "U1115": "CAN Bus Communication Error (EPS)",
    "U1120": "CAN Bus Communication Error (BCM)",
    "U1125": "CAN Bus Communication Error (SRS)",
    "U1130": "CAN Bus Communication Error (Cluster)",
    "U1135": "CAN Bus Communication Error (HVAC)",
    "U1140": "CAN Bus Communication Error (TPMS)",
    "U1145": "CAN Bus Communication Error (Immo)",
    "U1150": "CAN Bus Off Error",
    "U1155": "CAN Bus Error Passive",
    "U1160": "LIN Bus Communication Error",
    "U1165": "K-Line Communication Error",
    "U1170": "Communication Bus Short to Ground",
    "U1175": "Communication Bus Short to Battery",
    "U1180": "Communication Bus Open Circuit",
    "U1185": "Invalid Data Received from ECM",
    "U1190": "Invalid Data Received from TCM",
    "U1195": "Invalid Data Received from ABS",
    "U1200": "Lost Communication with ECM",
    "U1205": "Lost Communication with TCM",
    "U1210": "Lost Communication with ABS",
    "U1215": "Lost Communication with EPS",
    "U1220": "Lost Communication with BCM",
    "U1225": "Lost Communication with SRS",
    "U1230": "Lost Communication with Instrument Cluster",
    "U1235": "Lost Communication with HVAC",
    "U1240": "Lost Communication with TPMS",
    "U1245": "Lost Communication with Immobilizer",
    "U1250": "Software Incompatibility with ECM",
    "U1255": "Software Incompatibility with TCM",
    "U1260": "Software Incompatibility with ABS",
    "U1265": "Software Incompatibility with BCM",
    "U1270": "Event Information - Communication Bus",
    "U1280": "Parity Error in Communication",
    "U1285": "Checksum Error in Communication",
    "U1290": "Frame Error in Communication",
    "U1295": "Overrun Error in Communication",
    "U1300": "Bus Off Recovery",
    "U1310": "Network Initialization Error",
    "U1315": "Gateway Module Malfunction",
    "U1320": "Gateway Module Communication Error",
    "U1325": "Gateway Module Programming Error",
    "U1330": "Network Configuration Error",
    "U1335": "Network Security Error",
    "U1340": "Network Authentication Error",
    "U1345": "Network Encryption Error",
    "U1350": "Network Timeout Error",
    "U1355": "Network Congestion Error",
    "U1360": "Network Routing Error",
    "U1365": "Network Addressing Error",
    "U1370": "Network Protocol Error",
    "U1375": "Network Data Length Error",
    "U1380": "Network Sequence Error",
    "U1385": "Network Timing Error",
    "U1390": "Network Synchronization Error",
    "U1395": "Network Collision Error",
}

# ==============================================================================
# GESTOR DE PIDs
# ==============================================================================

class PIDManager:
    """Gestiona la lectura y decodificacion de PIDs."""

    def __init__(self, communicator):
        self.communicator = communicator
        self.last_values = {}
        self.supported_pids = set()
        self._mode21_cache = {}  # {pid_hex: (isotp_data, timestamp)}

    def query_pid(self, pid_key):
        if pid_key not in ALL_PIDS:
            return None, f"PID {pid_key} no definido"

        pid_def = ALL_PIDS[pid_key]

        # --- Suzuki Mode 21 (CAN 7E0/7E8) ---
        if pid_def.category == "Suzuki Mode 21":
            return self._query_mode21_pid(pid_key, pid_def)

        # --- OBD2 Estandar / Suzuki Mode 01 ---
        module = None
        if pid_def.category == "Suzuki":
            for mod in SuzukiModule:
                if mod.name == pid_def.module:
                    module = mod
                    break

        response = self.communicator.send_obd2_command(pid_def.mode, pid_def.pid, module)

        if "NO DATA" in response or "ERROR" in response or "UNABLE" in response:
            return None, f"No data: {response.strip()}"

        # Parsear respuesta
        lines = response.strip().split("\r")
        data_line = ""
        for line in lines:
            line = line.strip()
            if not line or line == ">":
                continue
            if line.startswith("7E") or line.startswith("41") or line.startswith("7F"):
                data_line = line
                break
            if len(line) >= 4 and line[0:2] in ("41", "42", "43", "44", "45", "46", "47", "48", "49", "4A", "4B", "4C", "4D", "4E", "4F", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59", "5A", "5B", "5C", "5D", "5E", "5F", "60", "61", "62", "63"):
                data_line = line
                break

        if not data_line:
            return None, "No se encontro linea de datos valida"

        # Extraer bytes de datos
        parts = data_line.split()
        try:
            # Saltar header CAN si existe
            if parts[0].startswith("7E"):
                parts = parts[2:]
            # Saltar modo respuesta y PID
            if len(parts) >= 2:
                data_bytes = [int(p, 16) for p in parts[2:]]
            else:
                return None, "Respuesta demasiado corta"
        except ValueError:
            return None, f"Error parseando hex: {data_line}"

        if len(data_bytes) < pid_def.bytes_count:
            return None, f"Datos insuficientes: {len(data_bytes)} < {pid_def.bytes_count}"

        try:
            value = pid_def.decode_func(data_bytes[:pid_def.bytes_count])
            self.last_values[pid_key] = {
                "value": value,
                "unit": pid_def.unit,
                "timestamp": time.time(),
                "name": pid_def.name,
                "description": pid_def.description,
            }
            return value, None
        except Exception as e:
            return None, f"Error decodificando: {e}"

    def _query_mode21_pid(self, pid_key, pid_def):
        """Consulta un PID Suzuki Mode 21 sobre CAN (ISO-TP multi-frame)."""
        try:
            cmd = f"{pid_def.mode}{pid_def.pid.zfill(2)}"
            now = time.time()

            # Asegurar que el header CAN apunta al ECM
            if not hasattr(self, '_last_module') or self._last_module != "ECM":
                self.communicator.send_command("ATSH7E0")
                self._last_module = "ECM"

            # Verificar cache (300ms TTL para datos en vivo mas fluidos)
            cache_key = pid_def.pid.upper()
            if cache_key in self._mode21_cache:
                cached_data, cached_ts = self._mode21_cache[cache_key]
                if now - cached_ts < 0.3:
                    isotp_data = cached_data
                    value = self._decode_mode21_data(isotp_data, pid_key, pid_def)
                    if value is not None:
                        self.last_values[pid_key] = {
                            "value": value,
                            "unit": pid_def.unit,
                            "timestamp": now,
                            "name": pid_def.name,
                            "description": pid_def.description,
                        }
                        return value, None

            # Enviar comando al ECU
            response = self.communicator.send_command(cmd, timeout=3.0)

            if "NO DATA" in response or "ERROR" in response or "UNABLE" in response:
                return None, f"No data: {response.strip()[:80]}"

            # Parsear ISO-TP
            isotp_data = parse_isotp_response(response)

            # Fallback: si parse_isotp no funciono, intentar parseo manual
            if not isotp_data or len(isotp_data) < 2:
                isotp_data = self._fallback_parse_mode21(response)

            if not isotp_data or len(isotp_data) < 2:
                return None, "Respuesta Mode 21 vacia o incompleta"

            # Guardar en cache
            self._mode21_cache[cache_key] = (isotp_data, now)

            # Decodificar
            value = self._decode_mode21_data(isotp_data, pid_key, pid_def)
            if value is not None:
                self.last_values[pid_key] = {
                    "value": value,
                    "unit": pid_def.unit,
                    "timestamp": now,
                    "name": pid_def.name,
                    "description": pid_def.description,
                }
                return value, None

            return None, "No se pudo decodificar Mode 21"

        except Exception as e:
            return None, f"Error Mode 21: {e}"

    def _fallback_parse_mode21(self, response):
        """Parseo manual cuando parse_isotp_response falla."""
        for line in response.replace("\r", "\n").split("\n"):
            line = line.strip().replace(">", "").strip()
            if not line:
                continue
            parts = line.split()
            if not parts:
                continue

            # Formato con header CAN: 7E8 10 XX 61 00 ...
            if parts[0].upper() == "7E8" and len(parts) > 2:
                try:
                    # Quitar ID CAN, tomar todo lo demas
                    hex_str = "".join(parts[1:])
                    raw = bytes.fromhex(hex_str)
                    # Buscar 61 (Mode 21 response) en los bytes
                    for i in range(len(raw)):
                        if raw[i] == 0x61 and i + 1 < len(raw):
                            return raw[i:]  # 61 + PID + data
                except ValueError:
                    pass

            # Formato sin headers: 61 00 ...
            if parts[0] == "61" or (len(parts[0]) == 2 and parts[0][0] == "6"):
                try:
                    hex_str = "".join(parts)
                    return bytes.fromhex(hex_str)
                except ValueError:
                    pass

        return b""

    def _decode_mode21_data(self, isotp_data, pid_key, pid_def):
        """Decodifica datos ISO-TP segun el tipo de PID."""
        # Para el bloque principal 0x00 (con decoder custom)
        if pid_def.unit == "block" and pid_def.decode_func != decode_raw_hex:
            # Saltar SID (61) + PID (00) = offset 2
            data_slice = isotp_data[2:] if len(isotp_data) > 2 else isotp_data
            return pid_def.decode_func(data_slice)

        # Para PIDs virtuales de 2100 (RPM, SPD, ECT, etc.)
        if pid_def.unit not in ("block", "string", "raw") and pid_def.decode_func != decode_raw_hex:
            data_slice = isotp_data[2:] if len(isotp_data) > 2 else isotp_data
            return pid_def.decode_func(list(data_slice))

        # Para string (ECU ID, etc.)
        if pid_def.unit == "string":
            return pid_def.decode_func(list(isotp_data))

        # Para raw/block (otros PIDs Mode 21)
        data_bytes = list(isotp_data[2:]) if len(isotp_data) > 2 else list(isotp_data)
        if len(data_bytes) < pid_def.bytes_count and pid_def.unit != "string":
            return None
        return pid_def.decode_func(data_bytes[:pid_def.bytes_count] if pid_def.unit != "string" else data_bytes)

    def query_supported_pids(self):
        self.supported_pids = set()
        for pid_key in ["0100", "0120", "0140", "0160"]:
            value, error = self.query_pid(pid_key)
            if value and not error:
                self.supported_pids.add(pid_key)
        return self.supported_pids

    def get_pid_list_by_module(self, module_name="All"):
        result = {}
        for key, pid in ALL_PIDS.items():
            if module_name == "All" or pid.module == module_name or pid.category == "Generic":
                result[key] = pid
        return result

# ==============================================================================
# ESCANER DE DTCs
# ==============================================================================

class DTCScanner:
    """Escanea y gestiona Diagnostic Trouble Codes."""

    def __init__(self, communicator):
        self.communicator = communicator

    def _parse_dtc_bytes(self, byte1, byte2):
        """Convierte 2 bytes a codigo DTC."""
        category_map = {0x00: "P", 0x40: "C", 0x80: "B", 0xC0: "U"}
        category = category_map.get(byte1 & 0xC0, "P")
        digit2 = (byte1 >> 4) & 0x03
        digit3 = byte1 & 0x0F
        digit4 = (byte2 >> 4) & 0x0F
        digit5 = byte2 & 0x0F
        return f"{category}{digit2}{digit3:X}{digit4:X}{digit5:X}"

    def _get_dtc_description(self, code):
        if code in SUZUKI_DTC_DATABASE:
            return SUZUKI_DTC_DATABASE[code]
        prefix = code[:2]
        cat = DTC_CATEGORIES.get(code[0], "Unknown")
        ptype = DTC_PREFIXES.get(prefix, "Unknown")
        return f"[{ptype}] {cat} - Codigo no documentado en base de datos Suzuki"

    def read_dtcs(self, mode="03"):
        """Lee DTCs almacenados (03), pendientes (07) o permanentes (0A)."""
        response = self.communicator.send_command(mode)
        dtcs = []

        if "NO DATA" in response or "ERROR" in response:
            return dtcs

        lines = response.strip().split("\r")
        for line in lines:
            line = line.strip().replace(" ", "")
            if not line or line == ">" or len(line) < 6:
                continue

            # Buscar lineas que empiecen con modo respuesta
            if line.startswith("43") or line.startswith("47") or line.startswith("4A"):
                # Saltar header y modo
                if line.startswith("7E"):
                    idx = line.find("43")
                    if idx < 0:
                        idx = line.find("47")
                    if idx < 0:
                        idx = line.find("4A")
                    if idx >= 0:
                        line = line[idx:]

                data = line[2:]  # Saltar modo respuesta
                # Cada DTC = 4 hex chars = 2 bytes
                for i in range(0, len(data), 4):
                    chunk = data[i:i+4]
                    if len(chunk) == 4:
                        try:
                            b1 = int(chunk[0:2], 16)
                            b2 = int(chunk[2:4], 16)
                            if b1 == 0 and b2 == 0:
                                continue
                            code = self._parse_dtc_bytes(b1, b2)
                            desc = self._get_dtc_description(code)
                            cat = DTC_CATEGORIES.get(code[0], "Unknown")
                            prefix = DTC_PREFIXES.get(code[:2], "Unknown")
                            is_pending = (mode == "07")
                            is_permanent = (mode == "0A")
                            dtcs.append(DTCRecord(
                                code=code, description=desc, category=cat,
                                prefix_type=prefix, is_pending=is_pending,
                                is_permanent=is_permanent
                            ))
                        except ValueError:
                            continue
        return dtcs

    def read_all_dtcs(self):
        """Lee todos los tipos de DTCs."""
        all_dtcs = []
        all_dtcs.extend(self.read_dtcs("03"))
        all_dtcs.extend(self.read_dtcs("07"))
        all_dtcs.extend(self.read_dtcs("0A"))
        return all_dtcs

    def clear_dtcs(self):
        """Borra todos los DTCs."""
        response = self.communicator.send_command("04")
        return "OK" in response or "44" in response

    def read_suzuki_dtcs_from_2100(self):
        """
        Lee DTCs del bloque Mode 21 0x00 (Suzuki-proprietary).

        En el Baleno, los primeros 6 bytes del bloque 2100 son DTCs:
          Bytes 0-3 (addr 0x00-0x03): DTCs 1-4
          Bytes 24-25 (addr 0x20-0x21): DTCs 5-6

        Fuerza una lectura fresca ignorando cache.
        """
        dtcs = []
        try:
            # Forzar lectura fresca (sin cache)
            pid_def = ALL_PIDS.get("S21_00")
            if not pid_def:
                return dtcs

            # Enviar 2100 directamente
            response = self.communicator.send_command("2100", timeout=3.0)
            if "NO DATA" in response or "ERROR" in response:
                return dtcs

            isotp_data = parse_isotp_response(response)
            if not isotp_data or len(isotp_data) < 6:
                return dtcs

            # DTCs en bytes 2-7 del payload (post SID+PID)
            # SDL addresses 0x00-0x03 = DTCs 1-4
            # SDL addresses 0x20-0x21 = DTCs 5-6 (may need offset calc)

            # Parse DTC pairs from the data
            dtc_offsets = [2, 4, 6, 8]  # DTC1 at data[2:4], DTC2 at data[4:6], etc.

            for offset in dtc_offsets:
                if offset + 1 >= len(isotp_data):
                    break
                b1 = isotp_data[offset]
                b2 = isotp_data[offset + 1]
                if b1 == 0xFF and b2 == 0xFF:
                    continue  # Empty slot
                if b1 == 0x00 and b2 == 0x00:
                    continue  # Empty slot
                code = self._parse_dtc_bytes(b1, b2)
                desc = self._get_dtc_description(code)
                cat = DTC_CATEGORIES.get(code[0], "Unknown")
                prefix = DTC_PREFIXES.get(code[:2], "Unknown")
                dtcs.append(DTCRecord(
                    code=code, description=desc, category=cat,
                    prefix_type=prefix, is_pending=False,
                    is_permanent=False, module="ECM-Mode21"
                ))

        except Exception as e:
            self.communicator._log(f"[DTCScanner] Error en read_suzuki_dtcs_from_2100: {e}")

        return dtcs

    def read_suzuki_dtcs_mode18(self):
        """
        Lee DTCs extendidos Suzuki via Mode 0x18 (Suzuki-proprietary).

        Algunos ECUs Suzuki responden a Mode 18 con sub-funciones:
          18 00 FF = Report all DTCs (status mask = 0xFF)
          18 02 FF = Report DTCs con snapshot data
        """
        dtcs = []
        for subcmd in ["1800FF", "1802FF"]:
            try:
                response = self.communicator.send_command(subcmd, timeout=2.0)
                if "NO DATA" in response or "ERROR" in response or "7F" in response:
                    continue

                # Parse response
                data = parse_isotp_response(response)
                if not data or len(data) < 3:
                    continue

                # Skip response header (mode 58 + subfunction)
                start = 3 if len(data) > 3 else 2
                for i in range(start, len(data) - 1, 2):
                    b1, b2 = data[i], data[i + 1]
                    if b1 == 0xFF and b2 == 0xFF:
                        continue
                    if b1 == 0x00 and b2 == 0x00:
                        continue
                    code = self._parse_dtc_bytes(b1, b2)
                    desc = self._get_dtc_description(code)
                    cat = DTC_CATEGORIES.get(code[0], "Unknown")
                    prefix = DTC_PREFIXES.get(code[:2], "Unknown")
                    dtcs.append(DTCRecord(
                        code=code, description=desc, category=cat,
                        prefix_type=prefix, is_pending=False,
                        is_permanent=False, module="ECM-Mode18"
                    ))
            except Exception as e:
                self.communicator._log(f"[DTCScanner] Error en Mode 18: {e}")
                continue

        return dtcs
# ==============================================================================
# MOTOR DE DIAGNOSTICO SUZUKI - SUZUKI SCANNER ENGINE
# ==============================================================================
# Componentes:
#   SafetyGuard   - Proteccion contra lockout, saturacion, intervalos minimos
#   SafeProxy     - Cola de comandos con ACK y timeout dinamico
#   SessionManager - Control de sesiones UDS (0x10)
#   TopologyScanner - Detecta modulos presentes en el bus CAN
#   SuzukiScannerEngine - Orquestador principal
# ==============================================================================

class ModuleStatus(Enum):
    UNKNOWN = auto()
    PRESENT = auto()
    MISSING = auto()
    ERROR_BUS = auto()


class UDSSessionType(Enum):
    DEFAULT = 0x01
    PROGRAMMING = 0x02
    EXTENDED_DIAGNOSTIC = 0x03
    SAFETY_SYSTEM = 0x04


class SafetyGuard:
    """
    Protege contra operaciones de riesgo en el bus CAN.

    - Lockout: bloquea accesos de seguridad tras N fallos consecutivos
    - MinInterval: garantiza 20ms entre comandos a modulos criticos (ABS, SRS)
    - BusSaturation: detecta inundacion del bus y ejecuta safety_shutdown
    - SafetyShutdown: cierra puerto si el bus se satura
    """

    CRITICAL_MODULES = {"ABS", "SRS", "EPS", "DSBS"}
    MIN_INTERVAL_MS = 20
    MAX_BUS_LOAD = 0.85
    LOCKOUT_THRESHOLD = 3
    LOCKOUT_RESET_CYCLES = 5

    def __init__(self):
        self._lock = threading.Lock()
        self._last_cmd_times: Dict[str, float] = {}
        self._security_failures: Dict[str, int] = {}
        self._locked_modules: Dict[str, int] = {}
        self._bus_load_samples: deque = deque(maxlen=20)
        self._shutdown_triggered = False
        self._cycle_count = 0

    def check_module(self, module_name: str) -> bool:
        """
        Verifica si un modulo puede recibir comandos.
        Retorna False si el modulo esta lockeado.
        """
        with self._lock:
            if module_name in self._locked_modules:
                remaining = self._locked_modules[module_name] - self._cycle_count
                if remaining > 0:
                    return False
                else:
                    del self._locked_modules[module_name]
            return True

    def record_security_failure(self, module_name: str):
        """
        Registra un fallo de seguridad (ej: SecurityAccess denegado).
        Si excede LOCKOUT_THRESHOLD, bloquea el modulo hasta reset de ciclo.
        """
        with self._lock:
            count = self._security_failures.get(module_name, 0) + 1
            self._security_failures[module_name] = count
            if count >= self.LOCKOUT_THRESHOLD:
                self._locked_modules[module_name] = self.LOCKOUT_RESET_CYCLES
                self._security_failures[module_name] = 0

    def record_interlock(self, module_name: str):
        """Registra timestamp del ultimo comando para control de intervalo."""
        with self._lock:
            self._last_cmd_times[module_name] = time.time()

    def wait_interval(self, module_name: str):
        """
        Espera el intervalo minimo si el modulo es critico.
        Evita saturar el bus con modulos de seguridad.
        """
        if module_name not in self.CRITICAL_MODULES:
            return
        with self._lock:
            last = self._last_cmd_times.get(module_name, 0)
        elapsed = (time.time() - last) * 1000
        if elapsed < self.MIN_INTERVAL_MS:
            time.sleep((self.MIN_INTERVAL_MS - elapsed) / 1000.0)

    def record_bus_load(self, load: float):
        """Registra muestra de carga del bus. 0.0 a 1.0."""
        with self._lock:
            self._bus_load_samples.append(load)
            avg = sum(self._bus_load_samples) / len(self._bus_load_samples)
            if avg > self.MAX_BUS_LOAD and not self._shutdown_triggered:
                self._shutdown_triggered = True
                return True  # Senal de safety_shutdown
        return False

    def is_safe(self) -> bool:
        """Retorna True si el bus esta en estado seguro."""
        return not self._shutdown_triggered

    def reset_cycles(self):
        """Incrementa contador de ciclos. Desbloquea modulos expirados."""
        self._cycle_count += 1

    def clear(self):
        """Resetea todos los guardas."""
        with self._lock:
            self._security_failures.clear()
            self._locked_modules.clear()
            self._bus_load_samples.clear()
            self._shutdown_triggered = False


class SafeProxy:
    """
    Proxy de comandos con cola, ACK validation y timeout dinamico.

    - Encola peticiones en orden FIFO
    - Espera confirmacion del ELM327 (">") antes de soltar el siguiente
    - Timeout dinamico: 100ms para CAN, 500ms para K-Line
    - Si timeout expira, emite NACK y pasa al siguiente comando
    """

    CAN_TIMEOUT = 0.1
    KLINE_TIMEOUT = 0.5
    MAX_QUEUE_SIZE = 50

    def __init__(self, communicator: 'OBD2ConnectionManager'):
        self.communicator = communicator
        self._cmd_queue: queue.Queue = queue.Queue(maxsize=self.MAX_QUEUE_SIZE)
        self._ack_event = threading.Event()
        self._last_ack = True
        self._worker_thread = None
        self._running = False

    def start(self):
        """Inicia el worker de procesamiento de cola."""
        self._running = True
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()

    def stop(self):
        """Detiene el worker."""
        self._running = False

    def enqueue(self, command: str, module_name: str = "ECM", timeout: float = None) -> bool:
        """
        Encola un comando para ejecucion.
        Retorna False si la cola esta llena.
        """
        if timeout is None:
            timeout = self.CAN_TIMEOUT
            if module_name in ("IMMO", "HVAC", "BCM"):
                timeout = self.KLINE_TIMEOUT

        try:
            self._cmd_queue.put_nowait({
                "command": command,
                "module": module_name,
                "timeout": timeout,
                "timestamp": time.time(),
            })
            return True
        except queue.Full:
            return False

    def _worker_loop(self):
        """Loop interno que procesa la cola de comandos."""
        while self._running:
            try:
                item = self._cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if not self._running:
                break

            cmd = item["command"]
            module = item["module"]
            to = item["timeout"]

            # Enviar comando
            response = self.communicator.send_command(cmd, timeout=to)

            # Verificar ACK (">" en la respuesta = ELM327 listo)
            ack = ">" in response or "OK" in response
            if not ack:
                self._last_ack = False
                self._log(f"NACK: {cmd} -> {response.strip()}")
            else:
                self._last_ack = True

            self._cmd_queue.task_done()

    def _log(self, message):
        if hasattr(self.communicator, '_log'):
            self.communicator._log(message)

    def wait_for_queue(self, timeout: float = 10.0) -> bool:
        """Espera a que la cola se vacie."""
        try:
            self._cmd_queue.join()
            return True
        except Exception:
            return False

    @property
    def queue_size(self) -> int:
        return self._cmd_queue.qsize()

    @property
    def last_ack_ok(self) -> bool:
        return self._last_ack


class SessionManager:
    """
    Gestiona sesiones de diagnostico UDS segun ISO 14229.

    Flujo:
        IDLE -> sessionControl(0x10, DEFAULT) -> DIAGNOSTIC_MODE
        DIAGNOSTIC_MODE -> sessionControl(0x10, EXTENDED) -> EXTENDED_MODE
        EXTENDED_MODE -> testerPresent(0x3E) cada 2s -> mantiene sesion

    Seguridad:
        - No intenta PROGRAMMING ni SAFETY sin autorizacion explicita
        - TesterPresent cada 2s para evitar timeout de sesion
        - Si falla sessionControl, retorna a IDLE
    """

    SESSION_TIMEOUT = 5.0
    TESTER_PRESENT_INTERVAL = 2.0

    def __init__(self, communicator: 'OBD2ConnectionManager'):
        self.communicator = communicator
        self.current_session = UDSSessionType.DEFAULT
        self._tester_timer = None
        self._running = False

    def open_session(self, session_type: UDSSessionType = UDSSessionType.EXTENDED_DIAGNOSTIC,
                     module: SuzukiModule = SuzukiModule.ECM) -> bool:
        """
        Abre una sesion de diagnostico UDS.

        Envia 0x10 + sessionType al modulo especificado.
        La respuesta esperada es 0x50 + sessionType.
        """
        if not self.communicator.is_connected():
            return False

        # Cambiar al modulo destino
        self.communicator.set_module(module)

        # 0x10 03 = Extended Diagnostic Session
        cmd = f"10{session_type.value:02X}"
        response = self.communicator.send_command(cmd, timeout=1.0)

        # Verificar respuesta positiva (0x50 + sessionType)
        expected = f"50{session_type.value:02X}"
        if expected in response.replace(" ", ""):
            self.current_session = session_type
            self._running = True
            self._start_tester_present()
            return True

        # Si fallo, verificar NRC (Negative Response Code)
        if "7F" in response:
            nrc_hex = response.split()[-1] if response.split() else "00"
            try:
                nrc = int(nrc_hex, 16)
                nrc_desc = UDS_NEGATIVE_RESPONSE_CODES.get(nrc, f"Desconocido (0x{nrc:02X})")
                self.communicator._log(f"Session {session_type.name} denegada: {nrc_desc}")
            except ValueError:
                pass

        return False

    def close_session(self):
        """Cierra la sesion actual retornando a DEFAULT."""
        self._stop_tester_present()
        self.communicator.send_command("1001", timeout=0.5)
        self.current_session = UDSSessionType.DEFAULT
        self._running = False

    def _start_tester_present(self):
        """Inicia el envio periodico de TesterPresent (0x3E) para mantener sesion."""
        self._tester_timer = threading.Thread(target=self._tester_loop, daemon=True)
        self._tester_timer.start()

    def _stop_tester_present(self):
        self._running = False
        if hasattr(self, '_tester_timer') and self._tester_timer and self._tester_timer.is_alive():
            self._tester_timer.join(timeout=1.0)

    def _tester_loop(self):
        """Envia 0x3E cada 2s para evitar timeout de sesion."""
        while self._running:
            self.communicator.send_command("3E", timeout=0.5)
            time.sleep(self.TESTER_PRESENT_INTERVAL)


class TopologyScanner:
    """
    Escanea el bus CAN para detectar modulos electronicos presentes.

    Algoritmo:
        1. Para cada SuzukiModule, envia ATSH <tx_id> + 0100 (ping)
        2. Si responde con datos validos -> ModuleStatus.PRESENT
        3. Si responde con "NO DATA" o timeout -> ModuleStatus.MISSING
        4. Si responde con error de bus -> ModuleStatus.ERROR_BUS

    Seguridad:
        - Intervalo minimo de 50ms entre pings
        - Timeout de 300ms por modulo
        - No escanea modulos bloqueados por SafetyGuard
    """

    PING_TIMEOUT = 0.3
    PING_INTERVAL = 0.05
    RESPONSE_COMMANDS = ["0100", "ATRV"]

    def __init__(self, communicator: 'OBD2ConnectionManager', safety: SafetyGuard = None):
        self.communicator = communicator
        self.safety = safety or SafetyGuard()
        self.results: Dict[SuzukiModule, ModuleStatus] = {}
        self.module_info: Dict[str, Dict] = {}
        self._scanning = False

    def scan_all(self, modules: List[SuzukiModule] = None) -> Dict[SuzukiModule, ModuleStatus]:
        """
        Escanea todos los modulos o una sublista.

        Retorna dict {module: status}.
        """
        self._scanning = True
        self.results.clear()

        targets = modules or list(SuzukiModule)
        self.communicator._log(f"Iniciando escaneo topologico: {len(targets)} modulos")

        for module in targets:
            if not self._scanning:
                break

            if self.safety and not self.safety.check_module(module.name):
                self.results[module] = ModuleStatus.ERROR_BUS
                continue

            status = self._ping_module(module)
            self.results[module] = status

            if status == ModuleStatus.PRESENT:
                self.communicator._log(f"  [+] {module.name} ({module.label}) - PRESENTE")
                self._collect_module_info(module)
            else:
                self.communicator._log(f"  [-] {module.name} ({module.label}) - {status.name}")

            self.safety.wait_interval(module.name)

        self.communicator._log(f"Escaneo completo: {sum(1 for s in self.results.values() if s == ModuleStatus.PRESENT)} modulos presentes")
        self._scanning = False
        return dict(self.results)

    def _ping_module(self, module: SuzukiModule) -> ModuleStatus:
        """
        Envia un ping al modulo y determina su estado.

        Usa 0100 (PIDs supported) como comando de deteccion estandar.
        """
        try:
            self.communicator.set_module(module)
            response = self.communicator.send_command("0100", timeout=self.PING_TIMEOUT)

            if not response or "ERROR" in response:
                return ModuleStatus.ERROR_BUS

            clean = response.replace(" ", "").replace("\\r", "").replace("\\n", "").replace(">", "")

            # Respuesta positiva: modo 41 + PID 00
            if "4100" in clean:
                return ModuleStatus.PRESENT

            # UDS responde con 0x50 o 0x7F
            if "5000" in clean or "7F00" in clean:
                return ModuleStatus.PRESENT

            if "NODATA" in clean.upper() or "UNABLE" in clean.upper():
                return ModuleStatus.MISSING

            return ModuleStatus.MISSING

        except Exception:
            return ModuleStatus.ERROR_BUS

    def _collect_module_info(self, module: SuzukiModule):
        """
        Intenta recolectar informacion extendida del modulo:
        - Part number / ECU name (0x22 F180)
        - Calibration ID (0x22 F190)
        """
        info = {"part_number": None, "calibration": None, "bus": module.bus}

        self.communicator.set_module(module)

        # 22 F180 = ECU Part Number / Hardware Number
        try:
            response = self.communicator.send_command("22F180", timeout=0.5)
            if "62" in response and "F180" in response:
                data = self._extract_hex_data(response)
                if data:
                    info["part_number"] = data
        except Exception as e:
            self.communicator._log(f"[TopologyScanner] Error leyendo F180 del {module.name}: {e}")

        # 22 F190 = Calibration ID / Software Version
        try:
            response = self.communicator.send_command("22F190", timeout=0.5)
            if "62" in response and "F190" in response:
                data = self._extract_hex_data(response)
                if data:
                    info["calibration"] = data
        except Exception as e:
            self.communicator._log(f"[TopologyScanner] Error leyendo F190 del {module.name}: {e}")

        self.module_info[module.name] = info

    def _extract_hex_data(self, response: str) -> str:
        """Extrae datos hex de una respuesta UDS y los convierte a ASCII si es texto."""
        try:
            parts = response.replace("\\r", " ").replace(">", "").split()
            # Buscar bytes despues del identificador
            hex_bytes = []
            capture = False
            for p in parts:
                if "F180" in p or "F190" in p:
                    capture = True
                    continue
                if capture:
                    try:
                        hex_bytes.append(int(p, 16))
                    except ValueError:
                        break

            if hex_bytes:
                # Intentar decodificar como ASCII
                try:
                    ascii_text = bytes(hex_bytes).decode("ascii", errors="ignore")
                    if ascii_text.isprintable():
                        return ascii_text.strip()
                except Exception:
                    pass
                return " ".join(f"{b:02X}" for b in hex_bytes)
        except Exception as e:
            self.communicator._log(f"[TopologyScanner] Error extrayendo datos hex: {e}")
        return None

    def stop_scan(self):
        """Detiene el escaneo en curso."""
        self._scanning = False

    @property
    def present_modules(self) -> List[SuzukiModule]:
        """Lista de modulos detectados como presentes."""
        return [m for m, s in self.results.items() if s == ModuleStatus.PRESENT]

    @property
    def summary(self) -> str:
        """Resumen textual del escaneo."""
        total = len(self.results)
        present = len(self.present_modules)
        return f"{present}/{total} modulos presentes"


class SuzukiScannerEngine:
    """
    Orquestador principal del motor de diagnostico Suzuki.

    Integra:
        TopologyScanner    - deteccion de modulos
        SessionManager     - sesiones UDS
        SafeProxy          - cola de comandos segura
        SafetyGuard        - proteccion contra riesgos

    Flujo tipico:
        1. scan_topology() -> detecta modulos presentes
        2. read_all_dtcs()  -> lee DTCs de todos los modulos detectados
        3. read_module_info(module) -> datos extendidos de un modulo especifico
    """

    def __init__(self, communicator: 'OBD2ConnectionManager'):
        self.communicator = communicator
        self.safety = SafetyGuard()
        self.proxy = SafeProxy(communicator)
        self.session = SessionManager(communicator)
        self.topology = TopologyScanner(communicator, self.safety)
        self.dtc_scanner = DTCScanner(communicator)

        self._dtc_results: Dict[str, List[DTCRecord]] = {}
        self._topology_cache: Dict[SuzukiModule, ModuleStatus] = {}
        self._info_cache: Dict[str, Dict] = {}

    def initialize(self):
        """Inicializa el motor y arranca el proxy."""
        self.safety.clear()
        self.proxy.start()
        self.communicator._log("SuzukiScannerEngine inicializado")

    def shutdown(self):
        """Apaga el motor de forma segura."""
        self.proxy.stop()
        self.safety.clear()
        self.communicator._log("SuzukiScannerEngine detenido")

    # ------------------------------------------------------------------
    # TOPOLOGIA
    # ------------------------------------------------------------------

    def scan_topology(self, modules: List[SuzukiModule] = None, force: bool = False) -> Dict[SuzukiModule, ModuleStatus]:
        """
        Escanea la topologia de red del vehiculo.

        Si ya hay cache y force=False, retorna cache.
        """
        if self._topology_cache and not force:
            return dict(self._topology_cache)

        self._topology_cache = self.topology.scan_all(modules)
        self._info_cache = dict(self.topology.module_info)
        return dict(self._topology_cache)

    def get_present_modules(self) -> List[SuzukiModule]:
        """Retorna lista de modulos detectados como presentes."""
        return self.topology.present_modules

    def get_module_info(self, module_name: str) -> Dict:
        """Retorna informacion extendida de un modulo."""
        return self._info_cache.get(module_name, {})

    # ------------------------------------------------------------------
    # DTCs MULTI-MODULO
    # ------------------------------------------------------------------

    def read_all_dtcs(self, modules: List[SuzukiModule] = None) -> Dict[str, List[DTCRecord]]:
        """
        Lee DTCs de todos los modulos presentes (o de una sublista).

        Para modulos UDS usa 0x19 (ReadDTCInformation).
        Para modulos OBD2 estandar usa 0x03.

        Retorna: { "ECM": [DTCRecord, ...], "ABS": [DTCRecord, ...] }
        """
        self._dtc_results.clear()
        targets = modules or self.get_present_modules()

        if not targets:
            self.communicator._log("No hay modulos presentes para escanear DTCs")
            return {}

        self.communicator._log(f"Leyendo DTCs de {len(targets)} modulo(s)...")

        for module in targets:
            if not self.safety.check_module(module.name):
                self.communicator._log(f"  [!] {module.name} bloqueado por safety, saltando")
                continue

            try:
                dtcs = self._read_module_dtcs(module)
                if dtcs:
                    self._dtc_results[module.name] = dtcs
                    self.communicator._log(f"  [{module.name}] {len(dtcs)} DTC(s) encontrados")
                else:
                    self.communicator._log(f"  [{module.name}] Sin DTCs")
            except Exception as e:
                self.communicator._log(f"  [{module.name}] Error: {e}")

            self.safety.wait_interval(module.name)

        return dict(self._dtc_results)

    def _read_module_dtcs(self, module: SuzukiModule) -> List[DTCRecord]:
        """
        Lee DTCs de un modulo especifico usando el protocolo adecuado.

        Para el ECM del Baleno:
          1. Intenta Mode 03 (OBD2 estandar)
          2. Intenta Mode 18 (Suzuki extendido)
          3. Intenta Mode 21 0x00 (DTCs embebidos en bloque de datos)

        Para otros modulos:
          UDS (0x19): para modulos en bus Safety CAN-UDS o Gateway
          OBD2 (0x03): para modulos estandar
        """
        self.communicator.set_module(module)

        dtcs = []

        if module == SuzukiModule.ECM:
            # ===== ECM: Multi-metodo para Baleno =====
            # Metodo 1: OBD2 estandar Mode 03
            for mode, is_pending, is_permanent in [("03", False, False), ("07", True, False), ("0A", False, True)]:
                try:
                    response = self.communicator.send_command(mode, timeout=2.0)
                    parsed = self._parse_obd2_dtcs(response, module, is_pending, is_permanent)
                    dtcs.extend(parsed)
                except Exception:
                    continue

            # Metodo 2: Suzuki Mode 18 (extendido)
            if not dtcs:
                try:
                    parsed = self.dtc_scanner.read_suzuki_dtcs_mode18()
                    dtcs.extend(parsed)
                except Exception:
                    pass

            # Metodo 3: DTCs embebidos en Mode 21 0x00
            if not dtcs:
                try:
                    parsed = self.dtc_scanner.read_suzuki_dtcs_from_2100()
                    dtcs.extend(parsed)
                except Exception:
                    pass
        else:
            is_uds = "UDS" in module.bus

            if is_uds:
                for subfunc in ["0A", "0B"]:
                    try:
                        response = self.communicator.send_command(f"19{subfunc}", timeout=1.0)
                        parsed = self._parse_uds_dtcs(response, module)
                        dtcs.extend(parsed)
                    except Exception:
                        continue
            else:
                for mode, is_pending, is_permanent in [("03", False, False), ("07", True, False), ("0A", False, True)]:
                    try:
                        response = self.communicator.send_command(mode, timeout=1.0)
                        parsed = self._parse_obd2_dtcs(response, module, is_pending, is_permanent)
                        dtcs.extend(parsed)
                    except Exception:
                        continue

        return dtcs

    def _parse_obd2_dtcs(self, response: str, module: SuzukiModule,
                          is_pending: bool = False, is_permanent: bool = False) -> List[DTCRecord]:
        """Parsea respuesta OBD2 modo 03/07/0A a lista de DTCRecord."""
        dtcs = []
        if "NO DATA" in response or "ERROR" in response or not response:
            return dtcs

        lines = response.strip().split("\r")
        for line in lines:
            line = line.strip().replace(" ", "")
            if not line or line == ">" or len(line) < 6:
                continue

            if any(line.startswith(h) for h in ["43", "47", "4A"]):
                if line.startswith("7E"):
                    for h in ["43", "47", "4A"]:
                        idx = line.find(h)
                        if idx >= 0:
                            line = line[idx:]
                            break

                data = line[2:]
                for i in range(0, len(data), 4):
                    chunk = data[i:i+4]
                    if len(chunk) == 4:
                        try:
                            b1 = int(chunk[0:2], 16)
                            b2 = int(chunk[2:4], 16)
                            if b1 == 0 and b2 == 0:
                                continue
                            code = self._parse_dtc_bytes(b1, b2)
                            desc = self._get_dtc_description(code)
                            cat = DTC_CATEGORIES.get(code[0], "Unknown")
                            ptype = DTC_PREFIXES.get(code[:2], "Unknown")
                            dtcs.append(DTCRecord(
                                code=code, description=desc, category=cat,
                                prefix_type=ptype, is_pending=is_pending,
                                is_permanent=is_permanent, module=module.name
                            ))
                        except ValueError:
                            continue
        return dtcs

    def _parse_uds_dtcs(self, response: str, module: SuzukiModule) -> List[DTCRecord]:
        """Parsea respuesta UDS 0x19 a lista de DTCRecord."""
        dtcs = []
        if "NO DATA" in response or "ERROR" in response or "7F" in response or not response:
            return dtcs

        lines = response.strip().split("\r")
        for line in lines:
            line = line.strip().replace(" ", "")
            if not line or line == ">" or len(line) < 6:
                continue

            # UDS response: 0x59 + subfunction + data
            if "59" in line:
                idx = line.find("59")
                if idx >= 0:
                    line = line[idx:]
                data = line[4:] if len(line) > 4 else ""
                for i in range(0, len(data), 4):
                    chunk = data[i:i+4]
                    if len(chunk) == 4:
                        try:
                            b1 = int(chunk[0:2], 16)
                            b2 = int(chunk[2:4], 16)
                            if b1 == 0 and b2 == 0:
                                continue
                            code = self._parse_dtc_bytes(b1, b2)
                            desc = self._get_dtc_description(code)
                            cat = DTC_CATEGORIES.get(code[0], "Unknown")
                            ptype = DTC_PREFIXES.get(code[:2], "Unknown")
                            dtcs.append(DTCRecord(
                                code=code, description=desc, category=cat,
                                prefix_type=ptype, module=module.name
                            ))
                        except ValueError:
                            continue
        return dtcs

    def _parse_dtc_bytes(self, byte1: int, byte2: int) -> str:
        """Convierte 2 bytes a string DTC."""
        cat_map = {0x00: "P", 0x40: "C", 0x80: "B", 0xC0: "U"}
        cat = cat_map.get(byte1 & 0xC0, "P")
        d2 = (byte1 >> 4) & 0x03
        d3 = byte1 & 0x0F
        d4 = (byte2 >> 4) & 0x0F
        d5 = byte2 & 0x0F
        return f"{cat}{d2}{d3:X}{d4:X}{d5:X}"

    def _get_dtc_description(self, code: str) -> str:
        """Busca descripcion de DTC en base de datos Suzuki."""
        if code in SUZUKI_DTC_DATABASE:
            return SUZUKI_DTC_DATABASE[code]
        prefix = code[:2]
        cat = DTC_CATEGORIES.get(code[0], "Unknown")
        ptype = DTC_PREFIXES.get(prefix, "Unknown")
        return f"[{ptype}] {cat} - No documentado"

    def get_dtc_summary(self) -> str:
        """Resumen de DTCs encontrados por modulo."""
        if not self._dtc_results:
            return "Sin DTCs"
        parts = []
        for module, dtcs in self._dtc_results.items():
            parts.append(f"{module}: {len(dtcs)}")
        return " | ".join(parts)

    # ------------------------------------------------------------------
    # LECTURA POR MODULO ESPECIFICO
    # ------------------------------------------------------------------

    def read_module_pid(self, module: SuzukiModule, pid_key: str) -> Optional[Any]:
        """
        Lee un PID de un modulo especifico.

        Cambia al modulo, envia el comando y parsea la respuesta.
        """
        if not self.safety.check_module(module.name):
            return None

        pid_def = ALL_PIDS.get(pid_key)
        if not pid_def:
            return None

        self.communicator.set_module(module)
        cmd = f"{pid_def.mode}{pid_def.pid.zfill(2)}"
        response = self.communicator.send_command(cmd, timeout=1.0)

        if "NO DATA" in response or "ERROR" in response:
            return None

        lines = response.strip().split("\r")
        for line in lines:
            line = line.strip()
            if not line or line == ">":
                continue
            parts = line.split()
            try:
                if parts[0].startswith("7E"):
                    parts = parts[2:]
                if len(parts) >= 2:
                    data_bytes = [int(p, 16) for p in parts[2:]]
                    if len(data_bytes) >= pid_def.bytes_count:
                        return pid_def.decode_func(data_bytes[:pid_def.bytes_count])
            except (ValueError, IndexError):
                continue
        return None

class OBD2ConnectionManager:
    """
    Gestiona la conexion OBD2 con adaptador ELM327 via puerto COM.

    Caracteristicas profesionales:
    - Cache de protocolo por MAC address (reconexion <1s)
    - Jerarquia de negociacion: cache -> CAN 11bit/500k -> barrido completo
    - Maquina de estados con watchdog y reconexion silenciosa
    - Heartbeat (is_alive) para detectar estado zombie del adaptador
    - Thread-safe mediante Lock
    """

    # Archivo de cache donde se guarda el protocolo detectado por MAC
    CACHE_FILE = os.path.join(os.path.dirname(__file__), ".obd2_cache.json")

    # Tiempo entre latidos (heartbeat) cuando no hay trafico
    HEARTBEAT_INTERVAL = 5.0

    # Intentos de reconexion silenciosa antes de rendirse
    MAX_RECOVERY_ATTEMPTS = 3

    def __init__(self, config):
        self.config = config
        self.serial_port = None
        self.simulator = None
        self.state = ConnectionState.DISCONNECTED
        self.current_protocol = None
        self.current_module = SuzukiModule.ECM
        self._lock = threading.Lock()
        self._log_callback = None
        self._last_activity = 0.0
        self._recovery_attempts = 0
        self._heartbeat_timer = None
        self._watchdog_active = False

    # ==========================================================================
    # LOGGING
    # ==========================================================================

    def set_log_callback(self, callback):
        self._log_callback = callback

    def _log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] {message}"
        if self._log_callback:
            self._log_callback(log_msg)

    # ==========================================================================
    # SIMULACION
    # ==========================================================================

    def enable_simulation(self):
        """Activa modo simulacion (sin hardware real)."""
        self.simulator = ELM327Simulator()
        self.simulator.connect()
        self.state = ConnectionState.CONNECTED
        self._log("[SIM] Modo simulacion activado")

    def disable_simulation(self):
        if self.simulator:
            self.simulator.disconnect()
            self.simulator = None
        self.state = ConnectionState.DISCONNECTED
        self._log("[SIM] Modo simulacion desactivado")

    def is_simulation(self):
        return self.simulator is not None

    # ==========================================================================
    # DETECCION DE PUERTOS
    # ==========================================================================

    def list_ports(self):
        """Lista puertos COM disponibles en el sistema."""
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append(f"{p.device} - {p.description}")
        return ports

    # ==========================================================================
    # GESTION DE CACHE DE PROTOCOLO (por MAC)
    # ==========================================================================

    def _get_mac_from_port(self, port_name):
        """
        Obtiene la MAC del adaptador desde el puerto COM.
        En Windows, algunos drivers reportan el serial number via pyserial.
        """
        try:
            for p in serial.tools.list_ports.comports():
                if p.device == port_name:
                    # Usar serial_number o hwid como identificador unico
                    return p.serial_number or p.hwid or port_name
        except Exception:
            pass
        return port_name

    def _load_cache(self):
        """Carga el archivo de cache de protocolo."""
        try:
            if os.path.exists(self.CACHE_FILE):
                with open(self.CACHE_FILE, "r") as f:
                    return json.load(f)
        except (json.JSONDecodeError, IOError):
            pass
        return {}

    def _save_cache(self, cache):
        """Guarda el archivo de cache de protocolo."""
        try:
            with open(self.CACHE_FILE, "w") as f:
                json.dump(cache, f, indent=2)
        except IOError:
            pass

    def _get_cached_protocol(self, mac_id):
        """Recupera el protocolo cacheado para una MAC."""
        cache = self._load_cache()
        entry = cache.get(mac_id, {})
        return entry.get("protocol")

    def _set_cached_protocol(self, mac_id, protocol):
        """Guarda el protocolo detectado asociado a una MAC."""
        cache = self._load_cache()
        cache[mac_id] = {
            "protocol": protocol,
            "last_seen": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self._save_cache(cache)

    # ==========================================================================
    # CONEXION JERARQUICA
    # ==========================================================================

    def connect(self):
        """
        Establece conexion con negociacion jerarquica:

        1. Si hay caché para esta MAC -> intenta AT SP <cacheado>
        2. Si falla -> intenta AT SP 6 (CAN 11bit/500k - mas comun)
        3. Si falla -> AT SP 0 (barrido completo) y guarda en caché
        """
        if self.simulator:
            return True

        self.state = ConnectionState.CONNECTING
        self._log(f"Conectando a {self.config.port} @ {self.config.baudrate} baud...")

        # Obtener identificador unico del adaptador para cache
        self._adapter_mac = self._get_mac_from_port(self.config.port)

        # Intentar apertura del puerto con deteccion automatica de baudrate
        if not self._open_port():
            self.state = ConnectionState.ERROR
            self._log("ERROR: No se pudo abrir el puerto COM")
            return False

        self.state = ConnectionState.INITIALIZING
        return self._negotiate_protocol()

    def _open_port(self):
        """Abre el puerto COM probando varios baudrates."""
        bauds_to_try = [self.config.baudrate] + [b for b in BAUD_RATES if b != self.config.baudrate]
        old_port = self.serial_port

        for baud in bauds_to_try:
            try:
                port = serial.Serial(
                    port=self.config.port,
                    baudrate=baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.config.timeout,
                    write_timeout=self.config.timeout,
                )
                port.reset_input_buffer()
                port.reset_output_buffer()
                time.sleep(0.5)

                # ATZ: Reset del ELM327 - devuelve "ELM327 vX.X" si funciona
                self.serial_port = port
                response = self._send_raw("ATZ", timeout=3.0)
                if "ELM" in response or ">" in response:
                    self.config.baudrate = baud
                    self._log(f"Puerto abierto @ {baud} baud")
                    return True
                else:
                    self.serial_port = old_port
                    port.close()
            except Exception:
                self.serial_port = old_port
                try:
                    port.close()
                except Exception:
                    pass
                continue
        return False

    def _negotiate_protocol(self):
        """
        Negociacion del protocolo para Suzuki Baleno 2019+.

        El Baleno NO soporta OBD2 estandar (Mode 01).
        Usa Suzuki-proprietary Mode 21 sobre CAN (7E0/7E8) a 500kbps.
        Requiere ATH1 (headers ON) para parsear ISO-TP multi-frame.
        """
        # Paso 1: Reset limpio del ELM327
        self._log("Reseteando ELM327...")
        self._send_raw("ATZ", timeout=3.0)
        time.sleep(1.0)

        # Paso 2: Configuracion base
        base_cmds = [
            ("ATE0", "Echo OFF"),
            ("ATL1", "Linefeeds ON"),
            ("ATS1", "Spaces ON (necesario para parser)"),
            ("ATH1", "Headers ON (NECESARIO para ISO-TP multi-frame)"),
            ("ATSP6", "Protocolo CAN 11bit 500k"),
            ("ATCAF1", "CAN Auto Format ON"),
            ("ATCFC1", "CAN Flow Control ON"),
            ("ATST96", "Timeout 960ms"),
            ("ATSH7E0", "CAN Header ECM (7E0)"),
        ]

        for cmd, desc in base_cmds:
            response = self._send_raw(cmd)
            if "OK" not in response:
                self._log(f"ADVERTENCIA: {desc} fallo: {response.strip()}")
            time.sleep(0.05)

        # Paso 3: Verificar con Mode 21 PID 0x00 (Suzuki-proprietary)
        self._log("Verificando conexion con Suzuki Mode 21...")
        test = self._send_raw("2100", timeout=3.0)
        self._log(f"Test 2100: {test.strip()[:120]}")

        if "7E8" in test or "6100" in test or "61" in test:
            self.current_protocol = "6"
            self._log("Suzuki Mode 21 detectado exitosamente!")
            self.state = ConnectionState.CONNECTED
            self._start_watchdog()
            return True

        # Paso 4: Fallback - intentar OBD2 estandar
        self._log("Mode 21 no respondio, probando OBD2 estandar...")
        self._send_raw("ATSH7DF", timeout=1.0)  # Broadcast para test OBD2
        test = self._send_raw("0100", timeout=3.0)
        if "NO DATA" not in test and "UNABLE" not in test and "7E8" in test:
            self._log("OBD2 estandar detectado (no Baleno)")
            self._send_raw("ATSH7E0")  # Restaurar header ECM
            self.current_protocol = "6"
            self.state = ConnectionState.CONNECTED
            self._start_watchdog()
            return True

        # Paso 5: Forzar CAN de todas formas (puede tardar en responder)
        self._log("Forzando CAN Protocolo 6...")
        self._send_raw("ATSH7E0")
        self._send_raw("ATSP6")
        time.sleep(0.5)

        test = self._send_raw("2100", timeout=5.0)
        if "7E8" in test or "61" in test:
            self.current_protocol = "6"
            self._log("Suzuki Mode 21 detectado (retry)!")
            self.state = ConnectionState.CONNECTED
            self._start_watchdog()
            return True

        # Ultimo recurso: marcar conectado de todas formas
        self._log("Usando CAN Protocolo 6 (fallback sin confirmacion)")
        self.current_protocol = "6"
        self.state = ConnectionState.CONNECTED
        self._start_watchdog()
        return True

    # ==========================================================================
    # DESCONEXION
    # ==========================================================================

    def disconnect(self):
        """Cierra la conexion de forma segura."""
        self._stop_watchdog()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.state = ConnectionState.DISCONNECTED
        self._recovery_attempts = 0
        self._log("Desconectado")

    # ==========================================================================
    # WATCHDOG Y HEARTBEAT
    # ==========================================================================

    def _start_watchdog(self):
        """Activa el watchdog que monitorea la salud de la conexion."""
        self._watchdog_active = True
        self._last_activity = time.time()
        self._check_health()

    def _stop_watchdog(self):
        """Desactiva el watchdog."""
        self._watchdog_active = False
        if self._heartbeat_timer and self._heartbeat_timer.is_alive():
            self._heartbeat_timer.cancel()
            self._heartbeat_timer = None

    def is_alive(self):
        """
        Verifica que el adaptador responde (latido).

        Usa ATRV (lectura de voltaje) que NO modifica la configuracion.
        ATZ NO se usa porque resetea el ELM327 y borra ATSP6/ATH1/etc.
        """
        now = time.time()
        if now - self._last_activity < self.HEARTBEAT_INTERVAL:
            return True

        if self.simulator:
            self._last_activity = now
            return True

        if not self.serial_port or not self.serial_port.is_open:
            return False

        try:
            # ATRV: lee voltaje de bateria - NO modifica configuracion
            response = self._send_raw("ATRV", timeout=2.0)
            ok = "V" in response or ">" in response
            if ok:
                self._last_activity = now
                self._recovery_attempts = 0
            return ok
        except Exception:
            return False

    def _check_health(self):
        """Callback interno del watchdog. Se llama periodicamente."""
        if not self._watchdog_active:
            return

        if self.state == ConnectionState.CONNECTED and not self.is_alive():
            self._log("ADVERTENCIA: Heartbeat fallo - adaptador no responde")
            self._initiate_recovery()

        # Reprogramar el chequeo
        if self._watchdog_active:
            self._heartbeat_timer = threading.Timer(
                self.HEARTBEAT_INTERVAL, self._check_health
            )
            self._heartbeat_timer.daemon = True
            self._heartbeat_timer.start()

    def _initiate_recovery(self):
        """
        Reconexion silenciosa sin intervencion del usuario.

        Cierra el puerto, espera 500ms, reabre y re-negocia el protocolo.
        Si falla MAX_RECOVERY_ATTEMPTS veces seguidas, pasa a ERROR.
        """
        if self._recovery_attempts >= self.MAX_RECOVERY_ATTEMPTS:
            self._log(f"ERROR: {self.MAX_RECOVERY_ATTEMPTS} intentos de recuperacion fallidos")
            self.state = ConnectionState.ERROR
            self._stop_watchdog()
            return

        self._recovery_attempts += 1
        old_state = self.state
        self.state = ConnectionState.RECOVERING
        self._log(f"Recuperacion intento {self._recovery_attempts}/{self.MAX_RECOVERY_ATTEMPTS}...")

        # Cerrar puerto
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass
        self.serial_port = None

        # Esperar 500ms para que el adaptador se resetee
        time.sleep(0.5)

        # Reabrir y re-negociar
        if self._open_port():
            if self._negotiate_protocol():
                self._recovery_attempts = 0
                self._log("Recuperacion exitosa")
                return

        # Si llegamos aqui, fallo la recuperacion
        self.state = old_state
        self._log(f"Recuperacion fallida, reintentando en breve...")

    # ==========================================================================
    # COMANDOS THREAD-SAFE
    # ==========================================================================

    def send_command(self, command, timeout=None):
        """
        Envia un comando al adaptador de forma thread-safe.

        Hilo seguro mediante threading.Lock.
        Si falla por timeout, dispara recuperacion silenciosa.
        """
        with self._lock:
            result = self._send_raw(command, timeout)

        # Marcar actividad para el heartbeat
        self._last_activity = time.time()

        # Si el comando fallo y estamos conectados, iniciar recuperacion
        if result.startswith("ERROR") and self.state == ConnectionState.CONNECTED:
            # Disparar watchdog en hilo separado para no bloquear
            threading.Thread(target=self._initiate_recovery, daemon=True).start()

        return result

    def send_obd2_command(self, mode, pid, module=None):
        """Envia un comando OBD2 modo/pid, opcionalmente para un modulo especifico."""
        if module:
            self.set_module(module)
        cmd = f"{mode}{pid.zfill(2)}"
        return self.send_command(cmd)

    def _send_raw(self, command, timeout=None):
        """
        Envia un comando raw al adaptador y lee la respuesta.

        Usa \\r (carriage return, 0x0D) como terminacion, que es lo que
        espera el ELM327 segun el datasheet de ELM Electronics.
        """
        if self.simulator:
            return self.simulator.process_command(command)

        if not self.serial_port or not self.serial_port.is_open:
            return "ERROR: No conectado"

        try:
            timer = threading.Timer(timeout or COMMAND_TIMEOUT, self._timeout_guard)
            timer.daemon = True
            timer.start()

            # Carriage return (0x0D) - terminacion estandar ELM327
            self.serial_port.write((command + "\r").encode())
            self.serial_port.flush()

            response_lines = []
            start_time = time.time()
            to = timeout or self.config.timeout

            while time.time() - start_time < to:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode("ascii", errors="ignore")
                    response_lines.append(line)
                    if ">" in line:
                        break
                time.sleep(0.01)

            timer.cancel()
            response = "".join(response_lines)
            self._log(f"TX: {command} | RX: {response.strip()}")
            return response

        except serial.SerialTimeoutException:
            return "ERROR: SerialTimeout"
        except Exception as e:
            self._log(f"ERROR enviando comando: {e}")
            return f"ERROR: {e}"

    def _timeout_guard(self):
        """Protege contra timeouts que dejarian el hilo colgado."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.cancel_read()
            except Exception:
                pass

    # ==========================================================================
    # GESTION DE MODULOS (CABECERAS CAN)
    # ==========================================================================

    def set_module(self, module):
        """
        Cambia el modulo de diagnostico activo.

        Envia ATSH <id_tx> para redirigir las tramas CAN al modulo
        correspondiente (ECM, TCM, ABS, BCM, etc.).
        """
        if module != self.current_module:
            self.current_module = module
            if not self.simulator:
                tx_id = f"{module.tx_id:03X}"
                self.send_command(f"ATSH{tx_id}")
                self._log(f"Modulo cambiado a {module.name} (TX: 0x{tx_id})")
            else:
                self.simulator.current_module = module

    def switch_can_speed(self, speed):
        """
        Cambia la velocidad del bus CAN.

        ATSW96 para CAN de alta velocidad (500kbps - motor/transmision)
        ATSW34 para CAN de media velocidad (125kbps - carroceria/confort)
        """
        self.config.can_speed = speed
        if not self.simulator:
            if speed == CANBusSpeed.HIGH_SPEED:
                self.send_command("ATSW96")
                self._log("CAN Bus: High-Speed 500kbps (Drivetrain)")
            else:
                self.send_command("ATSW34")
                self._log("CAN Bus: Medium-Speed 125kbps (Body/Comfort)")
        else:
            self.simulator.can_speed = speed

    def is_connected(self):
        if self.simulator:
            return self.simulator.connected
        return self.serial_port is not None and self.serial_port.is_open
# ==============================================================================
# SIMULADOR ELM327
# ==============================================================================

class ELM327Simulator:
    """Simulador de dispositivo ELM327 para pruebas sin hardware fisico."""

    def __init__(self):
        self.connected = False
        self.echo = True
        self.headers = False
        self.protocol = "6"
        self.can_speed = CANBusSpeed.HIGH_SPEED
        self.current_module = SuzukiModule.ECM
        self.simulation_data = self._init_simulation_data()

    def _init_simulation_data(self):
        return {
            "rpm": 850.0, "speed": 0.0, "coolant_temp": 88.0,
            "intake_temp": 35.0, "engine_load": 15.0, "throttle": 0.0,
            "maf": 2.5, "fuel_trim_short_b1": 0.0, "fuel_trim_long_b1": 2.0,
            "baro_pressure": 101.3, "map": 35.0, "battery_voltage": 14.2,
            "oil_temp": 95.0, "atf_temp": 75.0, "gear": 1, "tcc_status": 0,
            "wheel_speed_fl": 0.0, "wheel_speed_fr": 0.0,
            "wheel_speed_rl": 0.0, "wheel_speed_rr": 0.0,
            "dtcs": ["P0171", "P0420", "U1100"],
            "pending_dtcs": ["P0300"],
            "permanent_dtcs": [],
            "odometer": 45230.5, "run_time": 3600,
            "distance_mil": 150, "distance_since_clear": 5000,
            "warmups": 45, "fuel_level": 65.0, "ambient_temp": 28.0,
            "engine_torque": 0.0, "ref_torque": 180,
            "eps_current": 0.0, "eps_torque": 0.0, "eps_assist": 0.0,
            "abs_pressure": 0.0, "yaw_rate": 0.0,
            "lateral_accel": 0.0, "steering_angle": 0.0,
            "alternator_load": 25.0, "door_status": 0b00001100,
            "light_status": 0b00000001, "key_position": 2,
            "crash_sensor": 0, "seatbelt_status": 0b00001111,
        }

    def connect(self):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def process_command(self, cmd):
        if not self.connected:
            return "UNABLE TO CONNECT\\r\\r>"
        cmd = cmd.strip().upper()
        if not cmd:
            return "\\r\\r>"

        # Comandos AT
        if cmd == "ATZ":
            self.echo = True
            self.headers = False
            return "ELM327 v1.5\\r\\r>"
        if cmd == "ATE0":
            self.echo = False
            return "OK\\r\\r>"
        if cmd == "ATE1":
            self.echo = True
            return "OK\\r\\r>"
        if cmd == "ATH0":
            self.headers = False
            return "OK\\r\\r>"
        if cmd == "ATH1":
            self.headers = True
            return "OK\\r\\r>"
        if cmd.startswith("ATSP"):
            if len(cmd) > 4:
                self.protocol = cmd[4]
            return "OK\\r\\r>"
        if cmd == "ATRV":
            return f"{self.simulation_data['battery_voltage']:.1f}V\\r\\r>"
        if cmd == "ATI":
            return "ELM327 v1.5\\r\\r>"
        if cmd in ("ATL1", "ATS1", "ATCAF1", "ATCFC1", "ATD", "ATST96", "ATPB96"):
            return "OK\\r\\r>"

        if cmd.startswith("ATSH"):
            try:
                tx_hex = cmd[4:]
                tx_id = int(tx_hex, 16)
                module = SuzukiModule.by_tx(tx_id)
                if module:
                    self.current_module = module
                return "OK\\r\\r>"
            except ValueError:
                return "OK\\r\\r>"

        if cmd in ("ATSW96", "ATSW34"):
            return "OK\\r\\r>"

        # Comandos OBD2
        if cmd == "0100":
            return self._encode_pid("01", "00", [0xFF, 0xFF, 0xFF, 0xFE])
        if cmd == "0101":
            dtc_count = len(self.simulation_data["dtcs"])
            mil_on = 1 if dtc_count > 0 else 0
            return self._encode_pid("01", "01", [(mil_on << 7) | dtc_count, 0x00, 0x00, 0x00])
        if cmd == "0104":
            load = int(self.simulation_data["engine_load"] * 255 / 100)
            return self._encode_pid("01", "04", [load])
        if cmd == "0105":
            temp = int(self.simulation_data["coolant_temp"] + 40)
            return self._encode_pid("01", "05", [temp])
        if cmd == "010B":
            return self._encode_pid("01", "0B", [int(self.simulation_data["map"])])
        if cmd == "010C":
            rpm = int(self.simulation_data["rpm"] * 4)
            return self._encode_pid("01", "0C", [(rpm >> 8) & 0xFF, rpm & 0xFF])
        if cmd == "010D":
            return self._encode_pid("01", "0D", [int(self.simulation_data["speed"])])
        if cmd == "010F":
            return self._encode_pid("01", "0F", [int(self.simulation_data["intake_temp"] + 40)])
        if cmd == "0110":
            maf = int(self.simulation_data["maf"] * 100)
            return self._encode_pid("01", "10", [(maf >> 8) & 0xFF, maf & 0xFF])
        if cmd == "0111":
            throttle = int(self.simulation_data["throttle"] * 255 / 100)
            return self._encode_pid("01", "11", [throttle])
        if cmd == "0142":
            voltage = int(self.simulation_data["battery_voltage"] * 1000)
            return self._encode_pid("01", "42", [(voltage >> 8) & 0xFF, voltage & 0xFF])
        if cmd == "0146":
            return self._encode_pid("01", "46", [int(self.simulation_data["ambient_temp"] + 40)])
        if cmd == "015C":
            return self._encode_pid("01", "5C", [int(self.simulation_data["oil_temp"] + 40)])

        # DTCs
        if cmd == "03":
            return self._encode_dtcs("03", self.simulation_data["dtcs"])
        if cmd == "07":
            return self._encode_dtcs("07", self.simulation_data["pending_dtcs"])
        if cmd == "0A":
            return self._encode_dtcs("0A", self.simulation_data["permanent_dtcs"])
        if cmd == "04":
            self.simulation_data["dtcs"] = []
            self.simulation_data["pending_dtcs"] = []
            return self._encode_pid("04", "00", [0x00])
        if cmd.startswith("0902"):
            vin = "JS3TD94V0K4100001"
            data = [len(vin)] + list(vin.encode())
            return self._encode_pid("09", "02", data)

        return "NO DATA\\r\\r>"

    def _encode_pid(self, mode, pid, data_bytes):
        mode_resp = f"{int(mode, 16) + 0x40:02X}"
        pid_hex = pid.zfill(2)
        data_hex = " ".join(f"{b:02X}" for b in data_bytes)
        return f"{mode_resp} {pid_hex} {data_hex}\\r\\r>"

    def _encode_dtcs(self, mode, dtcs):
        if not dtcs:
            return self._encode_pid(mode, "00", [0x00])
        data = []
        for dtc in dtcs[:3]:
            first_char = dtc[0]
            first_byte = {"P": 0x00, "C": 0x40, "B": 0x80, "U": 0xC0}[first_char]
            first_byte |= (int(dtc[1]) & 0x03) << 4
            first_byte |= int(dtc[2], 16)
            second_byte = int(dtc[3:5], 16)
            data.extend([first_byte, second_byte])
        return self._encode_pid(mode, "00", data)


# ==============================================================================
# DASHBOARD MODULAR - CONFIGURACION DE SECCIONES
# ==============================================================================
# Cada seccion agrupa PIDs que se renderizan como graficos de linea en vivo.
# Formato: {nombre: {icon, pids: [pid_key, ...], layout: (filas, columnas)}}
# ==============================================================================

DASHBOARD_SECTIONS = {
    "Engine": {
        "icon": "\u26a1",
        "pids": ["S21_RPM", "S21_SPD", "S21_ECT", "S21_IAT", "S21_TPS", "S21_MAP"],
        "layout": (2, 3),
    },
    "Fuel & Timing": {
        "icon": "\ud83c\udf2c",
        "pids": ["S21_ADV", "S21_BAT"],
        "layout": (1, 2),
    },
}

CHART_COLORS = {
    "rpm": "#00FF88",
    "speed": "#00D4FF",
    "temp": "#FF6B35",
    "pressure": "#4FC3F7",
    "voltage": "#FFD700",
    "fuel": "#FF4081",
    "position": "#AB47BC",
    "timing": "#FFA726",
    "flow": "#26C6DA",
    "torque": "#66BB6A",
    "current": "#EF5350",
    "angle": "#7E57C2",
    "default": "#00BCD4",
}

def _chart_color(pid_key):
    pid_def = ALL_PIDS.get(pid_key)
    if not pid_def:
        return CHART_COLORS["default"]
    name = pid_def.name.lower()
    unit = pid_def.unit.lower()
    if "rpm" in name or unit == "rpm":
        return CHART_COLORS["rpm"]
    if "temp" in name or unit == "c" or unit == "°c":
        return CHART_COLORS["temp"]
    if "speed" in name or unit == "km/h":
        return CHART_COLORS["speed"]
    if "press" in name or unit == "kpa":
        return CHART_COLORS["pressure"]
    if "volt" in name or unit == "v":
        return CHART_COLORS["voltage"]
    if "fuel" in name or "fuel" in unit:
        return CHART_COLORS["fuel"]
    if "throttle" in name or "position" in name or unit == "%":
        return CHART_COLORS["position"]
    if "timing" in name or "advance" in name:
        return CHART_COLORS["timing"]
    if "maf" in name or "flow" in name or "rate" in name:
        return CHART_COLORS["flow"]
    if "torque" in name:
        return CHART_COLORS["torque"]
    if "current" in name or unit == "a":
        return CHART_COLORS["current"]
    if "angle" in name or "yaw" in name:
        return CHART_COLORS["angle"]
    return CHART_COLORS["default"]


# ==============================================================================
# TREND CHART - Grafico de linea en vivo con Canvas
# ==============================================================================

class TrendChart(ctk.CTkFrame):
    """
    Grafico de linea en tiempo real con estilo automotriz oscuro.

    Caracteristicas:
    - Fondo oscuro con rejilla minimalista
    - Linea de color neon segun tipo de PID
    - Auto-escalado en Y con margen
    - Ventana deslizante de 500 puntos
    - Estado visual 'NO SIGNAL' cuando no hay datos
    - Buffer interno con deque(maxlen=500)
    """

    PADDING = 40
    X_LABEL_SPACE = 30
    Y_TICK_COUNT = 5

    def __init__(self, master, pid_key, title, unit,
                 chart_color=None, min_val=None, max_val=None,
                 window_seconds=30, **kwargs):
        super().__init__(master, **kwargs)
        self.pid_key = pid_key
        self.title = title
        self.unit = unit
        self.chart_color = chart_color or _chart_color(pid_key)
        self.min_val = min_val
        self.max_val = max_val
        self.window_seconds = window_seconds
        self._data = deque(maxlen=500)
        self._timestamps = deque(maxlen=500)
        self._has_signal = False
        self._no_data_count = 0
        self._last_render = 0

        self.configure(fg_color=COLOR_BG_CARD, border_width=1,
                       border_color=COLOR_BORDER)

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # Header: titulo + valor actual + unidad
        header = ctk.CTkFrame(self, fg_color="transparent", height=24)
        header.grid(row=0, column=0, sticky="ew", padx=8, pady=(4, 0))
        header.grid_columnconfigure(0, weight=1)
        header.grid_columnconfigure(1, weight=0)

        self.title_label = ctk.CTkLabel(
            header, text=title.upper(),
            font=("Consolas", 9, "bold"),
            text_color=COLOR_TEXT_SECONDARY)
        self.title_label.grid(row=0, column=0, sticky="w")

        self.value_label = ctk.CTkLabel(
            header, text="--",
            font=("Consolas", 11, "bold"),
            text_color=self.chart_color)
        self.value_label.grid(row=0, column=1, sticky="e")

        self.unit_label = ctk.CTkLabel(
            header, text=unit,
            font=("Consolas", 9),
            text_color=COLOR_TEXT_SECONDARY)
        self.unit_label.grid(row=0, column=2, sticky="e", padx=(4, 0))

        # Canvas del grafico
        self.canvas = ctk.CTkCanvas(
            self, bg=COLOR_BG_CARD,
            highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew", padx=4, pady=(2, 4))

        # NO SIGNAL overlay text (se dibuja en el canvas)
        self._signal_text_id = None

        # Bind resize
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, event=None):
        self._render()

    def push_value(self, value):
        """Inserta un valor en el buffer y actualiza el grafico."""
        now = time.time()
        self._timestamps.append(now)
        self._data.append(value)
        self._has_signal = True
        self._no_data_count = 0
        self.value_label.configure(text=f"{value:.1f}" if isinstance(value, float) else str(value))
        self._render()

    def mark_no_signal(self):
        """Marca el grafico como 'Sin Senal' tras timeout de datos."""
        self._no_data_count += 1
        if self._no_data_count > 3 and self._has_signal:
            self._has_signal = False
            self.value_label.configure(text="NO SIGNAL")
            self._render()

    def _render(self):
        """Renderiza el grafico en el Canvas."""
        if not self.canvas.winfo_exists():
            return
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 50 or h < 50:
            return

        self.canvas.delete("all")

        pad = self.PADDING
        plot_w = w - pad - self.X_LABEL_SPACE
        plot_h = h - pad - 10
        origin_x = pad
        origin_y = 10
        plot_top = origin_y
        plot_bottom = origin_y + plot_h
        plot_right = origin_x + plot_w

        # No signal overlay
        if not self._has_signal or len(self._data) == 0:
            self.canvas.create_text(
                w // 2, h // 2,
                text="NO SIGNAL",
                fill="#555555",
                font=("Consolas", 16, "bold"),
                anchor="center")
            self.canvas.create_line(
                origin_x, plot_bottom, plot_right, plot_bottom,
                fill="#333333", width=1)
            return

        # Determinar rango Y
        data_min = min(self._data) if not self.min_val else self.min_val
        data_max = max(self._data) if not self.max_val else self.max_val
        if data_min == data_max:
            data_min -= 1
            data_max += 1
        y_range = data_max - data_min
        y_margin = y_range * 0.1
        y_min = data_min - y_margin
        y_max = data_max + y_margin
        if y_min == y_max:
            y_max = y_min + 1

        # Rejilla horizontal
        for i in range(self.Y_TICK_COUNT + 1):
            y = origin_y + plot_h * (1 - i / self.Y_TICK_COUNT)
            self.canvas.create_line(
                origin_x, y, plot_right, y,
                fill="#2A2A3A", width=1)
            val = y_min + (y_max - y_min) * i / self.Y_TICK_COUNT
            self.canvas.create_text(
                origin_x - 4, y,
                text=f"{val:.0f}",
                fill="#555555",
                font=("Consolas", 7),
                anchor="e")

        # Eje X (linea base)
        self.canvas.create_line(
            origin_x, plot_bottom, plot_right, plot_bottom,
            fill="#444444", width=1)

        # Linea de datos
        n = len(self._data)
        if n < 2:
            return

        points = []
        for i in range(n):
            x = plot_right - (plot_w * (n - 1 - i) / max(n - 1, 1))
            val = self._data[i]
            if val < y_min:
                y_clamped = plot_top
            elif val > y_max:
                y_clamped = plot_bottom
            else:
                y_clamped = plot_bottom - plot_h * (val - y_min) / (y_max - y_min)
            points.extend([x, y_clamped])

        if len(points) >= 4:
            self.canvas.create_line(
                *points, fill=self.chart_color,
                width=2, smooth=True)

    def clear_data(self):
        self._data.clear()
        self._timestamps.clear()
        self._has_signal = False
        self.value_label.configure(text="--")
        self._render()


# ==============================================================================
# SECTION PANEL - Panel de graficos para una seccion del dashboard
# ==============================================================================

class SectionPanel(ctk.CTkScrollableFrame):
    """
    Panel que contiene una grilla de TrendCharts para una seccion.

    Genera automaticamente los widgets segun la configuracion de PIDs,
    los distribuye en un grid (filas x columnas) y expone metodos
    para recibir datos enrutados.
    """

    def __init__(self, master, section_name, pid_keys, layout=(2, 2), **kwargs):
        super().__init__(master, **kwargs)
        self.section_name = section_name
        self.pid_keys = pid_keys
        self.layout = layout
        self.charts = {}  # pid_key -> TrendChart

        self.configure(fg_color="transparent")
        self._build_grid()

    def _build_grid(self):
        rows, cols = self.layout
        for r in range(rows):
            self.grid_rowconfigure(r, weight=1)
        for c in range(cols):
            self.grid_columnconfigure(c, weight=1)

        for idx, pid_key in enumerate(self.pid_keys):
            if idx >= rows * cols:
                break
            r = idx // cols
            c = idx % cols

            pid_def = ALL_PIDS.get(pid_key)
            if not pid_def:
                continue

            title = pid_def.name.replace("_", " ")
            unit = pid_def.unit
            color = _chart_color(pid_key)

            chart = TrendChart(
                self,
                pid_key=pid_key,
                title=title,
                unit=unit,
                chart_color=color,
                min_val=pid_def.min_val,
                max_val=pid_def.max_val,
            )
            chart.grid(row=r, column=c, padx=4, pady=4, sticky="nsew")
            self.charts[pid_key] = chart

    def route_value(self, pid_key, value):
        """Envia un valor al grafico correspondiente si pertenece a este panel."""
        chart = self.charts.get(pid_key)
        if chart:
            chart.push_value(value)

    def mark_no_signal(self, pid_key):
        chart = self.charts.get(pid_key)
        if chart:
            chart.mark_no_signal()

    def reset_all(self):
        for chart in self.charts.values():
            chart.clear_data()


# ==============================================================================
# DASHBOARD MANAGER - Navegacion por pestanas y enrutamiento de datos
# ==============================================================================

class DashboardManager(ctk.CTkFrame):
    """
    Gestor principal del dashboard modular.

    - Crea una barra de pestanas laterales para cada seccion
    - Muestra/oculta SectionPanels segun seleccion
    - Enruta datos entrantes al panel y grafico correctos
    - Mantiene buffers globales de datos para procesamiento batch
    """

    def __init__(self, master, config=None, **kwargs):
        super().__init__(master, **kwargs)
        self.config = config or DASHBOARD_SECTIONS
        self.panels = {}        # section_name -> SectionPanel
        self._active_section = None
        self._nav_buttons = {}  # section_name -> CTkButton
        self._last_active = None

        self.configure(fg_color="transparent")
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Barra de navegacion (pestanas verticales a la izquierda)
        self.nav_frame = ctk.CTkFrame(self, fg_color=COLOR_BG_CARD,
                                      width=160)
        self.nav_frame.grid(row=0, column=0, sticky="ns", padx=(0, 8))
        self.nav_frame.grid_propagate(False)
        self.nav_frame.grid_columnconfigure(0, weight=1)

        # Contenedor de paneles
        self.panel_container = ctk.CTkFrame(self, fg_color="transparent")
        self.panel_container.grid(row=0, column=1, sticky="nsew")
        self.panel_container.grid_columnconfigure(0, weight=1)
        self.panel_container.grid_rowconfigure(0, weight=1)

        self._build_navigation()
        self._build_panels()

        if self.config:
            first = list(self.config.keys())[0]
            self.show_section(first)

    def _build_navigation(self):
        ctk.CTkLabel(
            self.nav_frame,
            text="SECTIONS",
            font=("Consolas", 10, "bold"),
            text_color=COLOR_TEXT_SECONDARY
        ).grid(row=0, column=0, pady=(12, 6))

        for idx, (name, section) in enumerate(self.config.items()):
            icon = section.get("icon", "")
            btn = ctk.CTkButton(
                self.nav_frame,
                text=f"{icon} {name}",
                command=lambda n=name: self.show_section(n),
                fg_color="transparent",
                hover_color="#1E2A45",
                text_color=COLOR_TEXT_SECONDARY,
                anchor="w",
                font=("Consolas", 11),
                height=32)
            btn.grid(row=idx + 1, column=0, sticky="ew", padx=4, pady=2)
            self._nav_buttons[name] = btn

    def _build_panels(self):
        for name, section in self.config.items():
            panel = SectionPanel(
                self.panel_container,
                section_name=name,
                pid_keys=section["pids"],
                layout=section.get("layout", (2, 2)),
                fg_color="transparent")
            self.panels[name] = panel

    def show_section(self, name):
        """Activa una seccion, mostrando su panel y ocultando el resto."""
        if name == self._active_section:
            return
        self._active_section = name

        for n, panel in self.panels.items():
            if n == name:
                panel.grid(row=0, column=0, sticky="nsew")
                panel.lift()
            else:
                panel.grid_remove()

        for n, btn in self._nav_buttons.items():
            if n == name:
                btn.configure(fg_color="#1E2A45", text_color=COLOR_TEXT_PRIMARY,
                              font=("Consolas", 11, "bold"))
            else:
                btn.configure(fg_color="transparent", text_color=COLOR_TEXT_SECONDARY,
                              font=("Consolas", 11))

    def route_value(self, pid_key, value):
        """Enruta un valor al grafico correcto en la seccion que lo contiene."""
        for panel in self.panels.values():
            if pid_key in panel.charts:
                panel.route_value(pid_key, value)
                return

    def mark_no_signal(self, pid_key):
        for panel in self.panels.values():
            if pid_key in panel.charts:
                panel.mark_no_signal(pid_key)
                return

    def reset_all(self):
        for panel in self.panels.values():
            panel.reset_all()

    def all_pids(self):
        """Devuelve el conjunto completo de PIDs configurados (unicos, ordenados)."""
        seen = set()
        pids = []
        for section in self.config.values():
            for pid in section["pids"]:
                if pid not in seen:
                    seen.add(pid)
                    pids.append(pid)
        return pids

    def get_active_pids(self):
        """Devuelve PIDs de la seccion activa."""
        if self._active_section and self._active_section in self.config:
            return self.config[self._active_section]["pids"]
        return []


# ==============================================================================
# INTERFAZ GRAFICA - WIDGETS PERSONALIZADOS
# ==============================================================================

class GaugeCard(ctk.CTkFrame):
    """
    Widget de medidor estilo tarjeta automotriz.

    Muestra valor grande en el centro, titulo arriba, unidad abajo.
    Color de borde cambia segun el valor (verde=ok, naranja=alerta, rojo=critico).
    """

    def __init__(self, master, title, unit, warning_thresh=None, critical_thresh=None, invert=False, **kwargs):
        super().__init__(master, **kwargs)
        self.title = title
        self.unit = unit
        self.warning_thresh = warning_thresh
        self.critical_thresh = critical_thresh
        self.invert = invert
        self._current_value = None

        self.configure(fg_color=COLOR_BG_CARD, border_width=2, border_color=COLOR_BORDER)
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=0)

        ctk.CTkLabel(self, text=title.upper(), font=("Consolas", 11, "bold"),
                     text_color=COLOR_TEXT_SECONDARY).grid(row=0, column=0, pady=(12, 0))

        self.value_label = ctk.CTkLabel(self, text="--", font=("Consolas", 36, "bold"),
                                        text_color=COLOR_TEXT_PRIMARY)
        self.value_label.grid(row=1, column=0, pady=5)

        self.unit_label = ctk.CTkLabel(self, text=unit, font=("Consolas", 12),
                                       text_color=COLOR_TEXT_SECONDARY)
        self.unit_label.grid(row=2, column=0, pady=(0, 12))

    def update_value(self, value):
        self._current_value = value
        if isinstance(value, float):
            self.value_label.configure(text=f"{value:.1f}")
        else:
            self.value_label.configure(text=str(value))

        # Color coding segun umbrales
        if value is not None and self.warning_thresh is not None:
            border = COLOR_GREEN
            if self.invert:
                if value <= self.critical_thresh:
                    border = COLOR_RED
                elif value <= self.warning_thresh:
                    border = COLOR_ORANGE
            else:
                if value >= self.critical_thresh:
                    border = COLOR_RED
                elif value >= self.warning_thresh:
                    border = COLOR_ORANGE
            self.configure(border_color=border)

    def get_value(self):
        return self._current_value


class LogConsole(ctk.CTkTextbox):
    """Consola de log con auto-scroll."""

    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.configure(state="disabled", fg_color=COLOR_BG_CARD)

    def append(self, message):
        self.configure(state="normal")
        self.insert("end", message + "\n")
        self.see("end")
        self.configure(state="disabled")


# ==============================================================================
# INTERFAZ GRAFICA - APLICACION PRINCIPAL
# ==============================================================================

class Application:
    """
    Aplicacion principal con navegacion por sidebar.

    Arquitectura:
    - Sidebar izquierdo con 4 modulos de navegacion
    - Area de contenido principal que cambia segun modulo seleccionado
    - Barra de estado inferior persistente (conexion, protocolo, FPS)
    - Dark theme con acento Suzuki Racing Red
    """

    NAV_ITEMS = [
        ("dashboard", "Dashboard", "\u25b6"),
        ("diagnostics", "Diagnostic", "\u26a0"),
        ("topology", "Topologia", "\u2630"),
        ("monitors", "Monitors", "\u2714"),
        ("settings", "Settings", "\u2699"),
    ]

    def __init__(self):
        self.app = ctk.CTk()
        self.app.title(f"{APP_NAME} v{APP_VERSION}")
        self.app.geometry("1280x800")
        self.app.minsize(1024, 680)

        ctk.set_appearance_mode("dark")
        self._apply_theme()

        self.config = ConnectionConfig()
        self.communicator = OBD2ConnectionManager(self.config)
        self.pid_manager = PIDManager(self.communicator)

        # Motor de diagnostico profesional
        self.engine = SuzukiScannerEngine(self.communicator)

        self._live_data_running = False
        self._live_data_thread = None
        self._dtc_records = {}
        self._fps_counter = 0
        self._fps_timer = 0
        self._current_fps = 0
        self._current_nav = "dashboard"
        self._module_status = {}
        self._topology_scanned = False
        self._topo_thread = None

        self._build_ui()
        self.communicator.set_log_callback(self._on_log)

        # Iniciar actualizacion periodica de FPS y estado
        self.app.after(1000, self._update_status_bar)

    def _apply_theme(self):
        """Configura el tema oscuro con colores personalizados."""
        ctk.set_default_color_theme("dark-blue")
        self.app.configure(fg_color=COLOR_BG_DARK)

    # ==========================================================================
    # CONSTRUCCION DE LA UI
    # ==========================================================================

    def _build_ui(self):
        """Construye la estructura completa de la interfaz."""
        self.app.grid_columnconfigure(0, weight=0)
        self.app.grid_columnconfigure(1, weight=1)
        self.app.grid_rowconfigure(0, weight=0)
        self.app.grid_rowconfigure(1, weight=1)
        self.app.grid_rowconfigure(2, weight=0)

        self._build_topbar()
        self._build_sidebar()
        self._build_content_area()
        self._build_statusbar()

        # Mostrar vista inicial
        self._show_view("dashboard")

    # ------------------------------------------------------------------
    # TOPBAR
    # ------------------------------------------------------------------

    def _build_topbar(self):
        """Barra superior con titulo y controles de conexion rapida."""
        bar = ctk.CTkFrame(self.app, height=52, fg_color=COLOR_BG_SIDEBAR)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", padx=0, pady=0)
        bar.grid_propagate(False)
        bar.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(bar, text=APP_NAME, font=("Consolas", 16, "bold"),
                     text_color=SUZUKI_RED).grid(row=0, column=0, padx=(20, 30), pady=12)

        # Grupo conexion rapida
        conn_frame = ctk.CTkFrame(bar, fg_color="transparent")
        conn_frame.grid(row=0, column=1, sticky="e", padx=10)

        ctk.CTkLabel(conn_frame, text="Puerto:", font=("Consolas", 11),
                     text_color=COLOR_TEXT_SECONDARY).pack(side="left", padx=5)
        self.port_var = ctk.StringVar(value="COM1")
        self.port_menu = ctk.CTkOptionMenu(conn_frame, variable=self.port_var, values=["COM1"],
                                           width=100, fg_color=COLOR_BG_CARD,
                                           button_color=SUZUKI_RED,
                                           button_hover_color=SUZUKI_RED_HOVER)
        self.port_menu.pack(side="left", padx=3)
        ctk.CTkButton(conn_frame, text="\u21bb", width=30, command=self._refresh_ports,
                      fg_color=COLOR_BG_CARD, hover_color=COLOR_BORDER,
                      text_color=COLOR_TEXT_PRIMARY).pack(side="left", padx=3)

        ctk.CTkLabel(conn_frame, text="Baud:", font=("Consolas", 11),
                     text_color=COLOR_TEXT_SECONDARY).pack(side="left", padx=(15, 3))
        self.baud_var = ctk.StringVar(value=str(DEFAULT_BAUD))
        self.baud_menu = ctk.CTkOptionMenu(conn_frame, variable=self.baud_var,
                                           values=[str(b) for b in BAUD_RATES],
                                           width=80, fg_color=COLOR_BG_CARD,
                                           button_color=SUZUKI_RED,
                                           button_hover_color=SUZUKI_RED_HOVER)
        self.baud_menu.pack(side="left", padx=3)

        self.connect_btn = ctk.CTkButton(conn_frame, text="Conectar",
                                         command=self._toggle_connection, width=110,
                                         fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER,
                                         font=("Consolas", 12, "bold"))
        self.connect_btn.pack(side="left", padx=(15, 5))

        self.simulate_var = ctk.BooleanVar(value=False)
        self.simulate_cb = ctk.CTkCheckBox(conn_frame, text="Sim", variable=self.simulate_var,
                                           checkbox_width=18, checkbox_height=18,
                                           fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER,
                                           font=("Consolas", 10))
        self.simulate_cb.pack(side="left", padx=5)

    # ------------------------------------------------------------------
    # SIDEBAR
    # ------------------------------------------------------------------

    def _build_sidebar(self):
        """Sidebar de navegacion con iconos y labels."""
        self.sidebar = ctk.CTkFrame(self.app, width=180, fg_color=COLOR_BG_SIDEBAR)
        self.sidebar.grid(row=1, column=0, sticky="ns", padx=(0, 0), pady=0)
        self.sidebar.grid_propagate(False)

        self.nav_buttons = {}
        for idx, (key, label, icon) in enumerate(self.NAV_ITEMS):
            btn = ctk.CTkButton(self.sidebar, text=f"  {icon}  {label}",
                                command=lambda k=key: self._navigate(k),
                                anchor="w", height=48,
                                fg_color="transparent",
                                hover_color=COLOR_BG_CARD,
                                text_color=COLOR_TEXT_PRIMARY,
                                font=("Consolas", 13),
                                border_width=0)
            btn.pack(fill="x", padx=8, pady=(4, 0))
            self.nav_buttons[key] = btn

        # Separador
        ctk.CTkFrame(self.sidebar, height=1, fg_color=COLOR_BORDER).pack(fill="x", padx=15, pady=15)

        # Info del vehiculo en sidebar
        self.vin_label = ctk.CTkLabel(self.sidebar, text="VIN: ---", font=("Consolas", 10),
                                      text_color=COLOR_TEXT_SECONDARY)
        self.vin_label.pack(padx=10, pady=2)
        self.module_label = ctk.CTkLabel(self.sidebar, text="Modulo: ECM", font=("Consolas", 10),
                                         text_color=COLOR_TEXT_SECONDARY)
        self.module_label.pack(padx=10, pady=2)

    # ------------------------------------------------------------------
    # CONTENIDO PRINCIPAL
    # ------------------------------------------------------------------

    def _build_content_area(self):
        """Area de contenido principal (cambia segun navegacion)."""
        self.content = ctk.CTkFrame(self.app, fg_color=COLOR_BG_DARK)
        self.content.grid(row=1, column=1, sticky="nsew", padx=10, pady=10)
        self.content.grid_columnconfigure(0, weight=1)
        self.content.grid_rowconfigure(0, weight=1)

        # Contenedores para cada vista (solo se crea el visible)
        self.views = {}
        for key, _, _ in self.NAV_ITEMS:
            frame = ctk.CTkFrame(self.content, fg_color=COLOR_BG_DARK)
            frame.grid_columnconfigure(0, weight=1)
            frame.grid_rowconfigure(0, weight=1)
            self.views[key] = frame

        self._build_dashboard_view()
        self._build_diagnostics_view()
        self._build_topology_view()
        self._build_monitors_view()
        self._build_settings_view()

        # Consola en la parte inferior del area de contenido
        self.console = LogConsole(self.content, height=120, font=("Consolas", 11))
        self.console.grid(row=1, column=0, sticky="ew", padx=0, pady=(10, 0))
        self.console.append(f"{APP_NAME} v{APP_VERSION} iniciado")
        self.console.append("Seleccione puerto y presione Conectar")

    def _show_view(self, key):
        """Muestra la vista seleccionada y oculta las otras."""
        self._current_nav = key
        for k, frame in self.views.items():
            frame.grid_remove() if k != key else frame.grid(row=0, column=0, sticky="nsew")

        # Actualizar estado visual del sidebar
        for k, btn in self.nav_buttons.items():
            if k == key:
                btn.configure(fg_color=COLOR_BG_CARD, border_width=2, border_color=SUZUKI_RED)
            else:
                btn.configure(fg_color="transparent", border_width=0)

    def _navigate(self, key):
        """Navega a una vista. Detiene live data si sale del dashboard."""
        if key != "dashboard" and self._live_data_running:
            self._stop_live_data()
        elif key == "dashboard" and self.communicator.is_connected() and not self._live_data_running:
            self._start_live_data()
        self._show_view(key)

    # ==========================================================================
    # VISTA: DASHBOARD (MODULAR POR SECCIONES)
    # ==========================================================================

    def _build_dashboard_view(self):
        view = self.views["dashboard"]
        view.grid_columnconfigure(0, weight=1)
        view.grid_rowconfigure(0, weight=0)
        view.grid_rowconfigure(1, weight=1)

        # Botones de control del dashboard
        ctrl_frame = ctk.CTkFrame(view, fg_color="transparent")
        ctrl_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ctrl_frame.grid_columnconfigure(0, weight=1)

        self.scan_btn = ctk.CTkButton(ctrl_frame, text="\u25b6  Iniciar Scan",
                                      command=self._toggle_scan, width=160,
                                      fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER,
                                      font=("Consolas", 12, "bold"))
        self.scan_btn.grid(row=0, column=0, sticky="w")

        # Dashboard modular con secciones
        self.dashboard = DashboardManager(view)
        self.dashboard.grid(row=1, column=0, sticky="nsew")

    # ==========================================================================
    # VISTA: DIAGNOSTICS (DTCs)
    # ==========================================================================

    def _build_diagnostics_view(self):
        view = self.views["diagnostics"]
        view.grid_columnconfigure(0, weight=1)
        view.grid_rowconfigure(0, weight=0)
        view.grid_rowconfigure(1, weight=0)
        view.grid_rowconfigure(2, weight=1)

        # Barra de acciones
        action_bar = ctk.CTkFrame(view, fg_color=COLOR_BG_CARD)
        action_bar.grid(row=0, column=0, sticky="ew", padx=0, pady=(0, 10))

        ctk.CTkButton(action_bar, text="\u2714  Leer DTCs",
                      command=self._read_dtcs, width=140,
                      fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER,
                      font=("Consolas", 12, "bold")).pack(side="left", padx=10, pady=10)

        self.clear_dtc_btn = ctk.CTkButton(action_bar, text="\u26a0  Borrar DTCs",
                                           command=self._confirm_clear_dtcs, width=140,
                                           fg_color="#555", hover_color="#444",
                                           font=("Consolas", 12, "bold"))
        self.clear_dtc_btn.pack(side="left", padx=5, pady=10)

        ctk.CTkButton(action_bar, text="Exportar",
                      command=self._export_dtcs, width=100,
                      fg_color=COLOR_BG_SIDEBAR, hover_color=COLOR_BORDER,
                      font=("Consolas", 11)).pack(side="right", padx=10, pady=10)

        self.dtc_count_label = ctk.CTkLabel(action_bar, text="DTCs: 0",
                                           font=("Consolas", 13, "bold"),
                                           text_color=COLOR_GREEN)
        self.dtc_count_label.pack(side="right", padx=15, pady=10)

        # Panel de detalles del DTC seleccionado
        detail_frame = ctk.CTkFrame(view, fg_color=COLOR_BG_CARD)
        detail_frame.grid(row=1, column=0, sticky="ew", padx=0, pady=(0, 10))
        detail_frame.grid_columnconfigure(0, weight=1)
        detail_frame.grid_columnconfigure(1, weight=3)

        ctk.CTkLabel(detail_frame, text="Codigo:", font=("Consolas", 11),
                     text_color=COLOR_TEXT_SECONDARY).grid(row=0, column=0, sticky="w", padx=10, pady=5)
        self.dtc_code_var = ctk.StringVar(value="---")
        ctk.CTkLabel(detail_frame, textvariable=self.dtc_code_var, font=("Consolas", 20, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).grid(row=0, column=1, sticky="w", padx=10, pady=5)

        ctk.CTkLabel(detail_frame, text="Descripcion:", font=("Consolas", 11),
                     text_color=COLOR_TEXT_SECONDARY).grid(row=1, column=0, sticky="w", padx=10, pady=2)
        self.dtc_desc_var = ctk.StringVar(value="Presione 'Leer DTCs' para escanear")
        ctk.CTkLabel(detail_frame, textvariable=self.dtc_desc_var, font=("Consolas", 13),
                     text_color=COLOR_TEXT_PRIMARY).grid(row=1, column=1, sticky="w", padx=10, pady=2)

        ctk.CTkLabel(detail_frame, text="Categoria:", font=("Consolas", 11),
                     text_color=COLOR_TEXT_SECONDARY).grid(row=2, column=0, sticky="w", padx=10, pady=2)
        self.dtc_cat_var = ctk.StringVar(value="---")
        ctk.CTkLabel(detail_frame, textvariable=self.dtc_cat_var, font=("Consolas", 12),
                     text_color=COLOR_TEXT_SECONDARY).grid(row=2, column=1, sticky="w", padx=10, pady=2)

        # Lista de DTCs
        self.dtc_listbox = ctk.CTkTextbox(view, font=("Consolas", 12),
                                          fg_color=COLOR_BG_CARD)
        self.dtc_listbox.grid(row=2, column=0, sticky="nsew", padx=0, pady=0)
        self.dtc_listbox.insert("0.0", "Los DTCs apareceran aqui despues del escaneo...\n")
        self.dtc_listbox.configure(state="disabled")

    # ==========================================================================
    # VISTA: TOPOLOGIA DE RED
    # ==========================================================================

    def _build_topology_view(self):
        view = self.views["topology"]
        view.grid_columnconfigure(0, weight=1)
        view.grid_rowconfigure(0, weight=0)
        view.grid_rowconfigure(1, weight=0)
        view.grid_rowconfigure(2, weight=1)

        # Barra de acciones
        action_bar = ctk.CTkFrame(view, fg_color=COLOR_BG_CARD)
        action_bar.grid(row=0, column=0, sticky="ew", padx=0, pady=(0, 10))

        ctk.CTkButton(action_bar, text="\u2630  Escanear Red",
                      command=self._start_topology_scan, width=160,
                      fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER,
                      font=("Consolas", 12, "bold")).pack(side="left", padx=10, pady=10)

        self.topology_scan_btn = ctk.CTkButton(action_bar, text="Detener",
                                              command=self._stop_topology_scan, width=100,
                                              fg_color="#555", hover_color="#444",
                                              font=("Consolas", 11))
        self.topology_scan_btn.pack(side="left", padx=5, pady=10)

        self.topology_summary = ctk.CTkLabel(action_bar, text="No escaneado",
                                            font=("Consolas", 12, "bold"),
                                            text_color=COLOR_TEXT_SECONDARY)
        self.topology_summary.pack(side="right", padx=15, pady=10)

        # Grid de modulos
        self.topology_grid = ctk.CTkScrollableFrame(view, fg_color=COLOR_BG_DARK)
        self.topology_grid.grid(row=1, column=0, sticky="nsew", padx=0, pady=(0, 10))
        self.topology_grid.grid_columnconfigure((0, 1, 2, 3, 4), weight=1)

        # Detalles del modulo seleccionado
        detail_frame = ctk.CTkFrame(view, fg_color=COLOR_BG_CARD)
        detail_frame.grid(row=2, column=0, sticky="ew", padx=0, pady=0)
        detail_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(detail_frame, text="Modulo:", font=("Consolas", 11),
                    text_color=COLOR_TEXT_SECONDARY).grid(row=0, column=0, sticky="w", padx=10, pady=3)
        self.topo_mod_var = ctk.StringVar(value="---")
        ctk.CTkLabel(detail_frame, textvariable=self.topo_mod_var, font=("Consolas", 14, "bold"),
                    text_color=COLOR_TEXT_PRIMARY).grid(row=0, column=1, sticky="w", padx=10, pady=3)

        ctk.CTkLabel(detail_frame, text="Part Number:", font=("Consolas", 11),
                    text_color=COLOR_TEXT_SECONDARY).grid(row=1, column=0, sticky="w", padx=10, pady=3)
        self.topo_pn_var = ctk.StringVar(value="---")
        ctk.CTkLabel(detail_frame, textvariable=self.topo_pn_var, font=("Consolas", 11),
                    text_color=COLOR_TEXT_PRIMARY).grid(row=1, column=1, sticky="w", padx=10, pady=3)

        ctk.CTkLabel(detail_frame, text="Calibration:", font=("Consolas", 11),
                    text_color=COLOR_TEXT_SECONDARY).grid(row=2, column=0, sticky="w", padx=10, pady=3)
        self.topo_cal_var = ctk.StringVar(value="---")
        ctk.CTkLabel(detail_frame, textvariable=self.topo_cal_var, font=("Consolas", 11),
                    text_color=COLOR_TEXT_PRIMARY).grid(row=2, column=1, sticky="w", padx=10, pady=3)

        ctk.CTkLabel(detail_frame, text="Bus:", font=("Consolas", 11),
                    text_color=COLOR_TEXT_SECONDARY).grid(row=3, column=0, sticky="w", padx=10, pady=3)
        self.topo_bus_var = ctk.StringVar(value="---")
        ctk.CTkLabel(detail_frame, textvariable=self.topo_bus_var, font=("Consolas", 11),
                    text_color=COLOR_TEXT_PRIMARY).grid(row=3, column=1, sticky="w", padx=10, pady=3)

        # Widgets de modulo se rellenan en _refresh_topology_display
        self.topo_cards = {}

    # ==========================================================================
    # OPERACIONES DE TOPOLOGIA
    # ==========================================================================

    def _start_topology_scan(self):
        """Inicia el escaneo topologico en un hilo separado."""
        if not self.communicator.is_connected():
            self.console.append("ERROR: Conecte primero al vehiculo")
            return

        self.console.append("Iniciando escaneo topologico de red CAN...")
        self.topology_scan_btn.configure(text="Escaneando...", fg_color=COLOR_ORANGE)

        def task():
            try:
                self.engine.initialize()
                results = self.engine.scan_topology(force=True)
                self._module_status = {m.name: s for m, s in results.items()}
                self._topology_scanned = True
                self.app.after(0, self._refresh_topology_display)
                self.app.after(0, lambda: self.console.append(
                    f"Topologia: {self.engine.topology.summary}"))
            except Exception as e:
                self.app.after(0, lambda: self.console.append(f"Error escaneo: {e}"))
            self.app.after(0, lambda: self.topology_scan_btn.configure(
                text="Detener", fg_color="#555"))

        self._topo_thread = threading.Thread(target=task, daemon=True)
        self._topo_thread.start()

    def _stop_topology_scan(self):
        """Detiene el escaneo topologico en curso."""
        self.engine.topology.stop_scan()
        self.console.append("Escaneo topologico detenido por usuario")

    def _refresh_topology_display(self):
        """Actualiza la grid de modulos en la vista de topologia."""
        for widget in self.topology_grid.winfo_children():
            widget.destroy()

        self.topo_cards.clear()

        col = 0
        row = 0
        for module in SuzukiModule:
            status = self._module_status.get(module.name, ModuleStatus.UNKNOWN)
            present = (status == ModuleStatus.PRESENT)

            card = ctk.CTkFrame(self.topology_grid, fg_color=COLOR_BG_CARD,
                               border_width=2,
                               border_color=COLOR_GREEN if present else COLOR_BORDER)
            card.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")

            # Indicador de estado
            status_color = COLOR_GREEN if present else "#555"
            status_icon = "\u25cf" if present else "\u25cb"
            ctk.CTkLabel(card, text=status_icon, font=("Consolas", 18),
                        text_color=status_color).pack(anchor="ne", padx=8, pady=(5, 0))

            # Nombre del modulo
            ctk.CTkLabel(card, text=module.name, font=("Consolas", 14, "bold"),
                        text_color=COLOR_TEXT_PRIMARY if present else "#555").pack(pady=(5, 0))

            # Label del modulo
            ctk.CTkLabel(card, text=module.label, font=("Consolas", 10),
                        text_color=COLOR_TEXT_SECONDARY if present else "#444").pack()

            # Bus
            bus_color = COLOR_TEXT_SECONDARY
            ctk.CTkLabel(card, text=module.bus, font=("Consolas", 9),
                        text_color=bus_color).pack(pady=(0, 5))

            # Info extendida
            info = self.engine.get_module_info(module.name)
            if info and info.get("part_number"):
                ctk.CTkLabel(card, text=info["part_number"], font=("Consolas", 8),
                            text_color=COLOR_TEXT_SECONDARY).pack(pady=(0, 2))

            # Al hacer clic, mostrar detalle
            card.bind("<Button-1>", lambda e, m=module: self._show_module_detail(m))

            self.topo_cards[module.name] = card

            col += 1
            if col >= 5:
                col = 0
                row += 1

        # Actualizar resumen
        present = sum(1 for s in self._module_status.values() if s == ModuleStatus.PRESENT)
        total = len(SuzukiModule)
        self.topology_summary.configure(text=f"{present}/{total} modulos")

    def _show_module_detail(self, module: SuzukiModule):
        """Muestra detalle de un modulo en el panel inferior de topologia."""
        self.topo_mod_var.set(f"{module.name} ({module.label})")
        self.topo_bus_var.set(module.bus)

        info = self.engine.get_module_info(module.name)
        if info:
            self.topo_pn_var.set(info.get("part_number") or "No disponible")
            self.topo_cal_var.set(info.get("calibration") or "No disponible")
        else:
            self.topo_pn_var.set("No disponible")
            self.topo_cal_var.set("No disponible")

    # ==========================================================================
    # VISTA: MONITORS (READINESS)
    # ==========================================================================

    def _build_monitors_view(self):
        view = self.views["monitors"]
        view.grid_columnconfigure(0, weight=1)
        view.grid_rowconfigure(0, weight=0)
        view.grid_rowconfigure(1, weight=0)
        view.grid_rowconfigure(2, weight=1)

        # Boton de refresco
        ctk.CTkButton(view, text="\u21bb  Refrescar Monitores",
                      command=self._refresh_monitors, width=160,
                      fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER,
                      font=("Consolas", 12, "bold")).grid(row=0, column=0, sticky="w", pady=(0, 10))

        ctk.CTkLabel(view, text="Monitores de Emisiones (Readiness)",
                     font=("Consolas", 18, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).grid(row=1, column=0, sticky="w", pady=(0, 15))

        monitor_frame = ctk.CTkFrame(view, fg_color=COLOR_BG_CARD)
        monitor_frame.grid(row=2, column=0, sticky="nsew", padx=0, pady=0)
        monitor_frame.grid_columnconfigure(1, weight=1)
        monitor_frame.grid_columnconfigure(2, weight=0)

        # Encabezados
        ctk.CTkLabel(monitor_frame, text="Monitor", font=("Consolas", 12, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).grid(row=0, column=0, sticky="w", padx=15, pady=10)
        ctk.CTkLabel(monitor_frame, text="Estado", font=("Consolas", 12, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).grid(row=0, column=1, sticky="w", padx=15, pady=10)
        ctk.CTkLabel(monitor_frame, text="Requerido", font=("Consolas", 12, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).grid(row=0, column=2, sticky="w", padx=15, pady=10)

        self.monitor_widgets = {}
        monitors = [
            ("MIL", "Check Engine", True),
            ("Catalyst", "Catalizador", True),
            ("Heated Catalyst", "Catalizador Calefaccionado", False),
            ("Evap System", "Sistema EVAP", True),
            ("Secondary Air", "Sistema Aire Secundario", False),
            ("A/C Refrig.", "Refrigerante A/C", False),
            ("O2 Sensor", "Sensor Oxigeno", True),
            ("O2 Heater", "Calentador O2", True),
            ("EGR", "Recirculacion Gases", True),
        ]

        for i, (key, label, required) in enumerate(monitors):
            row = i + 1
            icon = ctk.CTkLabel(monitor_frame, text="\u25cf", font=("Consolas", 16),
                               text_color="#555")
            icon.grid(row=row, column=0, padx=(15, 10), pady=6, sticky="w")

            name = ctk.CTkLabel(monitor_frame, text=label, font=("Consolas", 12),
                               text_color=COLOR_TEXT_SECONDARY)
            name.grid(row=row, column=0, padx=(40, 10), pady=6, sticky="w")

            status = ctk.CTkLabel(monitor_frame, text="--", font=("Consolas", 12, "bold"),
                                 text_color="#555")
            status.grid(row=row, column=1, padx=15, pady=6, sticky="w")

            req_label = ctk.CTkLabel(monitor_frame, text="Si" if required else "No",
                                    font=("Consolas", 11), text_color=COLOR_TEXT_SECONDARY)
            req_label.grid(row=row, column=2, padx=15, pady=6, sticky="w")

            self.monitor_widgets[key] = {
                "icon": icon, "status": status, "required": required
            }

        # Sello ITV
        self.readiness_total = ctk.CTkLabel(monitor_frame, text="Estado ITV: 0/0 completados",
                                            font=("Consolas", 14, "bold"),
                                            text_color=COLOR_TEXT_SECONDARY)
        self.readiness_total.grid(row=len(monitors) + 1, column=0, columnspan=3,
                                  sticky="ew", padx=15, pady=15)

        # MIL status
        self.mil_label = ctk.CTkLabel(monitor_frame, text="MIL: OFF",
                                     font=("Consolas", 16, "bold"),
                                     text_color=COLOR_GREEN)
        self.mil_label.grid(row=len(monitors) + 2, column=0, columnspan=3,
                           sticky="ew", padx=15, pady=(0, 15))

    def _refresh_monitors(self):
        """Lee el estado MIL y readiness via OBD2 y actualiza la UI."""
        if not self.communicator.is_connected():
            self.console.append("ERROR: Conecte primero al vehiculo")
            return

        self.console.append("Leyendo estado de monitores de emisiones...")

        # PID 01 01: MIL status + DTC count
        value, error = self.pid_manager.query_pid("0101")
        if value is not None and isinstance(value, str) and "MIL" in value:
            mil_on = "True" in value or "True" in value
            dtc_count = 0
            import re
            m = re.search(r'DTCs:(\d+)', value)
            if m:
                dtc_count = int(m.group(1))
            self.mil_label.configure(
                text=f"MIL: {'ON' if mil_on else 'OFF'}  |  DTCs: {dtc_count}",
                text_color=COLOR_RED if mil_on else COLOR_GREEN)

            for key in self.monitor_widgets:
                if key == "MIL":
                    self.monitor_widgets[key]["icon"].configure(
                        text_color=COLOR_RED if mil_on else COLOR_GREEN)
                    self.monitor_widgets[key]["status"].configure(
                        text="ON" if mil_on else "OFF",
                        text_color=COLOR_RED if mil_on else COLOR_GREEN)

    # ==========================================================================
    # VISTA: SETTINGS
    # ==========================================================================

    def _build_settings_view(self):
        view = self.views["settings"]
        view.grid_columnconfigure(0, weight=1)
        view.grid_rowconfigure(0, weight=0)
        view.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(view, text="Configuracion", font=("Consolas", 18, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).grid(row=0, column=0, sticky="w", pady=(0, 15))

        scroll = ctk.CTkScrollableFrame(view, fg_color=COLOR_BG_DARK)
        scroll.grid(row=1, column=0, sticky="nsew")

        # --- Seccion: Conexion ---
        conn_section = ctk.CTkFrame(scroll, fg_color=COLOR_BG_CARD)
        conn_section.pack(fill="x", padx=0, pady=(0, 15))

        ctk.CTkLabel(conn_section, text="Conexion", font=("Consolas", 14, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).pack(anchor="w", padx=15, pady=(12, 5))

        # Puerto COM
        port_row = ctk.CTkFrame(conn_section, fg_color="transparent")
        port_row.pack(fill="x", padx=15, pady=5)
        ctk.CTkLabel(port_row, text="Puerto:", font=("Consolas", 12),
                     text_color=COLOR_TEXT_SECONDARY, width=100).pack(side="left")
        self.port_settings_var = ctk.StringVar(value="COM1")
        ctk.CTkOptionMenu(port_row, variable=self.port_settings_var, values=["COM1"],
                         width=150, fg_color=COLOR_BG_SIDEBAR,
                         button_color=SUZUKI_RED, button_hover_color=SUZUKI_RED_HOVER).pack(side="left", padx=5)

        # Baudrate
        baud_row = ctk.CTkFrame(conn_section, fg_color="transparent")
        baud_row.pack(fill="x", padx=15, pady=5)
        ctk.CTkLabel(baud_row, text="Baudrate:", font=("Consolas", 12),
                     text_color=COLOR_TEXT_SECONDARY, width=100).pack(side="left")
        ctk.CTkOptionMenu(baud_row, values=[str(b) for b in BAUD_RATES],
                         width=150, fg_color=COLOR_BG_SIDEBAR,
                         button_color=SUZUKI_RED, button_hover_color=SUZUKI_RED_HOVER,
                         variable=self.baud_var).pack(side="left", padx=5)

        # Timeout
        timeout_row = ctk.CTkFrame(conn_section, fg_color="transparent")
        timeout_row.pack(fill="x", padx=15, pady=5)
        ctk.CTkLabel(timeout_row, text="Timeout (s):", font=("Consolas", 12),
                     text_color=COLOR_TEXT_SECONDARY, width=100).pack(side="left")
        self.timeout_var = ctk.StringVar(value=str(SERIAL_TIMEOUT))
        ctk.CTkEntry(timeout_row, textvariable=self.timeout_var, width=100,
                    fg_color=COLOR_BG_SIDEBAR, font=("Consolas", 12)).pack(side="left", padx=5)

        # --- Seccion: Modulos ---
        mod_section = ctk.CTkFrame(scroll, fg_color=COLOR_BG_CARD)
        mod_section.pack(fill="x", padx=0, pady=(0, 15))

        ctk.CTkLabel(mod_section, text="Modulos de Diagnostico", font=("Consolas", 14, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).pack(anchor="w", padx=15, pady=(12, 5))

        mod_row = ctk.CTkFrame(mod_section, fg_color="transparent")
        mod_row.pack(fill="x", padx=15, pady=5)
        ctk.CTkLabel(mod_row, text="Modulo activo:", font=("Consolas", 12),
                     text_color=COLOR_TEXT_SECONDARY, width=100).pack(side="left")
        self.module_var = ctk.StringVar(value="ECM")
        self.module_menu = ctk.CTkOptionMenu(mod_row, variable=self.module_var,
                                            values=[m.name for m in SuzukiModule],
                                            width=150, fg_color=COLOR_BG_SIDEBAR,
                                            button_color=SUZUKI_RED,
                                            button_hover_color=SUZUKI_RED_HOVER)
        self.module_menu.pack(side="left", padx=5)
        ctk.CTkButton(mod_row, text="Aplicar", command=self._switch_module, width=80,
                     fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER,
                     font=("Consolas", 11)).pack(side="left", padx=5)

        # Info de modulos
        self.module_info = ctk.CTkTextbox(mod_section, height=100, font=("Consolas", 11),
                                          fg_color=COLOR_BG_SIDEBAR)
        self.module_info.pack(fill="x", padx=15, pady=(10, 15))
        for m in SuzukiModule:
            self.module_info.insert("end", f"{m.name:8s}  TX=0x{m.tx_id:03X}  RX=0x{m.rx_id:03X}\n")
        self.module_info.configure(state="disabled")

        # --- Seccion: Acerca de ---
        about_section = ctk.CTkFrame(scroll, fg_color=COLOR_BG_CARD)
        about_section.pack(fill="x", padx=0, pady=(0, 15))

        ctk.CTkLabel(about_section, text="Acerca de", font=("Consolas", 14, "bold"),
                     text_color=COLOR_TEXT_PRIMARY).pack(anchor="w", padx=15, pady=(12, 5))
        ctk.CTkLabel(about_section, text=f"{APP_NAME} v{APP_VERSION}",
                    font=("Consolas", 12), text_color=COLOR_TEXT_SECONDARY).pack(anchor="w", padx=15, pady=2)
        ctk.CTkLabel(about_section, text="Compatible con ELM327 via USB/Bluetooth",
                    font=("Consolas", 11), text_color=COLOR_TEXT_SECONDARY).pack(anchor="w", padx=15, pady=2)
        ctk.CTkLabel(about_section, text="Protocolos: CAN (ISO 15765-4), UDS (ISO 14229)",
                    font=("Consolas", 11), text_color=COLOR_TEXT_SECONDARY).pack(anchor="w", padx=15, pady=(2, 12))

    # ==========================================================================
    # BARRA DE ESTADO
    # ==========================================================================

    def _build_statusbar(self):
        """Barra de estado inferior persistente."""
        self.statusbar = ctk.CTkFrame(self.app, height=32, fg_color=COLOR_BG_SIDEBAR)
        self.statusbar.grid(row=2, column=0, columnspan=2, sticky="ew")
        self.statusbar.grid_propagate(False)
        self.statusbar.grid_columnconfigure(3, weight=1)

        # Conector: estado de conexion
        self.conn_icon = ctk.CTkLabel(self.statusbar, text="\u25cf Desconectado",
                                      font=("Consolas", 11, "bold"),
                                      text_color=COLOR_RED)
        self.conn_icon.grid(row=0, column=0, padx=(15, 10), pady=5, sticky="w")

        # Protocolo
        self.protocol_label = ctk.CTkLabel(self.statusbar, text="--",
                                          font=("Consolas", 10),
                                          text_color=COLOR_TEXT_SECONDARY)
        self.protocol_label.grid(row=0, column=1, padx=10, pady=5, sticky="w")

        # FPS
        self.fps_label = ctk.CTkLabel(self.statusbar, text="0 fps",
                                     font=("Consolas", 10),
                                     text_color=COLOR_TEXT_SECONDARY)
        self.fps_label.grid(row=0, column=2, padx=10, pady=5, sticky="w")

        # Nombre en esquina derecha
        ctk.CTkLabel(self.statusbar, text=f"{APP_NAME} v{APP_VERSION}",
                    font=("Consolas", 10), text_color=COLOR_TEXT_SECONDARY).grid(
                    row=0, column=4, padx=15, pady=5, sticky="e")

    def _update_status_bar(self):
        """Actualiza la barra de estado con informacion actual."""
        # Estado conexion
        if self.communicator.is_simulation():
            self.conn_icon.configure(text="\u25cf Simulacion", text_color=COLOR_YELLOW)
        elif self.communicator.state == ConnectionState.CONNECTED:
            self.conn_icon.configure(text="\u25cf Conectado", text_color=COLOR_GREEN)
        elif self.communicator.state == ConnectionState.RECOVERING:
            self.conn_icon.configure(text="\u25cf Recuperando", text_color=COLOR_ORANGE)
        elif self.communicator.state == ConnectionState.ERROR:
            self.conn_icon.configure(text="\u25cf Error", text_color=COLOR_RED)
        else:
            self.conn_icon.configure(text="\u25cf Desconectado", text_color=COLOR_RED)

        # Protocolo
        if self.communicator.current_protocol:
            self.protocol_label.configure(text=f"CAN {self.communicator.current_protocol}")
        else:
            self.protocol_label.configure(text="--")

        # FPS
        now = time.time()
        if now - self._fps_timer >= 1.0:
            self._current_fps = self._fps_counter
            self._fps_counter = 0
            self._fps_timer = now
        self.fps_label.configure(text=f"{self._current_fps} fps")
        self._fps_counter += 1

        self.app.after(1000, self._update_status_bar)

    # ==========================================================================
    # LOGGING
    # ==========================================================================

    def _on_log(self, message):
        self.app.after(0, lambda: self.console.append(message))

    # ==========================================================================
    # CONEXION / DESCONEXION
    # ==========================================================================

    def _toggle_connection(self):
        if self.communicator.is_connected():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if self.simulate_var.get():
            self.communicator.enable_simulation()
            self._on_connect_success()
            self.console.append("Modo simulacion activado")
            return

        self.config.port = self.port_var.get()
        self.config.baudrate = int(self.baud_var.get())
        self.console.append(f"Conectando a {self.config.port} @ {self.config.baudrate}...")

        if self.communicator.connect():
            self._on_connect_success()
        else:
            self.console.append("Error de conexion")

    def _on_connect_success(self):
        self.connect_btn.configure(text="Desconectar", fg_color="#555", hover_color="#444")
        self._refresh_ports()
        if self._current_nav == "dashboard":
            self._start_live_data()
        self.console.append("Conectado exitosamente")

    def _disconnect(self):
        self._stop_live_data()
        self.communicator.disconnect()
        self.connect_btn.configure(text="Conectar", fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER)
        self.console.append("Desconectado")

    # ==========================================================================
    # SCAN DE DATOS EN VIVO (MODULAR POR SECCIONES)
    # ==========================================================================

    def _toggle_scan(self):
        if self._live_data_running:
            self._stop_live_data()
            self.scan_btn.configure(text="\u25b6  Iniciar Scan",
                                    fg_color=SUZUKI_RED, hover_color=SUZUKI_RED_HOVER)
        else:
            self._start_live_data()

    def _start_live_data(self):
        if not self.communicator.is_connected():
            self.console.append("ERROR: Conecte primero al vehiculo")
            return
        self._live_data_running = True
        self.scan_btn.configure(text="\u25a0  Detener Scan", fg_color="#555", hover_color="#444")
        self.console.append("Iniciando scan de datos en vivo...")
        self._live_data_thread = threading.Thread(target=self._live_data_loop, daemon=True)
        self._live_data_thread.start()

    def _stop_live_data(self):
        self._live_data_running = False
        if self._live_data_thread and self._live_data_thread.is_alive():
            self._live_data_thread.join(timeout=2.0)
            self._live_data_thread = None
        self.dashboard.reset_all()

    def _live_data_loop(self):
        """Polling de datos en vivo sobre todos los PIDs configurados en secciones."""
        all_pids = self.dashboard.all_pids()
        if not all_pids:
            return

        while self._live_data_running and self.communicator.is_connected():
            for pid_key in all_pids:
                if not self._live_data_running:
                    break
                value, error = self.pid_manager.query_pid(pid_key)
                if value is not None:
                    self.app.after(0, lambda k=pid_key, v=value: self.dashboard.route_value(k, v))
                else:
                    self.app.after(0, lambda k=pid_key: self.dashboard.mark_no_signal(k))
                time.sleep(0.05)

            time.sleep(0.2)

    # ==========================================================================
    # DIAGNOSTICO (DTCs) MULTI-MODULO
    # ==========================================================================

    def _read_dtcs(self):
        if not self.communicator.is_connected():
            self.console.append("ERROR: Conecte primero al vehiculo")
            return

        # Si no se ha escaneado topologia, hacerlo primero
        if not self._topology_scanned:
            self.console.append("Ejecutando escaneo topologico previo...")
            def topology_first():
                self.engine.scan_topology()
                self._topology_scanned = True
                self.app.after(0, lambda: self._continue_dtc_read())
            threading.Thread(target=topology_first, daemon=True).start()
            return

        self._continue_dtc_read()

    def _continue_dtc_read(self):
        self.console.append("Escaneando DTCs en todos los modulos...")

        def task():
            try:
                result = self.engine.read_all_dtcs()
                self._dtc_records = result
                self.app.after(0, lambda: self._display_dtcs(result))
            except Exception as e:
                self.app.after(0, lambda: self.console.append(f"Error escaneo DTCs: {e}"))

        threading.Thread(target=task, daemon=True).start()

    def _display_dtcs(self, dtc_dict):
        self.dtc_listbox.configure(state="normal")
        self.dtc_listbox.delete("0.0", "end")

        # Aplanar para conteo total
        all_dtcs = []
        for module, dtcs in dtc_dict.items():
            all_dtcs.extend(dtcs)

        if not all_dtcs:
            self.dtc_listbox.insert("0.0", "No se encontraron codigos de falla en ningun modulo.\n")
            self.dtc_count_label.configure(text="DTCs: 0", text_color=COLOR_GREEN)
            self.dtc_code_var.set("---")
            self.dtc_desc_var.set("Sin codigos de falla")
            self.dtc_cat_var.set("---")
        else:
            total = len(all_dtcs)
            modules_str = " | ".join(f"{m}: {len(d)}" for m, d in dtc_dict.items() if d)
            color = COLOR_RED if total > 0 else COLOR_GREEN
            self.dtc_count_label.configure(text=f"DTCs: {total}", text_color=color)

            for module, dtcs in dtc_dict.items():
                if not dtcs:
                    continue

                # Encabezado de modulo
                self.dtc_listbox.insert("end", f"\n{'='*20} [{module}] {'='*20}\n")

                for dtc in dtcs:
                    status = ""
                    if dtc.is_pending:
                        status = " [PENDIENTE]"
                    if dtc.is_permanent:
                        status = " [PERMANENTE]"
                    line = f"  {dtc.code}{status}  {dtc.description}\n"
                    self.dtc_listbox.insert("end", line)

            # Mostrar el primer DTC como seleccionado
            first = all_dtcs[0]
            self.dtc_code_var.set(first.code)
            self.dtc_desc_var.set(first.description)
            self.dtc_cat_var.set(f"{first.module} | {first.category} | {first.prefix_type}")

        self.dtc_listbox.configure(state="disabled")
        self.console.append(f"Escaneo completado: {len(all_dtcs)} DTC(s) en {len(dtc_dict)} modulo(s)")

    def _confirm_clear_dtcs(self):
        """Dialogo de confirmacion antes de borrar DTCs de todos los modulos."""
        if not any(self._dtc_records.values()):
            self.console.append("No hay DTCs para borrar")
            return

        modules = [m for m, d in self._dtc_records.items() if d]
        dialog = ctk.CTkToplevel(self.app)
        dialog.title("Confirmar borrado")
        dialog.geometry("520x350")
        dialog.transient(self.app)
        dialog.grab_set()
        dialog.configure(fg_color=COLOR_BG_DARK)

        ctk.CTkLabel(dialog, text="\u26a0  Borrar codigos de falla",
                    font=("Consolas", 16, "bold"),
                    text_color=COLOR_RED).pack(pady=(20, 10))

        modules_str = ", ".join(modules)
        warning = (
            f"Se borraran DTCs de: {modules_str}\n"
            "\n"
            "Esto tambien eliminara:\n"
            "  \u2022  Freeze Frame (datos congelados de la falla)\n"
            "  \u2022  Monitores de emisiones (Readiness)\n"
            "  \u2022  El auto fallara la ITV/VTV hasta\n"
            "     que los monitores se completen\n"
            "\n"
            "Borrar NO repara el problema mecanico."
        )
        ctk.CTkLabel(dialog, text=warning, font=("Consolas", 11),
                    text_color=COLOR_TEXT_SECONDARY,
                    justify="left").pack(padx=20, pady=10)

        btn_frame = ctk.CTkFrame(dialog, fg_color="transparent")
        btn_frame.pack(pady=15)

        ctk.CTkButton(btn_frame, text="Cancelar",
                     command=dialog.destroy, width=120,
                     fg_color=COLOR_BG_CARD, hover_color=COLOR_BORDER,
                     font=("Consolas", 12)).pack(side="left", padx=10)

        ctk.CTkButton(btn_frame, text="Borrar DTCs",
                     command=lambda: self._execute_clear_dtcs(dialog), width=120,
                     fg_color=COLOR_RED, hover_color="#c0392b",
                     font=("Consolas", 12, "bold")).pack(side="left", padx=10)

    def _execute_clear_dtcs(self, dialog):
        dialog.destroy()

        def task():
            # Borrar DTCs modulo por modulo
            for module_name in list(self._dtc_records.keys()):
                module = SuzukiModule.by_tx(getattr(SuzukiModule[module_name], 'tx_id', 0x7E0))
                if module:
                    self.communicator.set_module(module)
                    resp = self.communicator.send_command("04", timeout=1.0)
                    time.sleep(0.1)

            self._dtc_records = {}
            self.app.after(0, lambda: self._display_dtcs({}))
            self.app.after(0, lambda: self.console.append("DTCs borrados de todos los modulos"))

        threading.Thread(target=task, daemon=True).start()

    def _export_dtcs(self):
        all_dtcs = []
        for module, dtcs in self._dtc_records.items():
            all_dtcs.extend(dtcs)

        if not all_dtcs:
            self.console.append("No hay DTCs para exportar")
            return

        filename = f"DTCs_Suzuki_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        filepath = os.path.join(os.path.expanduser("~"), "Desktop", filename)

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(f"Reporte DTC - {APP_NAME}\n")
                f.write(f"Fecha: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("=" * 60 + "\n\n")

                for module, dtcs in self._dtc_records.items():
                    if not dtcs:
                        continue
                    f.write(f"[{module}]\n")
                    f.write("-" * 40 + "\n")
                    for dtc in dtcs:
                        f.write(f"{dtc.code} - {dtc.description}\n")
                        f.write(f"  Tipo: {dtc.prefix_type}")
                        if dtc.is_pending:
                            f.write(" | PENDIENTE")
                        if dtc.is_permanent:
                            f.write(" | PERMANENTE")
                        f.write("\n\n")

            self.console.append(f"Reporte exportado: Desktop\\{filename}")
        except Exception as e:
            self.console.append(f"Error exportando: {e}")

    # ==========================================================================
    # UTILIDADES
    # ==========================================================================

    def _refresh_ports(self):
        ports = self.communicator.list_ports()
        if ports:
            names = [p.split(" - ")[0] for p in ports]
            self.port_menu.configure(values=names)
            self.port_var.set(names[0])
            self.port_settings_var.set(names[0])
            self.console.append(f"Puertos: {len(ports)} detectados")
        else:
            self.console.append("No se detectaron puertos COM")

    def _switch_module(self):
        mname = self.module_var.get()
        for m in SuzukiModule:
            if m.name == mname:
                self.communicator.set_module(m)
                self.module_label.configure(text=f"Modulo: {mname}")
                self.console.append(f"Modulo cambiado a {mname}")
                break

    def run(self):
        self.app.mainloop()

# ==============================================================================
# PUNTO DE ENTRADA
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="Suzuki OBD2 Diagnostic Scanner")
    parser.add_argument("--simulate", action="store_true", help="Activar modo simulacion")
    args = parser.parse_args()

    app = Application()
    if args.simulate:
        app.simulate_var.set(True)
        app._connect()
    app.run()

if __name__ == "__main__":
    main()
