#!/usr/bin/env python3
"""
VESC Real-Time Diagnostic & Telemetry Monitor
Subsystem: PROP-SUB-05 (Propulsion System)
Aircraft: CT-2W1 eVTOL Scale 1:6 Prototype
"""

import sys
import time
import struct
import argparse
import serial
from pathlib import Path

VESC_DIR = Path(__file__).resolve().parent.parent
if str(VESC_DIR) not in sys.path:
    sys.path.insert(0, str(VESC_DIR))

from lib.vesc_protocol import (
    crc16,
    COMM_GET_VALUES,
    FAULT_CODES,
    decode_telemetry
)

POLE_PAIRS = 12

def make_packet(payload: bytes) -> bytes:
    crc = crc16(payload)
    if len(payload) <= 255:
        return bytes([2, len(payload)]) + payload + struct.pack('>H', crc) + bytes([3])
    else:
        return bytes([3]) + struct.pack('>H', len(payload)) + payload + struct.pack('>H', crc) + bytes([3])

def get_appconf(ser: serial.Serial) -> bytes:
    ser.reset_input_buffer()
    pkt = make_packet(bytes([17])) # COMM_GET_APPCONF
    ser.write(pkt)
    time.sleep(0.12)
    resp = ser.read(2048)
    i = 0
    while i < len(resp):
        if resp[i] == 3 and i + 3 <= len(resp):
            plen = struct.unpack('>H', resp[i+1:i+3])[0]
            pdata = resp[i+3:i+3+plen]
            if len(pdata) > 0 and pdata[0] == 17:
                return pdata[1:]
            i += 3 + plen + 3
        elif resp[i] == 2 and i + 2 <= len(resp):
            plen = resp[i+1]
            pdata = resp[i+2:i+2+plen]
            if len(pdata) > 0 and pdata[0] == 17:
                return pdata[1:]
            i += 2 + plen + 3
        else:
            i += 1
    return b""

def query_telemetry(ser: serial.Serial) -> dict:
    ser.reset_input_buffer()
    pkt = make_packet(bytes([COMM_GET_VALUES]))
    ser.write(pkt)
    time.sleep(0.02)
    resp = ser.read(256)
    
    i = 0
    while i < len(resp):
        if resp[i] == 3 and i + 3 <= len(resp):
            plen = struct.unpack('>H', resp[i+1:i+3])[0]
            pdata = resp[i+3:i+3+plen]
            if len(pdata) > 0 and pdata[0] == COMM_GET_VALUES:
                return decode_telemetry(pdata)
            i += 3 + plen + 3
        elif resp[i] == 2 and i + 2 <= len(resp):
            plen = resp[i+1]
            pdata = resp[i+2:i+2+plen]
            if len(pdata) > 0 and pdata[0] == COMM_GET_VALUES:
                return decode_telemetry(pdata)
            i += 2 + plen + 3
        else:
            i += 1
    return None

def monitor_live(port: str = "COM33", duration: float = 60.0, rate_hz: float = 20.0):
    print("=" * 75)
    print(f"  VESC REAL-TIME TELEMETRY MONITOR ON {port}")
    print(f"  Sampling Rate: {rate_hz:.0f} Hz | Duration: {duration:.0f} s")
    print("=" * 75)

    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Could not open {port}: {e}")
        print("Please check if the VESC micro-USB cable is securely plugged into your PC.")
        return 1

    try:
        appconf = bytearray(get_appconf(ser))
        if appconf and len(appconf) >= 35:
            can_mode = appconf[23]
            esc_idx = appconf[24]
            raw_mode = appconf[25]
            raw_mode_names = {0: "CURRENT", 1: "CURRENT_NO_REV", 2: "DUTY_CYCLE", 3: "RPM"}
            app_to_use = appconf[33]
            ppm_ctrl = appconf[34]
            ppm_names = {0: "OFF", 1: "CURRENT", 2: "CURRENT_NO_REV", 3: "DUTY", 4: "DUTY_NO_REV", 5: "DUTY_NO_REV_BRAKE"}
            print(f"[CONFIG] CAN Mode: {can_mode} | ESC Index: {esc_idx} | UAVCAN Mode: {raw_mode_names.get(raw_mode, str(raw_mode))}")
            print(f"[CONFIG] App to Use: {app_to_use} (4=PPM+UART) | PPM Control Type: {ppm_names.get(ppm_ctrl, str(ppm_ctrl))}")
        else:
            print("[WARN] Could not read APPCONF on startup.")

        print("\nStreaming live telemetry. Move DroneCAN GUI slider or PPM stick now...")
        print("-" * 75)
        print(f"{'Time(s)':<8} {'ERPM':<8} {'Mech RPM':<10} {'Duty (%)':<10} {'I_mot (A)':<10} {'V_bat (V)':<10} {'Fault Code':<15}")
        print("-" * 75)

        t0 = time.time()
        interval = 1.0 / rate_hz

        while time.time() - t0 < duration:
            loop_start = time.time()
            telem = query_telemetry(ser)
            
            if telem:
                elapsed = time.time() - t0
                erpm = telem['rpm']
                mech_rpm = erpm // POLE_PAIRS
                duty = telem['duty_now'] * 100.0
                i_mot = telem['current_motor']
                v_in = telem['v_in']
                fault = telem['fault_str']
                
                print(f"{elapsed:<8.2f} {erpm:<8} {mech_rpm:<10} {duty:<10.1f} {i_mot:<10.2f} {v_in:<10.1f} {fault:<15}")
            
            rem = interval - (time.time() - loop_start)
            if rem > 0:
                time.sleep(rem)

    except KeyboardInterrupt:
        print("\n[STOP] Monitoring stopped by user.")
    finally:
        ser.close()
    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="VESC Live Diagnostics")
    parser.add_argument("--port", default="COM33", help="VESC USB COM port (default: COM33)")
    parser.add_argument("--duration", type=float, default=60.0, help="Monitoring duration in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="Telemetry rate in Hz (default: 20)")
    args = parser.parse_args()
    sys.exit(monitor_live(args.port, args.duration, args.rate))
