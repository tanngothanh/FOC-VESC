#!/usr/bin/env python3
"""
Tool: Switch VESC UAVCAN Raw Mode between Duty Cycle (2) and RPM (3)
Usage:
    python set_uavcan_duty_mode.py --port COM33 --mode duty
    python set_uavcan_duty_mode.py --port COM33 --mode rpm
"""

import sys
import time
import struct
import argparse
import serial

def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = ((crc << 8) | (crc >> 8)) ^ byte
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
        crc &= 0xFFFF
    return crc

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
    time.sleep(0.15)
    resp = ser.read(2048)
    
    i = 0
    while i < len(resp):
        if resp[i] == 3: # Long frame
            plen = struct.unpack('>H', resp[i+1:i+3])[0]
            pdata = resp[i+3:i+3+plen]
            if pdata[0] == 17:
                return pdata[1:]
            i += 3 + plen + 3
        elif resp[i] == 2:
            plen = resp[i+1]
            pdata = resp[i+2:i+2+plen]
            if pdata[0] == 17:
                return pdata[1:]
            i += 2 + plen + 3
        else:
            i += 1
    return b""

def set_appconf(ser: serial.Serial, appconf: bytes) -> bool:
    payload = bytes([16]) + appconf # COMM_SET_APPCONF
    pkt = make_packet(payload)
    ser.write(pkt)
    time.sleep(0.2)
    return True

def main():
    parser = argparse.ArgumentParser(description="Configure VESC UAVCAN Throttle Mode")
    parser.add_argument("--port", default="COM33", help="VESC USB COM port (default: COM33)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--mode", choices=["duty", "rpm", "current"], default="duty",
                        help="Throttle mode: duty (2), rpm (3), current (0)")
    parser.add_argument("--read-only", action="store_true", help="Only read current mode without changing")
    args = parser.parse_args()

    mode_map = {
        "current": 0,
        "current_no_rev": 1,
        "duty": 2,
        "rpm": 3
    }
    mode_name_map = {v: k for k, v in mode_map.items()}

    print(f"Connecting to VESC on {args.port}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except Exception as e:
        print(f"Error opening {args.port}: {e}")
        return 1

    try:
        appconf = bytearray(get_appconf(ser))
        if not appconf or len(appconf) < 30:
            print("Failed to read APPCONF from VESC.")
            return 1

        curr_can_mode = appconf[23]
        curr_esc_idx = appconf[24]
        curr_raw_mode = appconf[25]
        rpm_max = struct.unpack('>f', appconf[26:30])[0]

        print("Current VESC UAVCAN Configuration:")
        print(f"  - CAN Mode: {curr_can_mode} (4 = VESC_UAVCAN)")
        print(f"  - ESC Index: {curr_esc_idx} (Slider 1 in DroneCAN GUI = Index 0)")
        print(f"  - Raw Mode: {curr_raw_mode} ({mode_name_map.get(curr_raw_mode, 'unknown')})")
        print(f"  - Raw RPM Max: {rpm_max:.1f} ERPM")

        if args.read_only:
            return 0

        target_mode = mode_map[args.mode]
        if curr_raw_mode == target_mode:
            print(f"\nVESC is already in mode '{args.mode}' (value {target_mode}). No change needed.")
            return 0

        print(f"\nUpdating UAVCAN Raw Mode from {curr_raw_mode} ({mode_name_map.get(curr_raw_mode, 'unknown')}) to {target_mode} ({args.mode})...")
        appconf[25] = target_mode
        set_appconf(ser, bytes(appconf))
        time.sleep(0.3)

        # Verify
        new_app = get_appconf(ser)
        if new_app and len(new_app) >= 26:
            new_mode = new_app[25]
            if new_mode == target_mode:
                print(f"SUCCESS: UAVCAN Raw Mode successfully set to {new_mode} ({args.mode})!")
                if target_mode == 2:
                    print("--> Duty Cycle Control is now active. You can now use DroneCAN GUI ESC Management to smoothly control motor duty 0-100%!")
            else:
                print(f"FAILED: Verification returned mode {new_mode}, expected {target_mode}")
        else:
            print("WARNING: Could not re-read APPCONF to verify.")

    finally:
        ser.close()

    return 0

if __name__ == "__main__":
    sys.exit(main())
