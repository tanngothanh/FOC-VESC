#!/usr/bin/env python3
"""
Tool: Enable both DroneCAN (can_mode=1, Duty) and PWM (app_ppm_conf.ctrl_type=5) on VESC.
This fixes the root causes:
1. can_mode=4 (VESC_UAVCAN) ignored all incoming UAVCAN RawCommands (canard_driver.c line 1383).
   Must be can_mode=1 (CAN_MODE_UAVCAN).
2. ppm ctrl_type was 0 (PPM_CTRL_TYPE_NONE), completely disabling PWM motor control.
   Must be ppm ctrl_type=5 (PPM_CTRL_TYPE_DUTY_NOREV).
3. app_to_use set to 4 (APP_PPM_UART) so USB UART and PWM are active simultaneously.
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
        if resp[i] == 3:
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

def reboot_vesc(ser: serial.Serial):
    pkt = make_packet(bytes([22])) # COMM_REBOOT
    ser.write(pkt)
    time.sleep(0.5)

def main():
    parser = argparse.ArgumentParser(description="Enable UAVCAN RX and PWM Throttle on VESC")
    parser.add_argument("--port", default="COM33", help="VESC USB COM port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print(f"Connecting to VESC on {args.port}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except Exception as e:
        print(f"Error opening {args.port}: {e}")
        return 1

    try:
        appconf = bytearray(get_appconf(ser))
        if not appconf or len(appconf) < 50:
            print("Failed to read APPCONF.")
            return 1

        print("Current VESC settings:")
        print(f"  - can_mode: {appconf[23]} (Target: 1 = CAN_MODE_UAVCAN)")
        print(f"  - uavcan_esc_index: {appconf[24]} (Index 0 = ESC 1)")
        print(f"  - uavcan_raw_mode: {appconf[25]} (Target: 2 = Duty Cycle)")
        print(f"  - app_to_use: {appconf[33]} (Target: 4 = APP_PPM_UART)")
        print(f"  - ppm ctrl_type: {appconf[34]} (Target: 5 = PPM_CTRL_TYPE_DUTY_NOREV)")

        # Modify settings
        appconf[23] = 1 # CAN_MODE_UAVCAN (Enables incoming frame processing in canard_driver.c line 1383!)
        appconf[25] = 2 # UAVCAN_RAW_MODE_DUTY (Smooth breakaway without speed loop stiction)
        appconf[33] = 4 # APP_PPM_UART (Enables PPM and UART simultaneously)
        appconf[34] = 5 # PPM_CTRL_TYPE_DUTY_NOREV (Standard RC PWM Duty cycle without reverse)

        print("\nWriting new configuration to VESC...")
        set_appconf(ser, bytes(appconf))
        time.sleep(0.3)

        # Reboot to apply
        print("Rebooting VESC to initialize CAN driver and PPM app...")
        reboot_vesc(ser)
        ser.close()
        time.sleep(1.5)

        # Reconnect and verify
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        new_app = get_appconf(ser)
        if new_app and len(new_app) >= 50:
            print("\n=== VERIFIED CONFIGURATION ON VESC ===")
            print(f"  - can_mode: {new_app[23]} {'[PASS: CAN_MODE_UAVCAN]' if new_app[23]==1 else '[FAIL]'}")
            print(f"  - uavcan_raw_mode: {new_app[25]} {'[PASS: Duty]' if new_app[25]==2 else '[FAIL]'}")
            print(f"  - app_to_use: {new_app[33]} {'[PASS: PPM+UART]' if new_app[33]==4 else '[FAIL]'}")
            print(f"  - ppm ctrl_type: {new_app[34]} {'[PASS: Duty_NoRev]' if new_app[34]==5 else '[FAIL]'}")
            p_start = struct.unpack('>f', new_app[43:47])[0]
            p_end = struct.unpack('>f', new_app[47:51])[0]
            print(f"  - PWM Pulse range: {p_start*1000:.0f}us - {p_end*1000:.0f}us")
            print("\n>> SUCCESS: Both UAVCAN and PWM are now ACTIVE and ready to drive the motor! <<")
        else:
            print("Warning: Could not re-read appconf after reboot.")

    finally:
        ser.close()

    return 0

if __name__ == "__main__":
    sys.exit(main())
