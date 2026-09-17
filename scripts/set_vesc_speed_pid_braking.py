#!/usr/bin/env python3
"""
Tool: Set s_pid_allow_braking = 1 and s_pid_ramp_erpms_s = 60000.0 in VESC MCCONF.
This resolves the throttle hanging ("treo ga") issue where the speed controller
could not decelerate the motor when commanded throttle was reduced.
"""

import sys
import time
import struct
import argparse
import serial

MCCONF_SIGNATURE = 0x57AD8F53


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
        return bytes([2, len(payload)]) + payload + struct.pack(">H", crc) + bytes([3])
    else:
        return bytes([3]) + struct.pack(">H", len(payload)) + payload + struct.pack(">H", crc) + bytes([3])


def get_mcconf(ser: serial.Serial) -> bytes:
    ser.reset_input_buffer()
    pkt = make_packet(bytes([14]))  # COMM_GET_MCCONF
    ser.write(pkt)
    time.sleep(0.15)
    resp = ser.read(4096)

    i = 0
    while i < len(resp):
        if resp[i] == 3:
            if i + 3 > len(resp):
                break
            plen = struct.unpack(">H", resp[i+1:i+3])[0]
            if i + 3 + plen > len(resp):
                break
            pdata = resp[i+3:i+3+plen]
            if len(pdata) > 0 and pdata[0] == 14:
                return pdata[1:]
            i += 3 + plen + 3
        elif resp[i] == 2:
            if i + 2 > len(resp):
                break
            plen = resp[i+1]
            if i + 2 + plen > len(resp):
                break
            pdata = resp[i+2:i+2+plen]
            if len(pdata) > 0 and pdata[0] == 14:
                return pdata[1:]
            i += 2 + plen + 3
        else:
            i += 1
    return b""


def set_mcconf(ser: serial.Serial, mcconf: bytes) -> bool:
    payload = bytes([13]) + mcconf  # COMM_SET_MCCONF
    pkt = make_packet(payload)
    ser.write(pkt)
    time.sleep(0.3)
    return True


def reboot_vesc(ser: serial.Serial):
    pkt = make_packet(bytes([22]))  # COMM_REBOOT
    ser.write(pkt)
    time.sleep(0.1)
    pkt29 = make_packet(bytes([29]))  # Standard VESC COMM_REBOOT
    ser.write(pkt29)
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="Configure VESC Speed PID Allow Braking in MCCONF")
    parser.add_argument("--port", default="COM33", help="VESC USB COM port (default: COM33)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print(f"Connecting to VESC on {args.port}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {args.port}: {e}")
        return 1

    try:
        print("Reading MCCONF via COMM_GET_MCCONF (14)...")
        buf = bytearray(get_mcconf(ser))
        if not buf or len(buf) < 360:
            print(f"[ERROR] Failed to read valid MCCONF payload (got {len(buf)} bytes).")
            return 1

        # Parse signature
        sig = struct.unpack(">I", buf[0:4])[0]
        print(f"MCCONF Signature: 0x{sig:08X} (Payload len: {len(buf)} bytes)")
        if sig != MCCONF_SIGNATURE:
            print(f"[ERROR] Signature mismatch! Expected 0x{MCCONF_SIGNATURE:08X}, got 0x{sig:08X}")
            return 1

        curr_allow_braking = buf[348]
        curr_ramp = struct.unpack(">f", buf[349:353])[0]
        print(f"Current Settings:")
        print(f"  - s_pid_allow_braking: {curr_allow_braking}")
        print(f"  - s_pid_ramp_erpms_s:  {curr_ramp:.1f}")

        # Modify values
        buf[348] = 1
        buf[349:353] = struct.pack(">f", 60000.0)

        print(f"\nWriting updated MCCONF via COMM_SET_MCCONF (13)...")
        set_mcconf(ser, bytes(buf))
        time.sleep(0.3)

        print("Rebooting VESC via COMM_REBOOT (22)...")
        reboot_vesc(ser)
        ser.close()

        print("Waiting 1.5s for reboot and re-enumeration...")
        time.sleep(1.5)

        # Reconnect with retry loop to allow Windows USB re-enumeration
        print("Reconnecting to verify configuration...")
        ser = None
        for attempt in range(10):
            try:
                ser = serial.Serial(args.port, args.baud, timeout=1.0)
                if ser.is_open:
                    break
            except Exception:
                time.sleep(0.5)

        if not ser or not ser.is_open:
            print(f"[ERROR] Could not reconnect to {args.port} after reboot.")
            return 1
        verify_buf = get_mcconf(ser)
        if not verify_buf or len(verify_buf) < 360:
            print("[ERROR] Failed to read back MCCONF after reboot.")
            return 1

        verify_allow_braking = verify_buf[348]
        verify_ramp = struct.unpack(">f", verify_buf[349:353])[0]
        print(f"Verified Settings:")
        print(f"  - s_pid_allow_braking: {verify_allow_braking}")
        print(f"  - s_pid_ramp_erpms_s:  {verify_ramp:.1f}")

        if verify_allow_braking == 1:
            print("\n[SUCCESS] s_pid_allow_braking is now 1")
            return 0
        else:
            print(f"\n[FAIL] Verification failed: s_pid_allow_braking is {verify_allow_braking}")
            return 1

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
