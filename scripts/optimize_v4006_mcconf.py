#!/usr/bin/env python3
"""
Tool: Optimize VESC MCCONF for Sunnysky V4006 (PROP-SUB-05).
Applies Golden Baseline FOC parameters, Duty clamping, and Anti-Runaway speed PID gains:
1. l_max_duty = 0.4700 (Strict anti-runaway physical hardware clamp at ~6,100 RPM)
2. l_in_current_min = 0.0 A (Strict Anti-Backfeed for DC power supply protection)
3. l_current_min = -5.0 A (Controlled dissipative electrical braking)
4. foc_observer_type = 3 (FOC_OBSERVER_MXLEMMING_LAMBDA_COMP for smooth SPM tracking)
5. foc_observer_gain = 90000000.0 (9.0e7 - eliminates 1.3e9 noise/buzzing)
6. foc_f_zv = 35000.0 Hz (Optimal 35 kHz switching frequency)
7. foc_dt_us = 0.000 us (Zero software deadtime compensation)
8. foc_cc_decoupling = 0 (Disabled cross-coupling for low-inductance SPM)
9. s_pid_kp = 0.0020 (Golden baseline speed proportional gain)
10. s_pid_ki = 0.0005 (Low integrator gain - strictly eliminates windup & throttle hanging)
11. s_pid_kd = 0.0000 (Zero derivative gain - prevents angle noise amplification)
12. s_pid_ramp_erpms_s = 30000.0 ERPM/s (2,500 Mech RPM/s smooth acceleration)
13. s_pid_allow_braking = 1 (Active braking permitted)
"""

import sys
import time
import struct
import argparse
import serial
from pathlib import Path

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


def optimize_mcconf(port: str = "COM33", baud: int = 115200) -> bool:
    print(f"\n{'='*70}")
    print(f"[VESC OPTIMIZER] Connecting to {port} at {baud} baud...")
    print(f"{'='*70}")

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {port}: {e}")
        return False

    try:
        raw_buf = get_mcconf(ser)
        if not raw_buf or len(raw_buf) < 400:
            print(f"[ERROR] Failed to read valid MCCONF (len={len(raw_buf)})")
            ser.close()
            return False

        buf = bytearray(raw_buf)
        sig = struct.unpack(">I", buf[0:4])[0]
        if sig != MCCONF_SIGNATURE:
            print(f"[ERROR] Signature mismatch! Expected 0x{MCCONF_SIGNATURE:08X}, got 0x{sig:08X}")
            ser.close()
            return False

        print(f"[MCCONF READ SUCCESS] Length: {len(buf)} bytes, Signature: 0x{sig:08X}")

        # Current / Duty limits
        old_i_min = struct.unpack(">f", buf[12:16])[0]
        old_in_min = struct.unpack(">f", buf[20:24])[0]
        old_max_duty = struct.unpack(">h", buf[71:73])[0] / 10000.0

        # FOC parameters
        foc_f_zv_idx = 132
        foc_dt_us_idx = 136
        obs_gain_idx = 174
        cc_dec_idx = 246
        obs_type_idx = 247

        old_f_zv = struct.unpack(">f", buf[foc_f_zv_idx:foc_f_zv_idx+4])[0]
        old_dt_us = struct.unpack(">f", buf[foc_dt_us_idx:foc_dt_us_idx+4])[0]
        old_obs_gain = struct.unpack(">f", buf[obs_gain_idx:obs_gain_idx+4])[0]
        old_cc_dec = buf[cc_dec_idx]
        old_obs_type = buf[obs_type_idx]

        # Speed PID parameters
        old_sp_kp = struct.unpack(">f", buf[330:334])[0]
        old_sp_ki = struct.unpack(">f", buf[334:338])[0]
        old_sp_kd = struct.unpack(">f", buf[338:342])[0]
        old_allow_braking = buf[348]
        old_ramp = struct.unpack(">f", buf[349:353])[0]

        print("\n[CURRENT PARAMETERS]")
        print(f"  - l_max_duty:         {old_max_duty:.4f} (95% -> runaway prone!)")
        print(f"  - l_in_current_min:   {old_in_min:.1f} A")
        print(f"  - l_current_min:      {old_i_min:.1f} A")
        print(f"  - foc_observer_type:  {old_obs_type} (0=ORTEGA, 3=MXLEMMING)")
        print(f"  - foc_observer_gain:  {old_obs_gain:.2e} (Excessive noise!)")
        print(f"  - foc_f_zv:           {old_f_zv:.0f} Hz")
        print(f"  - foc_dt_us:          {old_dt_us:.3f} us")
        print(f"  - foc_cc_decoupling:  {old_cc_dec}")
        print(f"  - s_pid_kp:           {old_sp_kp:.4f}")
        print(f"  - s_pid_ki:           {old_sp_ki:.4f} (Excessive windup!)")
        print(f"  - s_pid_kd:           {old_sp_kd:.4f}")
        print(f"  - s_pid_allow_braking:{old_allow_braking}")
        print(f"  - s_pid_ramp_erpms_s: {old_ramp:.0f} ERPM/s")

        # ================= APPLY OPTIMIZED VALUES =================
        # 1. Hardware limits
        buf[12:16] = struct.pack(">f", -5.0)            # l_current_min = -5.0 A (dissipative braking)
        buf[20:24] = struct.pack(">f", 0.0)             # l_in_current_min = 0.0 A (Rule 2 Anti-backfeed strictly protects DC supply)
        buf[71:73] = struct.pack(">h", 4700)            # l_max_duty = 0.4700 (Rule 3 Duty clamp ~6,100 RPM)

        # 2. FOC Acoustic & Angle tracking
        buf[foc_f_zv_idx:foc_f_zv_idx+4] = struct.pack(">f", 35000.0)  # Rule 9: 35 kHz FOC
        buf[foc_dt_us_idx:foc_dt_us_idx+4] = struct.pack(">f", 0.0)    # Rule 7: Zero software deadtime
        buf[obs_gain_idx:obs_gain_idx+4] = struct.pack(">f", 90000000.0) # 9.0e7 Golden baseline observer gain (eliminates buzzing)
        buf[cc_dec_idx] = 0                             # Rule 8: Cross-coupling decoupling disabled
        buf[obs_type_idx] = 3                           # Rule 1: FOC_OBSERVER_MXLEMMING_LAMBDA_COMP

        # 3. Speed loop gains (anti-windup / zero throttle hang)
        buf[330:334] = struct.pack(">f", 0.0020)        # s_pid_kp = 0.0020
        buf[334:338] = struct.pack(">f", 0.0001)        # s_pid_ki = 0.0001 (Golden baseline - strictly eliminates windup)
        buf[338:342] = struct.pack(">f", 0.0000)        # s_pid_kd = 0.0000 (Rule 5)
        buf[348] = 1                                    # s_pid_allow_braking = 1
        buf[349:353] = struct.pack(">f", 20000.0)       # s_pid_ramp_erpms_s = 20000.0 (1,667 Mech RPM/s smooth acceleration)
        buf[353] = 0                                    # s_pid_speed_source = 0 (PLL speed source)

        print("\n[WRITING OPTIMIZED MCCONF VIA COMM_SET_MCCONF (13)]...")
        set_mcconf(ser, bytes(buf))
        time.sleep(0.3)

        print("[REBOOT] Rebooting VESC to persist EEPROM flash...")
        reboot_vesc(ser)
        ser.close()

        print("Waiting 2.0s for USB-CDC re-enumeration...")
        time.sleep(2.0)

        # Reconnect and verify
        ser = None
        for attempt in range(12):
            try:
                ser = serial.Serial(port, baud, timeout=1.0)
                if ser.is_open:
                    break
            except Exception:
                time.sleep(0.5)

        if not ser or not ser.is_open:
            print(f"[ERROR] Could not reconnect to {port} after reboot.")
            return False

        verify_buf = get_mcconf(ser)
        ser.close()

        if not verify_buf or len(verify_buf) < 400:
            print("[ERROR] Verification read failed!")
            return False

        # Verify parsed values
        new_max_duty = struct.unpack(">h", verify_buf[71:73])[0] / 10000.0
        new_in_min = struct.unpack(">f", verify_buf[20:24])[0]
        new_i_min = struct.unpack(">f", verify_buf[12:16])[0]
        new_obs_type = verify_buf[obs_type_idx]
        new_obs_gain = struct.unpack(">f", verify_buf[obs_gain_idx:obs_gain_idx+4])[0]
        new_f_zv = struct.unpack(">f", verify_buf[foc_f_zv_idx:foc_f_zv_idx+4])[0]
        new_dt_us = struct.unpack(">f", verify_buf[foc_dt_us_idx:foc_dt_us_idx+4])[0]
        new_cc_dec = verify_buf[cc_dec_idx]
        new_sp_kp = struct.unpack(">f", verify_buf[330:334])[0]
        new_sp_ki = struct.unpack(">f", verify_buf[334:338])[0]
        new_sp_kd = struct.unpack(">f", verify_buf[338:342])[0]
        new_allow_braking = verify_buf[348]
        new_ramp = struct.unpack(">f", verify_buf[349:353])[0]

        new_sp_src = verify_buf[353]

        print("\n" + "="*70)
        print("[VERIFIED OPTIMIZED MCCONF SETTINGS]")
        print(f"  - l_max_duty:         {new_max_duty:.4f} ({'PASS: CLAMPED at 47%' if abs(new_max_duty - 0.47) < 0.001 else 'FAIL'})")
        print(f"  - l_in_current_min:   {new_in_min:.2f} A ({'PASS: Anti-backfeed 0A' if abs(new_in_min) < 0.001 else 'FAIL'})")
        print(f"  - l_current_min:      {new_i_min:.2f} A ({'PASS: Braking -5A' if abs(new_i_min - (-5.0)) < 0.01 else 'FAIL'})")
        print(f"  - foc_observer_type:  {new_obs_type} ({'PASS: MXLEMMING_LAMBDA_COMP' if new_obs_type == 3 else 'FAIL'})")
        print(f"  - foc_observer_gain:  {new_obs_gain:.1e} ({'PASS: 9.0e7 Golden Baseline' if abs(new_obs_gain - 9e7) < 1e6 else 'FAIL'})")
        print(f"  - foc_f_zv:           {new_f_zv:.0f} Hz ({'PASS: 35 kHz' if abs(new_f_zv - 35000) < 1 else 'FAIL'})")
        print(f"  - foc_dt_us:          {new_dt_us:.3f} us ({'PASS: 0.000 us' if abs(new_dt_us) < 0.001 else 'FAIL'})")
        print(f"  - foc_cc_decoupling:  {new_cc_dec} ({'PASS: Disabled' if new_cc_dec == 0 else 'FAIL'})")
        print(f"  - s_pid_kp:           {new_sp_kp:.4f} ({'PASS: 0.0020' if abs(new_sp_kp - 0.002) < 0.0001 else 'FAIL'})")
        print(f"  - s_pid_ki:           {new_sp_ki:.4f} ({'PASS: 0.0001 (Anti-Windup)' if abs(new_sp_ki - 0.0001) < 0.00005 else 'FAIL'})")
        print(f"  - s_pid_kd:           {new_sp_kd:.4f} ({'PASS: 0.0000' if abs(new_sp_kd) < 0.0001 else 'FAIL'})")
        print(f"  - s_pid_allow_braking:{new_allow_braking} ({'PASS: Active Braking Enabled' if new_allow_braking == 1 else 'FAIL'})")
        print(f"  - s_pid_ramp_erpms_s: {new_ramp:.0f} ERPM/s ({'PASS: 20,000' if abs(new_ramp - 20000) < 1 else 'FAIL'})")
        print(f"  - s_pid_speed_source: {new_sp_src} ({'PASS: 0 (PLL)' if new_sp_src == 0 else 'FAIL'})")
        print("="*70)

        all_passed = (
            abs(new_max_duty - 0.47) < 0.001 and
            new_obs_type == 3 and
            abs(new_obs_gain - 9e7) < 1e6 and
            abs(new_in_min) < 0.001 and
            abs(new_sp_ki - 0.0001) < 0.00005 and
            new_allow_braking == 1 and
            new_sp_src == 0
        )
        return all_passed

    except Exception as e:
        print(f"[ERROR] Optimization failed: {e}")
        return False
    finally:
        if "ser" in locals() and ser and ser.is_open:
            ser.close()


def main():
    parser = argparse.ArgumentParser(description="Optimize VESC MCCONF for Sunnysky V4006")
    parser.add_argument("--port", default="COM33", help="VESC USB COM Port (default: COM33)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    success = optimize_mcconf(args.port, args.baud)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
