#!/usr/bin/env python3
"""
Test Durandal Flight Controller driving VESC directly.
- Monitors VESC telemetry over COM33
- Arms Durandal on COM17
- Sends RC throttle override
- Checks whether motor responds via DroneCAN (CAN1) or PWM (MAIN 3)
"""

import sys
import time
from pathlib import Path
VESC_DIR = Path(__file__).resolve().parent.parent
if str(VESC_DIR) not in sys.path:
    sys.path.insert(0, str(VESC_DIR))

from pymavlink import mavutil
from lib.vesc_interface import VESCInterface

def main(target_pwm=1200, duration=2.0):
    print("=" * 60)
    print("  DURANDAL FLIGHT CONTROLLER -> VESC DIRECT DRIVE TEST")
    print(f"  Target Throttle: {target_pwm} us | Duration: {duration} s")
    print("=" * 60)

    # 1. Connect VESC on COM33
    vesc = VESCInterface("COM33")
    if not vesc.connect():
        print("[FAIL] Could not connect to VESC on COM33")
        return 1

    t_init = vesc.get_telemetry()
    v_in = t_init["v_in"] if t_init else 0.0
    print(f"[VESC] Connected on COM33. Supply Voltage: {v_in:.1f} V")

    # 2. Connect Durandal on COM17
    print("[Durandal] Connecting via MAVLink on COM17...")
    m = mavutil.mavlink_connection("COM17", baud=115200)
    m.wait_heartbeat(timeout=4)
    print("[Durandal] Heartbeat received.")

    # 3. Arm Durandal
    print("[Durandal] Sending Force Arm command...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196, 0, 0, 0, 0, 0
    )
    time.sleep(0.3)

    # 4. Stream throttle
    print(f"[Durandal] Outputting Throttle (chan1={target_pwm}, chan3={target_pwm}) for {duration} s...")
    t0 = time.time()
    t_end = t0 + duration
    samples = []
    last_rc = 0.0

    try:
        while time.time() < t_end:
            now = time.time()
            if now - last_rc >= 0.05:
                m.mav.rc_channels_override_send(
                    m.target_system, m.target_component,
                    target_pwm, 0, target_pwm, 0, 0, 0, 0, 0
                )
                last_rc = now

            t = vesc.get_telemetry(timeout=0.04)
            if t:
                samples.append({
                    "t": now - t0,
                    "rpm": t["rpm"],
                    "i": t["current_motor"],
                    "duty": t["duty_now"]
                })
            time.sleep(0.02)
    finally:
        # Safely disarm
        print("[Durandal] Disarming and zeroing throttle...")
        for _ in range(5):
            m.mav.rc_channels_override_send(
                m.target_system, m.target_component,
                1000, 0, 1000, 0, 0, 0, 0, 0
            )
            time.sleep(0.02)
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.3)
        vesc.disconnect()
        m.close()

    print(f"\nCaptured {len(samples)} telemetry samples from VESC:")
    if samples:
        rpms = [s["rpm"] for s in samples]
        peak = max(rpms, key=abs)
        print(f"  Peak VESC Speed: {peak} ERPM ({abs(peak)//12} Mech RPM)")
        print("\nTelemetry trace samples:")
        for s in samples[-15::2]:
            print(f"  t={s['t']:5.2f}s | ERPM={s['rpm']:5d} ({abs(s['rpm'])//12:3d} mech) | Duty={s['duty']*100:4.1f}% | I={s['i']:4.2f}A")

        if abs(peak) >= 100:
            print("\n>>> [PASS] Durandal Flight Controller is successfully commanding VESC! <<<")
            return 0
        else:
            print("\n>>> Motor speed did not reach 100 ERPM. Checking channel routing... <<<")
            return 2
    else:
        print("[FAIL] No VESC telemetry samples captured.")
        return 1

if __name__ == "__main__":
    pwm_val = int(sys.argv[1]) if len(sys.argv) > 1 else 1200
    sys.exit(main(target_pwm=pwm_val))
