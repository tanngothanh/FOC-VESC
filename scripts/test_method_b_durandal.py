#!/usr/bin/env python3
"""
Method B Verification: DroneCAN ESC control from Durandal Flight Controller
Tests:
- Arming Durandal over MAVLink (COM17)
- Sending Throttle RC Override
- Capturing Durandal's uavcan.equipment.esc.RawCommand on CAN (COM16)
- Verifying VESC motor rotation and telemetry feedback
"""

import sys
import time
import dronecan
from pymavlink import mavutil

def test_method_b(fc_port="COM17", slcan_port="slcan:COM16", throttle_pwm=1200, duration=1.8):
    print("=" * 60)
    print("  METHOD B: DroneCAN Control via Durandal Flight Controller")
    print(f"  MAVLink Port: {fc_port} | SLCAN Monitor: {slcan_port}")
    print(f"  Target Throttle PWM: {throttle_pwm} us | Duration: {duration} s")
    print("=" * 60)

    # 1. Connect MAVLink
    print(f"[1/4] Connecting to Durandal on {fc_port}...")
    m = mavutil.mavlink_connection(fc_port, baud=115200)
    m.wait_heartbeat(timeout=4)
    print("[MAVLink] Heartbeat received from Durandal.")

    # 2. Connect DroneCAN SLCAN listener
    print(f"[2/4] Connecting DroneCAN monitor on {slcan_port}...")
    node = dronecan.make_node(slcan_port, node_id=126, bitrate=1000000, baudrate=115200)

    fc_raw_commands = []
    vesc_statuses = []

    def on_raw_command(event):
        if event.transfer.source_node_id == 10:  # Durandal node ID
            fc_raw_commands.append({
                "time": time.time(),
                "cmd": list(event.message.cmd)
            })

    def on_vesc_status(event):
        if event.transfer.source_node_id == 103:  # VESC node ID
            msg = event.message
            vesc_statuses.append({
                "time": time.time(),
                "rpm": msg.rpm,
                "voltage": msg.voltage,
                "current": msg.current,
                "temperature": msg.temperature
            })

    node.add_handler(dronecan.uavcan.equipment.esc.RawCommand, on_raw_command)
    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_vesc_status)

    def spin_safe(t=0.005):
        try:
            node.spin(t)
        except Exception:
            pass

    # Clear stale frames
    for _ in range(10):
        spin_safe(0.01)

    # 3. Arm Durandal
    print("[3/4] Arming Durandal FC...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196, 0, 0, 0, 0, 0
    )
    time.sleep(0.3)

    # 4. Command Throttle
    print(f"[4/4] Sending Throttle {throttle_pwm} us for {duration} s...")
    t0 = time.time()
    t_end = t0 + duration
    last_rc = 0.0

    try:
        while time.time() < t_end:
            now = time.time()
            if now - last_rc >= 0.05:  # 20 Hz RC Override
                m.mav.rc_channels_override_send(
                    m.target_system, m.target_component,
                    throttle_pwm, 0, throttle_pwm, 0, 0, 0, 0, 0
                )
                last_rc = now
            spin_safe(0.005)
    finally:
        # Always safely disarm
        print("Sending Throttle 1000 us and Disarming...")
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
        m.close()
        node.close()

    print("\n" + "=" * 60)
    print("  METHOD B TEST RESULTS")
    print("=" * 60)
    print(f"Durandal RawCommand frames broadcasted on CAN: {len(fc_raw_commands)}")
    if fc_raw_commands:
        print(f"  First command: {fc_raw_commands[0]['cmd']}")
        max_cmds = [c['cmd'][0] for c in fc_raw_commands if c['cmd']]
        if max_cmds:
            print(f"  Peak command:  {max(max_cmds)}")

    print(f"VESC Status telemetry frames received: {len(vesc_statuses)}")
    if vesc_statuses:
        rpms = [s["rpm"] for s in vesc_statuses]
        peak_rpm = max(rpms, key=abs)
        print(f"  Peak Motor Speed: {peak_rpm} ERPM ({abs(peak_rpm)//12} Mech RPM)")
        print("\nTelemetry trace samples:")
        for s in vesc_statuses[-15::3]:
            dt = s["time"] - t0
            print(f"  t={dt:5.2f}s | ERPM={s['rpm']:5d} ({abs(s['rpm'])//12:3d} mech) | V={s['voltage']:.1f}V | I={s['current']:.2f}A")

        if len(fc_raw_commands) > 0 and abs(peak_rpm) >= 100:
            print("\n>>> [PASS] METHOD B VERIFIED: Durandal successfully drives VESC via DroneCAN! <<<")
            return True
        else:
            print("\n>>> [CHECK] Verification criteria not fully met. <<<")
            return False
    else:
        print("[FAIL] No VESC telemetry frames received.")
        return False

if __name__ == "__main__":
    pwm = int(sys.argv[1]) if len(sys.argv) > 1 else 1200
    res = test_method_b(throttle_pwm=pwm)
    sys.exit(0 if res else 1)
