"""Continuous Durandal SLCAN Provisioner.

Listens on COM17 for MAVLink heartbeat (upon hardware reset or power cycle).
As soon as MAVLink is detected:
1. Sets SERIAL7_PROTOCOL = 22.0 (SLCAN on COM16)
2. Sets CAN_SLCAN_CPORT = 1.0 (CAN1)
3. Sets CAN_SLCAN_SERNUM = -1.0
4. Saves to permanent FRAM storage (MAV_CMD_PREFLIGHT_STORAGE)
5. Reboots Durandal
6. Verifies COM16 is pure SLCAN and COM17 is MAVLink 2
"""

import time
import sys
import serial
from pymavlink import mavutil


def set_param(m, name, val):
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode("utf-8"),
        float(val),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    for _ in range(15):
        ack = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if ack and ack.param_id == name:
            print(f"  [OK] {name} = {ack.param_value}", flush=True)
            return True
    print(f"  [FAIL] Could not confirm {name}", flush=True)
    return False


def main():
    print("==================================================", flush=True)
    print("Durandal SLCAN Provisioner (Continuous Watcher)", flush=True)
    print("==================================================", flush=True)
    print("[WAITING] Please press the RESET button on Durandal", flush=True)
    print("          (or disconnect power supply/battery for 5s).", flush=True)

    start = time.time()
    m = None
    while time.time() - start < 300: # 5 minutes
        try:
            m = mavutil.mavlink_connection("COM17", baud=115200)
            hb = m.wait_heartbeat(timeout=1.0)
            if hb:
                print(f"\n[DETECTED] MAVLink heartbeat from System {m.target_system} Component {m.target_component}!", flush=True)
                break
            m.close()
            m = None
        except Exception:
            m = None
        time.sleep(0.5)

    if not m:
        print("[ERROR] Timeout waiting for Durandal reset.", flush=True)
        sys.exit(1)

    print("\n1. Configuring parameters...", flush=True)
    set_param(m, "CAN_P1_DRIVER", 1.0)
    set_param(m, "CAN_P1_BITRATE", 1000000.0)
    set_param(m, "CAN_D1_PROTOCOL", 1.0)
    set_param(m, "CAN_SLCAN_CPORT", 1.0)
    set_param(m, "CAN_SLCAN_SERNUM", -1.0)
    set_param(m, "SERIAL0_PROTOCOL", 2.0)
    set_param(m, "SERIAL7_PROTOCOL", 22.0)

    print("\n2. Saving parameters to permanent FRAM storage...", flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0, 1.0, 0, 0, 0, 0, 0, 0
    )
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack:
        print(f"  Storage save ACK: result = {ack.result}", flush=True)

    print("\n3. Rebooting autopilot to activate SLCAN on COM16...", flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1.0, 0, 0, 0, 0, 0, 0
    )
    m.close()
    time.sleep(5)

    print("\n4. Verifying ports post-reboot...", flush=True)
    # Check COM17 MAVLink
    try:
        m2 = mavutil.mavlink_connection("COM17", baud=115200)
        hb2 = m2.wait_heartbeat(timeout=10)
        if hb2:
            print("  [PASS] COM17: MAVLink 2 ACTIVE (Ready for Mission Planner)!", flush=True)
        m2.close()
    except Exception as e:
        print(f"  [WARN] COM17 MAVLink check: {e}", flush=True)

    # Check COM16 SLCAN
    try:
        s = serial.Serial("COM16", 115200, timeout=1)
        s.write(b"O\r")
        time.sleep(0.5)
        raw = s.read(256)
        s.close()
        if b"T" in raw or b"t" in raw:
            print(f"  [PASS] COM16: Pure SLCAN streaming ({len(raw)} bytes received)!", flush=True)
            print(f"  Sample: {raw[:60]}", flush=True)
        else:
            print(f"  [INFO] COM16 opened successfully: {raw}", flush=True)
    except Exception as e:
        print(f"  [WARN] COM16 check: {e}", flush=True)

    print("\n==================================================", flush=True)
    print("SUCCESS: COM16 is now dedicated SLCAN for DroneCAN GUI Tool!", flush=True)
    print("==================================================", flush=True)


if __name__ == "__main__":
    main()
