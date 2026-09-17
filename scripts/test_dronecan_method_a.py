#!/usr/bin/env python3
"""
Method A Verification: DroneCAN RawCommand to VESC over SLCAN COM16
Tests bidirectional communication:
- Sends uavcan.equipment.esc.RawCommand(cmd=[...]) at 50Hz
- Listens to VESC (Node 103) uavcan.equipment.esc.Status telemetry
"""

import sys
import time
import dronecan

def test_uavcan_method_a(port="slcan:COM16", target_cmd=600, duration=1.5, esc_index=1):
    print(f"Connecting to DroneCAN on {port} (Target ESC Index: {esc_index})...")
    node = dronecan.make_node(port, node_id=127, bitrate=1000000, baudrate=115200)

    vesc_statuses = []

    def on_esc_status(event):
        if event.transfer.source_node_id == 103:
            msg = event.message
            vesc_statuses.append({
                "time": time.time(),
                "esc_index": msg.esc_index,
                "rpm": msg.rpm,
                "voltage": msg.voltage,
                "current": msg.current,
                "temperature": msg.temperature
            })

    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc_status)

    def spin_safe(timeout=0.005):
        try:
            node.spin(timeout)
        except dronecan.transport.TransferError:
            pass

    # Initial listener phase (300ms)
    t_end = time.time() + 0.3
    while time.time() < t_end:
        spin_safe(0.01)

    print(f"Captured {len(vesc_statuses)} Status frames before test.")
    if vesc_statuses:
        last = vesc_statuses[-1]
        print(f"Initial VESC Telemetry: RPM={last['rpm']}, V={last['voltage']:.1f}V, I={last['current']:.2f}A, Temp={last['temperature']:.1f}C")

    print(f"\n>>> SENDING DRONECAN RAWCOMMAND (cmd={target_cmd}, ~{target_cmd/81.92:.1f}% throttle, ESC {esc_index}) for {duration}s <<<")
    t0 = time.time()
    t_end = t0 + duration
    last_send = 0.0

    raw_array = [0, target_cmd] if esc_index == 1 else [target_cmd]
    stop_array = [0, 0] if esc_index == 1 else [0]

    while time.time() < t_end:
        now = time.time()
        if now - last_send >= 0.02: # 50 Hz
            cmd_msg = dronecan.uavcan.equipment.esc.RawCommand(cmd=raw_array)
            node.broadcast(cmd_msg)
            last_send = now
        spin_safe(0.005)

    # Stop command (send 0 for 300ms)
    print("Sending Stop command...")
    for _ in range(15):
        node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=stop_array))
        spin_safe(0.02)

    print(f"\nTotal VESC Telemetry Status frames received: {len(vesc_statuses)}")
    if vesc_statuses:
        rpms = [s["rpm"] for s in vesc_statuses]
        max_rpm = max(rpms)
        min_rpm = min(rpms)
        peak = max_rpm if abs(max_rpm) >= abs(min_rpm) else min_rpm
        print(f"Peak Speed Reached: {peak} ERPM ({abs(peak)//12} Mech RPM)")
        print("\nTelemetry samples during test:")
        for s in vesc_statuses[-20::4]:
            print(f"  t={s['time']-t0:.2f}s | ERPM={s['rpm']:6d} ({abs(s['rpm'])//12:4d} mech) | V={s['voltage']:.1f}V | I={s['current']:.2f}A | T={s['temperature']:.1f}C")

        if abs(peak) >= 150:
            print("\n[PASS] Method A DroneCAN RawCommand Test SUCCEEDED! Motor spun and telemetry verified.")
            node.close()
            return True
        else:
            print("\n[FAIL] Motor did not reach target speed.")
            node.close()
            return False
    else:
        print("\n[FAIL] No telemetry received from VESC Node 103.")
        node.close()
        return False

if __name__ == "__main__":
    cmd = int(sys.argv[1]) if len(sys.argv) > 1 else 600
    res = test_uavcan_method_a(target_cmd=cmd)
    sys.exit(0 if res else 1)
