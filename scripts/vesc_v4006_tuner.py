"""VESC Auto-Tuning & Provisioning CLI Tool for Sunnysky V4006.

Subcommands:
  status:      Read and print firmware and live telemetry status.
  monitor:     Stream real-time telemetry dashboard (10-20 Hz).
  test-rpm:    Safely ramp motor to target mechanical RPM with hardware safety aborts.
  export-xml:  Export VESC Tool XML files from JSON profiles.
"""

import sys
import os
import time
import argparse
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Add project root and VESC root to path
VESC_DIR = Path(__file__).resolve().parent.parent
if str(VESC_DIR) not in sys.path:
    sys.path.insert(0, str(VESC_DIR))

from lib.vesc_protocol import FAULT_CODES
from lib.vesc_interface import (
    VESCInterface,
    VESCSLCANInterface,
    throttle_to_mech_rpm,
    mech_rpm_to_erpm
)
from scripts.export_vesc_xml import export_xml, DEFAULT_PROFILE, DEFAULT_PROFILES_DIR


DEFAULT_VESC_PORT = "COM33"
DEFAULT_CAN_PORT = "COM16"
DEFAULT_NODE_ID = 103
DEFAULT_ESC_INDEX = 1
MOTOR_POLE_PAIRS = 12  # 24 poles -> 12 pole pairs


def get_interface(args):
    """Factory creating VESCInterface or VESCSLCANInterface based on CLI arguments."""
    if getattr(args, "use_can", False):
        port = getattr(args, "can_port", DEFAULT_CAN_PORT)
        node_id = getattr(args, "node_id", DEFAULT_NODE_ID)
        esc_index = getattr(args, "esc_index", DEFAULT_ESC_INDEX)
        print(f"[CAN] Initializing SLCAN interface on {port} (Node ID {node_id}, ESC {esc_index})...")
        iface = VESCSLCANInterface(port=port, node_id=node_id, esc_index=esc_index, timeout=0.8)
    else:
        port = getattr(args, "port", DEFAULT_VESC_PORT)
        baud = getattr(args, "baud", 115200)
        print(f"[USB] Initializing VESC Direct Serial on {port} @ {baud} bps...")
        iface = VESCInterface(port=port, baudrate=baud, timeout=0.8)
    return iface


def cmd_status(args):
    """Checks connection and prints current status."""
    iface = get_interface(args)
    if not iface.connect():
        print(f"[ERROR] Failed to connect to VESC.")
        sys.exit(1)

    print("[OK] Connected successfully.")
    try:
        if isinstance(iface, VESCInterface):
            fw = iface.get_fw_version(timeout=1.0)
            if fw:
                print(f"Firmware: V{fw['major']}.{fw['minor']} (HW: {fw['hw_name']})")

        print("Querying telemetry...")
        telem = iface.get_telemetry(timeout=1.0)
        if telem is None:
            print("[WARN] No telemetry response received within timeout.")
        else:
            mech_rpm = telem['rpm'] // MOTOR_POLE_PAIRS
            print("-" * 50)
            print(f"  Voltage:        {telem['v_in']:.2f} V")
            print(f"  Motor Current:  {telem['current_motor']:.2f} A")
            print(f"  Battery Current:{telem['current_in']:.2f} A")
            print(f"  Duty Cycle:     {telem['duty_now'] * 100:.1f} %")
            print(f"  Speed:          {mech_rpm} RPM ({telem['rpm']} ERPM)")
            print(f"  MOSFET Temp:    {telem['temp_mos']:.1f} °C")
            print(f"  Motor Temp:     {telem['temp_motor']:.1f} °C")
            print(f"  Fault Status:   {telem['fault_str']} (Code {telem['fault_code']})")
            print("-" * 50)
    finally:
        iface.disconnect()


def cmd_monitor(args):
    """Streams live telemetry dashboard."""
    rate = max(1, min(50, args.rate))
    delay = 1.0 / rate
    duration = args.duration

    iface = get_interface(args)
    if not iface.connect():
        print(f"[ERROR] Failed to connect for telemetry monitor.")
        sys.exit(1)

    print(f"[MONITOR] Streaming telemetry at {rate} Hz (duration: {duration}s, Ctrl+C to stop)...")
    header = f"{'Time(s)':<8} | {'V_in(V)':<8} | {'I_mot(A)':<9} | {'I_bat(A)':<9} | {'RPM':<7} | {'ERPM':<8} | {'Duty%':<6} | {'T_fet(C)':<8} | {'Fault':<15}"
    print("-" * len(header))
    print(header)
    print("-" * len(header))

    start_time = time.time()
    try:
        while True:
            elapsed = time.time() - start_time
            if duration > 0 and elapsed >= duration:
                break

            telem = iface.get_telemetry(timeout=delay * 0.9)
            if telem:
                mech_rpm = telem['rpm'] // MOTOR_POLE_PAIRS
                print(f"{elapsed:<8.2f} | {telem['v_in']:<8.2f} | {telem['current_motor']:<9.2f} | {telem['current_in']:<9.2f} | {mech_rpm:<7} | {telem['rpm']:<8} | {telem['duty_now']*100:<6.1f} | {telem['temp_mos']:<8.1f} | {telem['fault_str']:<15}")
            else:
                print(f"{elapsed:<8.2f} | {'--':<8} | {'--':<9} | {'--':<9} | {'--':<7} | {'--':<8} | {'--':<6} | {'--':<8} | {'TIMEOUT':<15}")

            time.sleep(delay)
    except KeyboardInterrupt:
        print("\n[STOP] Monitor stopped by user.")
    finally:
        iface.disconnect()


def cmd_test_rpm(args):
    """Executes safe RPM test with hardware safety guards."""
    target_mech_rpm = args.target
    duration = args.duration
    step = args.ramp_step
    limit_current = args.max_current
    min_voltage = args.min_voltage

    print("=" * 60)
    print("  HARDWARE SAFETY PROTOCOL (RULE 6 ENFORCED)")
    print(f"  Target Mechanical Speed: {target_mech_rpm} RPM ({target_mech_rpm * MOTOR_POLE_PAIRS} ERPM)")
    print(f"  Hold Duration:           {duration} s")
    print(f"  Ramp Step:               {step} RPM")
    print(f"  Current Safety Limit:    {limit_current} A")
    print(f"  Low Voltage Cutoff:      {min_voltage} V")
    print("  SAFETY INVARIANT: Current-limited DC supply (<= 5A) required!")
    print("=" * 60)

    iface = get_interface(args)
    if not iface.connect():
        print(f"[ERROR] Cannot connect to VESC for RPM test.")
        sys.exit(1)

    try:
        # Pre-check telemetry
        telem = iface.get_telemetry(timeout=1.0)
        if telem is None:
            print("[ABORT] Could not verify baseline telemetry. Aborting for hardware safety.")
            sys.exit(1)

        if telem['v_in'] < min_voltage:
            print(f"[ABORT] Supply voltage ({telem['v_in']:.2f}V) below safety threshold ({min_voltage}V)!")
            sys.exit(1)

        if telem['fault_code'] != 0:
            print(f"[ABORT] VESC in fault state: {telem['fault_str']}! Clear fault before running.")
            sys.exit(1)

        print(f"[PRE-CHECK OK] Battery: {telem['v_in']:.2f}V, Temp: {telem['temp_mos']:.1f}°C. Starting ramp...")

        # 1. Ramp Up
        current_rpm = 0
        while current_rpm < target_mech_rpm:
            current_rpm = min(target_mech_rpm, current_rpm + step)
            iface.set_mech_rpm(current_rpm)

            telem = iface.get_telemetry(timeout=0.1)
            if telem:
                if telem['v_in'] < min_voltage:
                    print(f"\n[EMERGENCY STOP] Voltage drop detected ({telem['v_in']:.2f}V < {min_voltage}V)!")
                    iface.stop()
                    sys.exit(2)
                if abs(telem['current_motor']) > limit_current or abs(telem['current_in']) > limit_current:
                    print(f"\n[EMERGENCY STOP] Overcurrent detected (I_mot={telem['current_motor']:.2f}A > {limit_current}A)!")
                    iface.stop()
                    sys.exit(3)
                if telem['fault_code'] != 0:
                    print(f"\n[EMERGENCY STOP] VESC Fault {telem['fault_str']}!")
                    iface.stop()
                    sys.exit(4)

                actual_rpm = telem['rpm'] // MOTOR_POLE_PAIRS
                print(f"  [RAMP UP] Target: {current_rpm} RPM | Actual: {actual_rpm} RPM | I: {telem['current_motor']:.2f}A | V: {telem['v_in']:.1f}V")

            time.sleep(0.08)

        # 2. Hold Target Speed
        print(f"[HOLD] Holding at {target_mech_rpm} RPM for {duration} seconds...")
        hold_end = time.time() + duration
        while time.time() < hold_end:
            iface.set_mech_rpm(target_mech_rpm)
            telem = iface.get_telemetry(timeout=0.1)
            if telem:
                if telem['v_in'] < min_voltage or abs(telem['current_motor']) > limit_current:
                    print("\n[EMERGENCY STOP] Parameter violation during hold phase!")
                    iface.stop()
                    sys.exit(5)
                actual_rpm = telem['rpm'] // MOTOR_POLE_PAIRS
                print(f"  [HOLD] Speed: {actual_rpm} RPM | I_mot: {telem['current_motor']:.2f}A | I_bat: {telem['current_in']:.2f}A | V: {telem['v_in']:.1f}V")
            time.sleep(0.1)

        # 3. Ramp Down
        print(f"[RAMP DOWN] Safely decelerating to 0 RPM...")
        while current_rpm > 0:
            current_rpm = max(0, current_rpm - step)
            iface.set_mech_rpm(current_rpm)
            time.sleep(0.05)

        iface.stop()
        print("[SUCCESS] Test completed safely. Motor stopped.")

    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted. Stopping motor immediately...")
        iface.stop()
    finally:
        iface.stop()
        iface.disconnect()


def cmd_test_throttle(args):
    """Tests throttle input (0.01 to 1.0) with linear 1000-6000 mechanical RPM mapping."""
    raw_throttle = args.throttle
    if raw_throttle > 1.0:
        throttle = raw_throttle / 100.0
    else:
        throttle = raw_throttle

    target_mech_rpm = throttle_to_mech_rpm(throttle, min_rpm=1000.0, max_rpm=6000.0)
    target_erpm = mech_rpm_to_erpm(target_mech_rpm)
    duration = args.duration
    limit_current = args.max_current
    min_voltage = args.min_voltage

    print("=" * 65)
    print(f"  [VESC THROTTLE TEST] Input: {throttle*100:.1f}% throttle")
    print(f"  Linear Mapping:     {target_mech_rpm} Mechanical RPM")
    print(f"  Electrical ERPM:    {target_erpm} ERPM (12 pole pairs)")
    print(f"  Safety Limits:      Max I: {limit_current}A | Min V: {min_voltage}V")
    print("=" * 65)

    if target_mech_rpm == 0:
        print("[INFO] Throttle < 1% maps to 0 RPM (Motor Off / Disarmed).")
        return

    iface = get_interface(args)
    if not iface.connect():
        print(f"[ERROR] Failed to connect.")
        sys.exit(1)

    try:
        telem = iface.get_telemetry(timeout=1.0)
        if telem is None:
            print("[ABORT] Could not verify baseline telemetry.")
            sys.exit(1)

        if telem['v_in'] < min_voltage:
            print(f"[ABORT] Supply voltage ({telem['v_in']:.2f}V) below threshold ({min_voltage}V)!")
            sys.exit(1)

        if telem['fault_code'] != 0:
            print(f"[ABORT] VESC in fault state: {telem['fault_str']}!")
            sys.exit(1)

        print(f"[PRE-CHECK OK] Battery: {telem['v_in']:.2f}V, Temp: {telem['temp_mos']:.1f}°C.")
        print(f"[RAMPING] Smooth breakaway to {target_mech_rpm} RPM...")

        current_rpm = 0
        step = 100
        while current_rpm < target_mech_rpm:
            current_rpm = min(target_mech_rpm, current_rpm + step)
            iface.set_mech_rpm(current_rpm)
            telem = iface.get_telemetry(timeout=0.08)
            if telem:
                if telem['v_in'] < min_voltage or abs(telem['current_motor']) > limit_current or telem['fault_code'] != 0:
                    print(f"\n[EMERGENCY STOP] Safety violation during ramp!")
                    iface.stop()
                    sys.exit(2)
            time.sleep(0.05)

        print(f"[HOLD] Holding at {target_mech_rpm} RPM for {duration} seconds...")
        hold_end = time.time() + duration
        while time.time() < hold_end:
            iface.set_mech_rpm(target_mech_rpm)
            telem = iface.get_telemetry(timeout=0.1)
            if telem:
                if telem['v_in'] < min_voltage or abs(telem['current_motor']) > limit_current or telem['fault_code'] != 0:
                    print(f"\n[EMERGENCY STOP] Safety violation during hold!")
                    iface.stop()
                    sys.exit(3)
                actual_rpm = telem['rpm'] // MOTOR_POLE_PAIRS
                print(f"  Throttle: {throttle*100:4.1f}% | Target: {target_mech_rpm} RPM | Actual: {actual_rpm:4d} RPM | I_mot: {telem['current_motor']:4.2f}A | V: {telem['v_in']:.1f}V | Duty: {telem['duty_now']*100:4.1f}%")
            time.sleep(0.1)

        print("[RAMP DOWN] Decelerating safely to 0 RPM...")
        while current_rpm > 0:
            current_rpm = max(0, current_rpm - step)
            iface.set_mech_rpm(current_rpm)
            time.sleep(0.04)

        iface.stop()
        print(f"[SUCCESS] Throttle test {throttle*100:.1f}% completed safely.")

    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted. Stopping motor immediately...")
        iface.stop()
    finally:
        iface.stop()
        iface.disconnect()


def cmd_test_low_sweep(args):
    """Executes 1% to 5% low-throttle sweep to verify smooth breakaway and jitter-free operation."""
    duration_per_step = args.step_duration
    limit_current = args.max_current
    min_voltage = args.min_voltage

    throttle_steps = [0.01, 0.02, 0.03, 0.04, 0.05]
    print("=" * 70)
    print("  [VESC 1-5% LOW-THROTTLE SWEEP VERIFICATION]")
    print("  Goal: Verify smooth breakaway ('đề 3') and jitter-free 1-5% operation")
    print("  Mapping: 1% -> 1000 RPM (12k ERPM) up to 5% -> 1202 RPM (14.4k ERPM)")
    print(f"  Safety: Max I = {limit_current}A | Min V = {min_voltage}V")
    print("=" * 70)

    iface = get_interface(args)
    if not iface.connect():
        print("[ERROR] Failed to connect.")
        sys.exit(1)

    try:
        telem = iface.get_telemetry(timeout=1.0)
        if telem is None or telem['v_in'] < min_voltage or telem['fault_code'] != 0:
            print("[ABORT] Pre-check failed.")
            sys.exit(1)

        results = []
        current_rpm = 0
        ramp_step = 50

        # Soft breakaway from 0 RPM into BEMF observer tracking range
        print("  [BREAKAWAY] Executing smooth breakaway sequence from 0 RPM...")
        for d in [0.02, 0.03, 0.04, 0.05]:
            iface.set_duty(d)
            time.sleep(0.08)

        for t_val in throttle_steps:
            target_rpm = throttle_to_mech_rpm(t_val, 1000.0, 6000.0)
            target_erpm = mech_rpm_to_erpm(target_rpm)
            print(f"\n---> STEP: {t_val*100:.0f}% Throttle -> Target: {target_rpm} RPM ({target_erpm} ERPM)")

            dur = duration_per_step + 1.0 if t_val == 0.01 else duration_per_step
            step_end = time.time() + dur
            samples = []
            while time.time() < step_end:
                iface.set_rpm(target_erpm)
                t = iface.get_telemetry(timeout=0.08)
                if t:
                    if t['v_in'] < min_voltage or abs(t['current_motor']) > limit_current or t['fault_code'] != 0:
                        print(f"\n[EMERGENCY STOP] Parameter violation at {t_val*100:.0f}% throttle! (V={t['v_in']:.2f}V, I={t['current_motor']:.2f}A, Fault={t['fault_str']})")
                        iface.stop()
                        sys.exit(4)
                    act_rpm = t['rpm'] // MOTOR_POLE_PAIRS
                    samples.append(act_rpm)
                    print(f"     Actual: {act_rpm:4d} RPM | I_mot: {t['current_motor']:4.2f}A | I_bat: {t['current_in']:4.2f}A | Duty: {t['duty_now']*100:4.1f}% | Fault: {t['fault_str']}")
                time.sleep(0.1)

            settled = samples[-6:] if len(samples) >= 6 else samples
            avg_rpm = sum(settled) / len(settled) if settled else 0
            results.append({"throttle": t_val, "target": target_rpm, "avg_actual": avg_rpm})

        print("\n[RAMP DOWN] Decelerating safely to 0 RPM...")
        for d in [0.04, 0.03, 0.02, 0.0]:
            iface.set_duty(d)
            time.sleep(0.08)
        iface.stop()

        print("\n" + "=" * 70)
        print("  [SWEEP SUMMARY TABLE]")
        print("  Throttle | Target RPM | Avg Actual RPM | Tracking Error | Status")
        print("  " + "-" * 66)
        for r in results:
            err = abs(r['avg_actual'] - r['target'])
            status = "PASS (Smooth)" if err < 150 else "WARN"
            print(f"  {r['throttle']*100:6.1f}% | {r['target']:10d} | {r['avg_actual']:14.1f} | {err:13.1f} | {status}")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted. Stopping motor...")
        iface.stop()
    finally:
        iface.stop()
        iface.disconnect()


def cmd_export_xml(args):
    """Exports XML configuration from JSON profile."""
    profile_path = args.profile
    motor_xml = args.motor_xml or (args.out_dir / "v4006_motor_config.xml")
    app_xml = args.app_xml or (args.out_dir / "v4006_app_uavcan.xml")
    export_xml(profile_path, motor_xml, app_xml)


def main():
    # Common parent parser for communication
    comm_parser = argparse.ArgumentParser(add_help=False)
    comm_parser.add_argument("--port", type=str, default=DEFAULT_VESC_PORT,
                             help=f"VESC USB Serial port (default: {DEFAULT_VESC_PORT})")
    comm_parser.add_argument("--baud", type=int, default=115200,
                             help="Baud rate for serial connection (default: 115200)")
    comm_parser.add_argument("--use-can", action="store_true",
                             help="Use SLCAN CAN interface instead of direct USB")
    comm_parser.add_argument("--can-port", type=str, default=DEFAULT_CAN_PORT,
                             help=f"SLCAN adapter COM port (default: {DEFAULT_CAN_PORT})")
    comm_parser.add_argument("--node-id", type=int, default=DEFAULT_NODE_ID,
                             help=f"DroneCAN/VESC CAN Node ID (default: {DEFAULT_NODE_ID})")
    comm_parser.add_argument("--esc-index", type=int, default=DEFAULT_ESC_INDEX,
                             help=f"UAVCAN ESC Index (default: {DEFAULT_ESC_INDEX})")

    parser = argparse.ArgumentParser(
        description="VESC Auto-Tuning and Provisioning Tool for Sunnysky V4006.",
        parents=[comm_parser]
    )
    subparsers = parser.add_subparsers(dest="command", required=True, help="Subcommand to execute")

    # Status subcommand
    status_parser = subparsers.add_parser("status", parents=[comm_parser],
                                          help="Read and display VESC telemetry and fault status")
    status_parser.set_defaults(func=cmd_status)

    # Monitor subcommand
    monitor_parser = subparsers.add_parser("monitor", parents=[comm_parser],
                                           help="Stream live telemetry dashboard at specified rate")
    monitor_parser.add_argument("--rate", type=int, default=10, help="Stream update rate in Hz (default: 10)")
    monitor_parser.add_argument("--duration", type=float, default=10.0, help="Monitoring duration in seconds (0 for indefinite, default: 10.0)")
    monitor_parser.set_defaults(func=cmd_monitor)

    # Test-RPM subcommand
    test_parser = subparsers.add_parser("test-rpm", parents=[comm_parser],
                                        help="Ramp motor to target mechanical RPM with hardware safety aborts")
    test_parser.add_argument("--target", type=int, required=True, help="Target mechanical RPM (1000 to 6000)")
    test_parser.add_argument("--duration", type=float, default=3.0, help="Hold duration at target in seconds (default: 3.0)")
    test_parser.add_argument("--ramp-step", type=int, default=100, help="Ramp step increment in mechanical RPM (default: 100)")
    test_parser.add_argument("--max-current", type=float, default=12.0, help="Safety current threshold in Amps (default: 12.0A)")
    test_parser.add_argument("--min-voltage", type=float, default=10.0, help="Low voltage cutoff in Volts (default: 10.0V)")
    test_parser.set_defaults(func=cmd_test_rpm)

    # Test-Throttle subcommand (1000 - 6000 RPM linear mapping)
    throttle_parser = subparsers.add_parser("test-throttle", parents=[comm_parser],
                                            help="Test specific throttle (0.01 to 1.0) linearly mapped to 1000-6000 RPM")
    throttle_parser.add_argument("--throttle", type=float, required=True,
                                help="Throttle input (0.01 to 1.0 or percent 1 to 100, e.g. 0.05 for 5%%)")
    throttle_parser.add_argument("--duration", type=float, default=3.0, help="Hold duration in seconds (default: 3.0)")
    throttle_parser.add_argument("--max-current", type=float, default=12.0, help="Safety current threshold in Amps (default: 12.0A)")
    throttle_parser.add_argument("--min-voltage", type=float, default=10.0, help="Low voltage cutoff in Volts (default: 10.0V)")
    throttle_parser.set_defaults(func=cmd_test_throttle)

    # Test-Low-Sweep subcommand (1% to 5% smooth breakaway verification)
    sweep_parser = subparsers.add_parser("test-low-sweep", parents=[comm_parser],
                                         help="Sweep 1%% -> 2%% -> 3%% -> 4%% -> 5%% throttle to verify smooth breakaway and no jitter")
    sweep_parser.add_argument("--step-duration", type=float, default=2.0, help="Hold duration per step in seconds (default: 2.0)")
    sweep_parser.add_argument("--max-current", type=float, default=12.0, help="Safety current threshold in Amps (default: 12.0A)")
    sweep_parser.add_argument("--min-voltage", type=float, default=10.0, help="Low voltage cutoff in Volts (default: 10.0V)")
    sweep_parser.set_defaults(func=cmd_test_low_sweep)

    # Export-XML subcommand
    export_parser = subparsers.add_parser("export-xml",
                                          help="Export VESC Tool XML files from JSON profile")
    export_parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE,
                               help=f"Path to profile JSON (default: {DEFAULT_PROFILE.name})")
    export_parser.add_argument("--motor-xml", type=Path, default=None,
                               help="Destination path for Motor Config XML")
    export_parser.add_argument("--app-xml", type=Path, default=None,
                               help="Destination path for App Config XML")
    export_parser.add_argument("--out-dir", type=Path, default=DEFAULT_PROFILES_DIR,
                               help="Output directory (default: profiles/)")
    export_parser.set_defaults(func=cmd_export_xml)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
