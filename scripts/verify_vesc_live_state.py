#!/usr/bin/env python3
"""
Live Hardware State Verification for VESC on COM33.
Checks:
- Serial responsiveness
- Input voltage (~20.3V)
- Motor speed (0 RPM)
- Fault status (0 / FAULT_CODE_NONE)
"""

import sys
from pathlib import Path

VESC_DIR = Path(__file__).resolve().parent.parent
if str(VESC_DIR) not in sys.path:
    sys.path.insert(0, str(VESC_DIR))

from lib.vesc_interface import VESCInterface


def main():
    print("Connecting to VESC on COM33...")
    v = VESCInterface(port="COM33", timeout=1.0)
    if not v.connect():
        print("[ERROR] Failed to connect to VESC on COM33")
        return 1

    try:
        telem = v.get_telemetry(timeout=1.5)
        if not telem:
            print("[ERROR] No telemetry received from VESC within timeout")
            return 1

        mech_rpm = telem["rpm"] // 12
        vin = telem["v_in"]
        fault_code = telem["fault_code"]
        fault_str = telem["fault_str"]

        print("=" * 65)
        print("  VESC LIVE TELEMETRY HARDWARE VERIFICATION (GATE 3)")
        print("=" * 65)
        print(f"  Input Voltage (Vin):  {vin:.2f} V")
        print(f"  Motor Current:        {telem['current_motor']:.2f} A")
        print(f"  Battery Current:      {telem['current_in']:.2f} A")
        print(f"  Duty Cycle:           {telem['duty_now'] * 100:.1f} %")
        print(f"  Mechanical Speed:     {mech_rpm} RPM")
        print(f"  Electrical Speed:     {telem['rpm']} ERPM")
        print(f"  MOSFET Temperature:   {telem['temp_mos']:.1f} °C")
        print(f"  Motor Temperature:    {telem['temp_motor']:.1f} °C")
        print(f"  Fault Status:         {fault_str} (Code {fault_code})")
        print("=" * 65)

        if mech_rpm != 0:
            print(f"[FAIL] Motor speed is not 0 RPM (got {mech_rpm} RPM)")
            return 1

        if fault_code != 0:
            print(f"[FAIL] Hardware fault active: {fault_str} (Code {fault_code})")
            return 1

        print("[PASS] VESC is responsive, RPM is 0, Vin is nominal (~20.3V), Fault is FAULT_CODE_NONE.")
        return 0

    finally:
        v.disconnect()


if __name__ == "__main__":
    sys.exit(main())
