"""Set VESC UAVCAN mode to RPM Control Loop (1000 - 6000 Mech RPM).

Settings applied:
- can_mode = 1 (CAN_MODE_UAVCAN)
- uavcan_esc_index = 1 (ESC 2 / Channel 1)
- uavcan_raw_mode = 3 (UAVCAN_RAW_MODE_RPM)
- uavcan_raw_rpm_max = 72000.0 ERPM (12 pole pairs -> 6000 Mech RPM)
"""

import sys
import time
import struct
import serial
from pathlib import Path

VESC_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(VESC_DIR))

from scripts.enable_vesc_uavcan_and_pwm import get_appconf, set_appconf, reboot_vesc

def configure_uavcan_rpm_mode(port: str = "COM33", baud: int = 115200, max_rpm: float = 6000.0, pole_pairs: int = 12):
    max_erpm = float(max_rpm * pole_pairs)
    print(f"[RPM CONFIG] Configuring VESC on {port}...")
    print(f"  - Target Max Mech RPM: {max_rpm:.0f} RPM")
    print(f"  - Pole Pairs:          {pole_pairs}")
    print(f"  - Target Max ERPM:     {max_erpm:.0f} ERPM")

    ser = serial.Serial(port, baud, timeout=1.0)
    appconf = bytearray(get_appconf(ser))
    if not appconf or len(appconf) < 50:
        print("[ERROR] Failed to read APPCONF from VESC!")
        ser.close()
        return False

    old_mode = appconf[25]
    old_max_erpm = struct.unpack(">f", appconf[26:30])[0]
    print(f"\n[CURRENT SETTINGS]")
    print(f"  - can_mode:          {appconf[23]} (1=UAVCAN)")
    print(f"  - uavcan_esc_index:  {appconf[24]}")
    print(f"  - uavcan_raw_mode:   {old_mode} (2=Duty, 3=RPM)")
    print(f"  - uavcan_raw_rpm_max:{old_max_erpm:.1f} ERPM")

    # Update to RPM mode
    appconf[23] = 1  # CAN_MODE_UAVCAN
    appconf[24] = 1  # uavcan_esc_index = 1
    appconf[25] = 3  # UAVCAN_RAW_MODE_RPM
    appconf[26:30] = struct.pack(">f", max_erpm)

    print(f"\n[WRITING NEW SETTINGS]")
    print(f"  -> can_mode:          1 (UAVCAN)")
    print(f"  -> uavcan_esc_index:  1 (Channel 2 / Index 1)")
    print(f"  -> uavcan_raw_mode:   3 (UAVCAN_RAW_MODE_RPM)")
    print(f"  -> uavcan_raw_rpm_max:{max_erpm:.1f} ERPM ({max_rpm:.0f} Mech RPM)")

    set_appconf(ser, bytes(appconf))
    time.sleep(0.3)

    print("[REBOOT] Rebooting VESC to apply RPM control loop...")
    reboot_vesc(ser)
    ser.close()
    time.sleep(1.8)

    # Reconnect and verify
    ser = serial.Serial(port, baud, timeout=1.0)
    new_conf = get_appconf(ser)
    ser.close()

    new_mode = new_conf[25]
    new_max_erpm = struct.unpack(">f", new_conf[26:30])[0]
    print(f"\n[VERIFIED VESC SETTINGS]")
    print(f"  - uavcan_raw_mode:   {new_mode} ({'PASS: RPM Mode' if new_mode == 3 else 'FAIL'})")
    print(f"  - uavcan_raw_rpm_max:{new_max_erpm:.1f} ERPM ({new_max_erpm/pole_pairs:.0f} Mech RPM)")

    if new_mode == 3 and abs(new_max_erpm - max_erpm) < 1.0:
        print("\n[SUCCESS] VESC is now configured in closed-loop RPM mode (0-100% -> 0-6000 Mech RPM)!")
        return True
    else:
        print("\n[FAIL] Verification failed!")
        return False

if __name__ == "__main__":
    success = configure_uavcan_rpm_mode()
    sys.exit(0 if success else 1)
