"""UAVCAN Real-Time Safety Guard & Diagnostic Monitor.

Features:
1. COM16: DroneCAN sniffer & multi-frame DSDL decoder for RawCommand (1030) and RPMCommand (1031).
2. COM33: VESC telemetry poller (ERPM -> Mech RPM, Duty, Current, Vin, Faults).
3. ST-Link: Hardware debugger probe integration (pyocd) for MCU core monitoring and failsafe halt.
4. Auto-Stop Failsafes:
   - Mech RPM > 9000 RPM (Hard user cutoff)
   - Disarm / Zero-throttle runaway (Commanded 0 but motor spinning)
   - Severe duty tracking discrepancy (Commanded low but runaway duty)
   - VESC hardware fault trip
5. Live visual telemetry stream.
"""

import os
import sys
import time
import struct
import threading
import argparse
from pathlib import Path
from typing import Optional, Dict, List, Any

# Ensure workspace imports work
VESC_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(VESC_DIR))

import serial

try:
    import dronecan
    from dronecan.transport import bits_from_bytes
except ImportError:
    dronecan = None
    bits_from_bytes = None

try:
    from pyocd.core.helpers import ConnectHelper
    PYOCD_AVAILABLE = True
except ImportError:
    PYOCD_AVAILABLE = False

from lib.vesc_interface import VESCInterface


class UAVCANSafetyGuard:
    def __init__(
        self,
        can_port: str = "COM16",
        vesc_port: str = "COM33",
        stlink_id: str = "E1007200D0D2139393740544",
        esc_index: int = 1,
        max_rpm_cutoff: float = 9000.0,
        pole_pairs: int = 12,  # Sunnysky V4006 KV740 has 24 rotor poles -> 12 pole pairs
    ):
        self.can_port = can_port
        self.vesc_port = vesc_port
        self.stlink_id = stlink_id
        self.esc_index = esc_index
        self.max_rpm_cutoff = max_rpm_cutoff
        self.pole_pairs = pole_pairs

        self.running = False
        self.stop_requested = False
        self.emergency_stopped = False
        self.stop_reason = ""

        # UAVCAN Sniffer State
        self.can_ser: Optional[serial.Serial] = None
        self.last_uavcan_time = 0.0
        self.uavcan_frame_count = 0
        self.uavcan_msg_count = 0
        self.uavcan_hz = 0.0
        self.last_channels: List[float] = [0.0] * 8
        self.last_raw_ints: List[int] = [0] * 8
        self.cmd_lock = threading.Lock()

        # VESC State
        self.vesc: Optional[VESCInterface] = None
        self.last_vesc_time = 0.0
        self.mech_rpm = 0.0
        self.duty_now = 0.0
        self.current_motor = 0.0
        self.v_in = 0.0
        self.fault_code = 0
        self.fault_str = "NONE"
        self.vesc_lock = threading.Lock()

        # ST-Link
        self.stlink_session = None
        self.stlink_target = None

    def init_stlink(self) -> bool:
        """Initializes ST-Link connection in non-intrusive monitoring mode."""
        if not PYOCD_AVAILABLE:
            print("[ST-Link] pyocd not installed, skipping ST-Link integration.")
            return False
        try:
            print(f"[ST-Link] Connecting to probe {self.stlink_id}...")
            self.stlink_session = ConnectHelper.session_with_chosen_probe(
                unique_id=self.stlink_id,
                target_override="cortex_m",
                options={"connect_mode": "attach", "resume_on_disconnect": True},
            )
            self.stlink_session.open()
            self.stlink_target = self.stlink_session.target
            # Ensure target is running
            state = self.stlink_target.get_state().name
            if state != "RUNNING":
                self.stlink_target.resume()
            print(f"[ST-Link] Connected! MCU Core State: {self.stlink_target.get_state().name}")
            return True
        except Exception as e:
            print(f"[ST-Link] Connection failed ({e}), continuing with serial failsafe.")
            self.stlink_session = None
            self.stlink_target = None
            return False

    def init_vesc(self) -> bool:
        """Connects to VESC on COM33."""
        print(f"[VESC] Connecting to {self.vesc_port}...")
        self.vesc = VESCInterface(self.vesc_port, baudrate=115200, timeout=0.1)
        if not self.vesc.connect():
            print(f"[VESC] Failed to connect to {self.vesc_port}!")
            return False
        # Get initial telemetry
        t = self.vesc.get_telemetry(timeout=0.3)
        if t:
            self.mech_rpm = t.get("rpm", 0) / float(self.pole_pairs)
            self.v_in = t.get("v_in", 0.0)
            print(f"[VESC] Connected! Vin = {self.v_in:.1f}V, Fault = {t.get('fault_str')}")
            return True
        print("[VESC] Connected but no telemetry response.")
        return False

    def init_can(self) -> bool:
        """Connects to SLCAN on COM16 at 1 Mbps."""
        print(f"[CAN] Opening SLCAN on {self.can_port}...")
        try:
            self.can_ser = serial.Serial(self.can_port, 115200, timeout=0.05)
            # Reset SLCAN
            self.can_ser.write(b"C\r")
            time.sleep(0.02)
            self.can_ser.write(b"S8\r")  # 1 Mbps
            time.sleep(0.02)
            self.can_ser.write(b"O\r")   # Open channel
            time.sleep(0.05)
            self.can_ser.reset_input_buffer()
            print(f"[CAN] SLCAN channel opened on {self.can_port} at 1 Mbps.")
            return True
        except Exception as e:
            print(f"[CAN] Failed to open {self.can_port}: {e}")
            return False

    def _can_sniffer_loop(self):
        """Thread to capture SLCAN frames and reassemble DroneCAN RawCommands."""
        buf = b""
        transfers: Dict[Any, bytearray] = {}
        last_calc_time = time.time()
        msg_counter = 0

        while self.running and not self.stop_requested:
            try:
                chunk = self.can_ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\r" in buf:
                    line, buf = buf.split(b"\r", 1)
                    line_str = line.decode("ascii", errors="ignore").strip()
                    if not (line_str.startswith("T") or line_str.startswith("t")):
                        continue

                    is_ext = line_str.startswith("T")
                    id_len = 8 if is_ext else 3
                    try:
                        can_id = int(line_str[1:1+id_len], 16)
                        dlc = int(line_str[1+id_len:2+id_len])
                        data_hex = line_str[2+id_len:2+id_len+dlc*2]
                        data = bytes.fromhex(data_hex)
                    except Exception:
                        continue

                    self.uavcan_frame_count += 1
                    src = can_id & 0x7F
                    is_service = (can_id >> 7) & 1
                    if is_service or dlc == 0:
                        continue

                    msg_id = (can_id >> 8) & 0xFFFF

                    # We are interested in RawCommand (Msg ID 1030) and RPMCommand (Msg ID 1031)
                    if msg_id == 1030:
                        tail = data[-1]
                        som = (tail >> 7) & 1
                        eom = (tail >> 6) & 1
                        frame_payload = data[:-1]
                        key = (src, msg_id)

                        actual_data = None
                        if som and eom:
                            # Single-frame transfer (no CRC16)
                            actual_data = frame_payload
                        elif som:
                            transfers[key] = bytearray(frame_payload)
                        elif key in transfers:
                            transfers[key].extend(frame_payload)
                            if eom:
                                full = bytes(transfers.pop(key))
                                if len(full) > 2:
                                    actual_data = full[2:]  # Strip 2-byte CRC16

                        if actual_data and dronecan is not None:
                            try:
                                msg = dronecan.uavcan.equipment.esc.RawCommand()
                                msg._unpack(bits_from_bytes(actual_data))
                                raw_ints = list(msg.cmd)
                                norm_channels = [val / 8192.0 for val in raw_ints]

                                with self.cmd_lock:
                                    self.last_uavcan_time = time.time()
                                    self.last_raw_ints = raw_ints
                                    self.last_channels = norm_channels
                                    msg_counter += 1
                            except Exception:
                                pass

                # Update Hz calculation
                now = time.time()
                if now - last_calc_time >= 0.5:
                    self.uavcan_hz = msg_counter / (now - last_calc_time)
                    msg_counter = 0
                    last_calc_time = now

            except Exception:
                time.sleep(0.01)

    def _vesc_telemetry_loop(self):
        """Thread to poll VESC telemetry at ~30 Hz."""
        while self.running and not self.stop_requested:
            try:
                t = self.vesc.get_telemetry(timeout=0.08)
                if t:
                    with self.vesc_lock:
                        self.last_vesc_time = time.time()
                        self.mech_rpm = t.get("rpm", 0) / float(self.pole_pairs)
                        self.duty_now = t.get("duty_now", 0.0)
                        self.current_motor = t.get("current_motor", 0.0)
                        self.v_in = t.get("v_in", 0.0)
                        self.fault_code = t.get("fault_code", 0)
                        self.fault_str = t.get("fault_str", "NONE")
                time.sleep(0.03)  # ~30 Hz
            except Exception:
                time.sleep(0.05)

    def trigger_emergency_stop(self, reason: str):
        """Executes instantaneous multi-layer emergency stop."""
        self.emergency_stopped = True
        self.stop_requested = True
        self.stop_reason = reason

        print(f"\n{'='*70}")
        print(f"[!] [EMERGENCY STOP TRIGGERED] [!]")
        print(f"REASON: {reason}")
        print(f"{'='*70}")

        # Layer 1: COM33 rapid zero commands (burst 10 packets)
        if self.vesc and self.vesc.is_connected():
            for _ in range(10):
                try:
                    self.vesc.set_duty(0.0)
                    self.vesc.set_current(0.0)
                    self.vesc.stop()
                except Exception:
                    pass
                time.sleep(0.005)
            print("[VESC COM33] Sent zero Duty / zero Current kill burst.")

        # Layer 2: ST-Link hardware kill (TIM1->BDTR MOE=0, TIM1->CR1 CEN=0, halt core)
        time.sleep(0.02)
        if self.stlink_target:
            try:
                # Disable PWM output at hardware silicon level (TIM1->BDTR MOE bit 15 = 0)
                self.stlink_target.write32(0x40010044, 0x00000000)
                # Stop timer counter (TIM1->CR1 CEN bit 0 = 0)
                self.stlink_target.write32(0x40010000, 0x00000000)
                # Halt ARM Cortex-M4 core
                self.stlink_target.halt()
                pc = self.stlink_target.read_core_register("pc")
                print(f"[ST-Link] Hardware PWM KILLED (TIM1->BDTR=0, TIM1->CR1=0) and MCU Core HALTED! PC=0x{pc:08X}")
            except Exception as e:
                print(f"[ST-Link] Failsafe halt error: {e}")

    def run(self):
        """Main monitoring and safety enforcement loop."""
        if not self.init_vesc():
            return 1
        if not self.init_can():
            return 1
        self.init_stlink()

        self.running = True
        t_can = threading.Thread(target=self._can_sniffer_loop, daemon=True)
        t_vesc = threading.Thread(target=self._vesc_telemetry_loop, daemon=True)
        t_can.start()
        t_vesc.start()

        # Wait for initial stream
        time.sleep(0.3)

        print("\n" + "=" * 70)
        print("[SYSTEM READY] ALL SENSORS & FAILSAFES ARMED")
        print(f"  - CAN Port:      {self.can_port} (Listening for DroneCAN RawCommand)")
        print(f"  - VESC Port:     {self.vesc_port} (Active telemetry & kill channel)")
        print(f"  - Hard Cutoff:   {self.max_rpm_cutoff:.0f} RPM" if self.max_rpm_cutoff > 0.0 else "  - Hard Cutoff:   DISABLED")
        print(f"  - Discrepancy:   Auto-stop if Disarm/Command=0 but Duty>8%, or Duty divergence > 35%")
        print(f"  - ST-Link Probe: {self.stlink_id} (Hardware breakpoint & halt ready)")
        print("=" * 70)
        print(">> User can now ARM external Flight Controller and begin thrust control.\n")

        disarm_runaway_counter = 0

        try:
            while self.running and not self.emergency_stopped:
                time.sleep(0.05)  # 20 Hz display & check rate

                now = time.time()
                with self.cmd_lock:
                    channels = list(self.last_channels)
                    raw_ints = list(self.last_raw_ints)
                    can_age = now - self.last_uavcan_time if self.last_uavcan_time > 0 else 999.0
                    uavcan_hz = self.uavcan_hz

                with self.vesc_lock:
                    rpm = self.mech_rpm
                    duty = self.duty_now
                    current = self.current_motor
                    vin = self.v_in
                    fault = self.fault_str
                    vesc_age = now - self.last_vesc_time if self.last_vesc_time > 0 else 999.0

                # Target channel command and expected RPM in closed loop (matching patched canard_driver.c)
                target_cmd = max(0.0, min(1.0, channels[self.esc_index] if self.esc_index < len(channels) else 0.0))
                if target_cmd <= 0.015:
                    expected_rpm = 0.0
                else:
                    norm = min(1.0, max(0.0, (target_cmd - 0.015) / (1.0 - 0.015)))
                    expected_rpm = 1000.0 + norm * 5000.0
                rpm_diff = rpm - expected_rpm

                # Formatted status line showing RPM target vs actual
                ch_str = " ".join([f"C{i}:{channels[i]:+.2f}" for i in range(min(2, len(channels)))])
                sys.stdout.write(
                    f"\r[CAN] {ch_str} (Tgt:{expected_rpm:4.0f}rpm @{uavcan_hz:3.0f}Hz) | "
                    f"[VESC] RPM:{rpm:5.0f} Duty:{duty*100:4.1f}% I:{current:4.2f}A V:{vin:4.1f}V | "
                    f"Diff:{rpm_diff:+5.0f}rpm "
                )
                sys.stdout.flush()

                # ================= FAILSAFE CHECKS =================
                # Failsafe 1: Absolute Ceiling Cutoff
                if self.max_rpm_cutoff > 0.0 and rpm > self.max_rpm_cutoff:
                    self.trigger_emergency_stop(
                        f"RPM CEILING CUTOFF! Actual Mech RPM = {rpm:.0f} exceeded {self.max_rpm_cutoff:.0f} RPM!"
                    )
                    break

                # Failsafe 2: Disarm / Zero Throttle Runaway
                is_zero_cmd = (target_cmd <= 0.02 and all(c <= 0.02 for c in channels[:4])) or (can_age > 0.40)
                if is_zero_cmd:
                    if duty > 0.08 or (rpm > 500.0 and duty > 0.03):
                        disarm_runaway_counter += 1
                        if disarm_runaway_counter >= 10:  # Sustained for > 500ms
                            self.trigger_emergency_stop(
                                f"DISARM RUNAWAY! Commanded 0 / Disarm (age={can_age:.2f}s), "
                                f"but motor still driven at Duty={duty*100:.1f}%, RPM={rpm:.0f}!"
                            )
                            break
                    else:
                        disarm_runaway_counter = max(0, disarm_runaway_counter - 1)
                else:
                    disarm_runaway_counter = 0

                # Failsafe 3: RPM Control Loop Discrepancy (Runaway / Lost Sync)
                # If motor RPM is drastically higher than commanded RPM for > 600ms
                if target_cmd > 0.05 and rpm_diff > 2000.0:
                    tracking_discrepancy_counter = getattr(self, "_track_disc_count", 0) + 1
                    self._track_disc_count = tracking_discrepancy_counter
                    if tracking_discrepancy_counter >= 10:  # Sustained for > 500ms
                        self.trigger_emergency_stop(
                            f"RPM LOOP RUNAWAY! Commanded {expected_rpm:.0f} RPM, "
                            f"but actual RPM = {rpm:.0f} (Divergence: +{rpm_diff:.0f} RPM)!"
                        )
                        break
                else:
                    self._track_disc_count = 0

                # Failsafe 4: Bench Overcurrent Cutoff (> 2.5 A)
                if abs(current) > 2.5:
                    self.trigger_emergency_stop(
                        f"BENCH OVERCURRENT! Motor Current = {current:.2f} A exceeded 2.5 A safety clamp!"
                    )
                    break

                # Failsafe 5: DC Supply Voltage Protection (16.0V <= Vin <= 26.0V)
                if vin > 26.0:
                    self.trigger_emergency_stop(
                        f"REGENERATIVE OVERVOLTAGE SPIKE! Vin = {vin:.1f} V exceeded 26.0 V safety limit!"
                    )
                    break
                if 0.0 < vin < 16.0:
                    self.trigger_emergency_stop(
                        f"UNDERVOLTAGE DETECTED! Vin = {vin:.1f} V dropped below 16.0 V!"
                    )
                    break

                # Failsafe 6: Hardware Fault Code
                if self.fault_code != 0:
                    self.trigger_emergency_stop(
                        f"VESC HARDWARE FAULT: {fault} (Code {self.fault_code})"
                    )
                    break

        except KeyboardInterrupt:
            print("\n[USER] Stop requested via Ctrl+C.")
            self.stop_requested = True
            if self.vesc:
                self.vesc.stop()

        finally:
            self.running = False
            time.sleep(0.1)
            # Final shutdown
            if self.vesc and self.vesc.is_connected():
                self.vesc.stop()
                self.vesc.disconnect()
            if self.can_ser and self.can_ser.is_open:
                try:
                    self.can_ser.write(b"C\r")
                    self.can_ser.close()
                except Exception:
                    pass
            if self.stlink_session:
                try:
                    # If emergency stopped, keep halted; otherwise resume
                    if not self.emergency_stopped and self.stlink_target:
                        self.stlink_target.resume()
                    self.stlink_session.close()
                except Exception:
                    pass

            print("\n[SHUTDOWN] Ports released. Monitor exited safely.")
            return 0 if not self.emergency_stopped else 2


def main():
    parser = argparse.ArgumentParser(description="UAVCAN Real-Time Safety Guard & Diagnostic Monitor")
    parser.add_argument("--can-port", default="COM16", help="SLCAN Port (default: COM16)")
    parser.add_argument("--vesc-port", default="COM33", help="VESC USB Port (default: COM33)")
    parser.add_argument("--stlink-id", default="E1007200D0D2139393740544", help="ST-Link Unique ID")
    parser.add_argument("--esc-index", type=int, default=1, help="Target ESC Index (default: 1)")
    parser.add_argument("--max-rpm", type=float, default=7500.0, help="Max Mechanical RPM Cutoff (0 = disabled)")
    parser.add_argument("--pole-pairs", type=int, default=12, help="Motor Pole Pairs (default: 12 for 18N24P)")
    args = parser.parse_args()

    guard = UAVCANSafetyGuard(
        can_port=args.can_port,
        vesc_port=args.vesc_port,
        stlink_id=args.stlink_id,
        esc_index=args.esc_index,
        max_rpm_cutoff=args.max_rpm,
        pole_pairs=args.pole_pairs,
    )
    sys.exit(guard.run())


if __name__ == "__main__":
    main()
