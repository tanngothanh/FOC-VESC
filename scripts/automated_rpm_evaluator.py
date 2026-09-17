#!/usr/bin/env python3
"""
Automated RPM Control Quality & Acoustic Verifier (PROP-SUB-05).
Executes Golden Baseline 5-Phase RPM Sweep and Deceleration Anti-Hang Test:
1. Connects to VESC on COM33 and attaches ST-Link probe for hardware safety.
2. Isolates CAN bus (RULE-CAN-PPM-BENCH-ISOLATION) for deterministic UART execution.
3. Steps through 1000 -> 2250 -> 3500 -> 4750 -> 6000 Mech RPM.
4. Tests immediate deceleration (6000 -> 1000 -> 0 RPM) to verify ZERO throttle hanging.
5. Computes Acoustic Smoothness Index (ASI) and telemetry stability metrics.
6. Saves Golden Baseline & Acceptance Report to benchmarks/acoustic/.
7. Restores dual-mode flight configuration (can_mode=1 UAVCAN, uavcan_raw_mode=3 RPM).
"""

import os
import sys
import time
import json
import struct
import argparse
import threading
import numpy as np
import serial
from pathlib import Path

# Force UTF-8 on Windows console stdout
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

VESC_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(VESC_DIR))

from lib.vesc_interface import VESCInterface
from lib.vesc_protocol import encode_comm_set_rpm, encode_comm_set_current, encode_comm_set_duty, encode_comm_get_values, decode_telemetry, encode_packet
from harness.acoustic_quality_verifier import AcousticQualityVerifier
from scripts.enable_vesc_uavcan_and_pwm import get_appconf, set_appconf, reboot_vesc

try:
    from pyocd.core.helpers import ConnectHelper
    PYOCD_AVAILABLE = True
except ImportError:
    PYOCD_AVAILABLE = False


class AutomatedRPMEvaluator:
    def __init__(
        self,
        vesc_port: str = "COM33",
        baud: int = 115200,
        stlink_id: str = "E1007200D0D2139393740544",
        pole_pairs: int = 12,
        max_rpm_cutoff: float = 6600.0,
        max_current_cutoff: float = 2.5,
    ):
        self.vesc_port = vesc_port
        self.baud = baud
        self.stlink_id = stlink_id
        self.pole_pairs = pole_pairs
        self.max_rpm_cutoff = max_rpm_cutoff
        self.max_current_cutoff = max_current_cutoff

        self.ser = None
        self.stlink_session = None
        self.stlink_target = None
        self.running = False
        self.emergency_stopped = False
        self.stop_reason = ""

        self.telemetry_records = []
        self.current_target_mech_rpm = 0.0

    def init_stlink(self) -> bool:
        """Initializes ST-Link in attach mode for hardware breakpoint / halt failsafe."""
        if not PYOCD_AVAILABLE:
            print("[ST-Link] pyocd not installed, skipping hardware probe.")
            return False
        try:
            print(f"[ST-Link] Attaching to probe {self.stlink_id}...")
            self.stlink_session = ConnectHelper.session_with_chosen_probe(
                unique_id=self.stlink_id,
                target_override="cortex_m",
                options={"connect_mode": "attach", "resume_on_disconnect": True}
            )
            self.stlink_session.open()
            self.stlink_target = self.stlink_session.target
            state = self.stlink_target.get_state().name
            if state != "RUNNING":
                self.stlink_target.resume()
            print(f"[ST-Link] Attached! MCU Core State: {self.stlink_target.get_state().name}")
            return True
        except Exception as e:
            print(f"[ST-Link] Attach error ({e}), relying on serial failsafe.")
            self.stlink_session = None
            self.stlink_target = None
            return False

    def trigger_emergency_stop(self, reason: str):
        """Multi-layer hardware emergency stop."""
        self.emergency_stopped = True
        self.running = False
        self.stop_reason = reason

        print(f"\n{'='*70}")
        print(f"[!] [EMERGENCY STOP TRIGGERED] [!]")
        print(f"REASON: {reason}")
        print(f"{'='*70}")

        # Layer 1: Zero current / zero duty burst (COMM_SET_CURRENT(0.0) calls mc_interface_release_motor())
        if self.ser and self.ser.is_open:
            for _ in range(15):
                try:
                    self.ser.write(encode_packet(encode_comm_set_current(0.0))) # current 0 -> release motor gates
                    self.ser.write(encode_packet(encode_comm_set_duty(0.0)))    # duty 0
                except Exception:
                    pass
                time.sleep(0.005)
            print("[VESC Serial] Sent zero command kill burst (Current=0A, Duty=0%).")

        # Layer 2: ST-Link hardware kill (TIM1->BDTR MOE=0, TIM1->CR1 CEN=0, halt core)
        time.sleep(0.02)
        if self.stlink_target:
            try:
                # Force TIM1->BDTR MOE (bit 15) to 0 to disable all gate drive outputs in silicon
                self.stlink_target.write32(0x40010044, 0x00000000)
                # Force TIM1->CR1 CEN (bit 0) to 0
                self.stlink_target.write32(0x40010000, 0x00000000)
                # Halt ARM Cortex-M4 core
                self.stlink_target.halt()
                print(f"[ST-Link] Hardware PWM KILLED (TIM1->BDTR=0, TIM1->CR1=0) and MCU Core HALTED!")
            except Exception as e:
                print(f"[ST-Link] Halt error: {e}")

    def set_bench_isolation_mode(self, isolate: bool = True):
        """Implements RULE-CAN-PPM-BENCH-ISOLATION:
        isolate=True:  can_mode=0 (Disabled), app_to_use=0 (None) for deterministic UART testing.
        isolate=False: can_mode=1 (UAVCAN), uavcan_raw_mode=3 (RPM), uavcan_raw_rpm_max=72000.0.
        """
        print(f"\n[BENCH ISOLATION] Setting isolation mode: {'ISOLATED (CAN=0)' if isolate else 'FLIGHT DUAL-MODE (CAN=1 UAVCAN)'}...")
        ser = serial.Serial(self.vesc_port, self.baud, timeout=1.0)
        appconf = bytearray(get_appconf(ser))
        if not appconf or len(appconf) < 50:
            print("[ERROR] Failed to read APPCONF!")
            ser.close()
            return False

        if isolate:
            appconf[23] = 0  # CAN_MODE_DISABLED
            appconf[42] = 0  # APP_NONE
        else:
            appconf[23] = 1  # CAN_MODE_UAVCAN
            appconf[24] = 1  # uavcan_esc_index = 1
            appconf[25] = 3  # UAVCAN_RAW_MODE_RPM
            appconf[26:30] = struct.pack(">f", 72000.0) # 72,000 ERPM = 6,000 Mech RPM
            appconf[42] = 4  # APP_PPM_UART dual mode

        set_appconf(ser, bytes(appconf))
        time.sleep(0.3)
        reboot_vesc(ser)
        ser.close()
        time.sleep(2.0)

        # Reconnect
        reconnected = False
        for _ in range(10):
            try:
                ser = serial.Serial(self.vesc_port, self.baud, timeout=1.0)
                if ser.is_open:
                    reconnected = True
                    break
            except Exception:
                time.sleep(0.5)

        if reconnected:
            check_conf = get_appconf(ser)
            ser.close()
            c_mode = check_conf[23] if check_conf else -1
            print(f"[BENCH ISOLATION] Reconnected! Active can_mode = {c_mode} ({'PASS' if (c_mode == 0 if isolate else c_mode == 1) else 'FAIL'})")
            return True
        else:
            print("[ERROR] Reconnection failed after isolation toggle!")
            return False

    def query_telemetry(self) -> dict:
        """Queries telemetry via COMM_GET_VALUES (cmd 4)."""
        if not self.ser or not self.ser.is_open:
            return {}
        try:
            self.ser.reset_input_buffer()
            pkt = encode_packet(encode_comm_get_values())
            self.ser.write(pkt)
            time.sleep(0.02)
            resp = self.ser.read(256)
            if len(resp) < 60:
                return {}

            # Frame unpack
            i = 0
            while i < len(resp):
                if resp[i] == 2:
                    plen = resp[i+1]
                    pdata = resp[i+2:i+2+plen]
                    if len(pdata) > 0 and pdata[0] == 4:
                        telem = decode_telemetry(pdata)
                        return telem
                    i += 2 + plen + 3
                elif resp[i] == 3:
                    plen = struct.unpack('>H', resp[i+1:i+3])[0]
                    pdata = resp[i+3:i+3+plen]
                    if len(pdata) > 0 and pdata[0] == 4:
                        telem = decode_telemetry(pdata)
                        return telem
                    i += 3 + plen + 3
                else:
                    i += 1
        except Exception:
            pass
        return {}

    def run_benchmark(self) -> dict:
        """Executes full 5-phase sweep and anti-hang deceleration verification."""
        print("\n" + "="*70)
        print("   STARTING AUTOMATED V4006 RPM CONTROL & ACOUSTIC EVALUATION")
        print("="*70)

        # Step 1: Isolate CAN bus for deterministic UART test
        if not self.set_bench_isolation_mode(isolate=True):
            return {"status": "FAIL", "reason": "Failed to set bench isolation mode"}

        # Step 2: Init ST-Link
        self.init_stlink()

        # Step 3: Open serial port for test
        self.ser = serial.Serial(self.vesc_port, self.baud, timeout=0.1)
        self.running = True

        # Check initial state
        initial_telem = self.query_telemetry()
        v_in = initial_telem.get("v_in", 20.3)
        print(f"[HARDWARE READY] V_in: {v_in:.2f} V, Initial RPM: {initial_telem.get('rpm', 0)/self.pole_pairs:.0f}")

        # Define 5 Golden Baseline Phases + Anti-Hang Test
        test_phases = [
            ("Phase_1_Breakaway_1000RPM", 1000.0, 3.5),
            ("Phase_2_LowCruise_2250RPM", 2250.0, 3.5),
            ("Phase_3_MidCruise_3500RPM", 3500.0, 3.5),
            ("Phase_4_HighCruise_4750RPM", 4750.0, 3.5),
            ("Phase_5_MaxCeiling_6000RPM", 6000.0, 4.0),
        ]

        phases_data = {}
        all_records = []

        try:
            for phase_name, target_rpm, duration in test_phases:
                if self.emergency_stopped:
                    break

                target_erpm = int(target_rpm * self.pole_pairs)
                self.current_target_mech_rpm = target_rpm
                print(f"\n>>> [TESTING] {phase_name}: Target = {target_rpm:.0f} Mech RPM ({target_erpm} ERPM) for {duration}s...")

                phase_records = []
                t_start = time.time()

                while time.time() - t_start < duration:
                    if self.emergency_stopped:
                        break

                    # Send RPM command at ~30 Hz
                    pkt = encode_packet(encode_comm_set_rpm(target_erpm))
                    self.ser.write(pkt)

                    # Query telemetry
                    t = self.query_telemetry()
                    if t:
                        mech_rpm = t.get("rpm", 0) / float(self.pole_pairs)
                        duty_pct = t.get("duty_now", 0.0) * 100.0
                        i_mot = t.get("current_motor", 0.0)
                        vin = t.get("v_in", 0.0)
                        fault = t.get("fault_str", "FAULT_CODE_NONE")

                        rec = {
                            "timestamp": time.time(),
                            "target_rpm": target_rpm,
                            "actual_rpm": mech_rpm,
                            "current_mot": i_mot,
                            "duty_pct": duty_pct,
                            "v_in": vin,
                            "fault": fault
                        }
                        all_records.append(rec)

                        # Steady-state evaluation: record after acceleration ramp has settled (> 1.0s)
                        if time.time() - t_start > 1.0:
                            phase_records.append(rec)

                        # Print live status
                        diff = mech_rpm - target_rpm
                        sys.stdout.write(
                            f"\r  [RUN] Tgt:{target_rpm:4.0f} | Mech:{mech_rpm:5.0f} RPM | Duty:{duty_pct:4.1f}% | "
                            f"I:{i_mot:4.2f}A | Diff:{diff:+5.0f} RPM "
                        )
                        sys.stdout.flush()

                        # Failsafe checks
                        if mech_rpm > self.max_rpm_cutoff:
                            self.trigger_emergency_stop(f"OVER-SPEED: Actual RPM {mech_rpm:.0f} exceeded {self.max_rpm_cutoff:.0f} RPM")
                            break
                        if abs(i_mot) > self.max_current_cutoff:
                            self.trigger_emergency_stop(f"OVER-CURRENT: Motor current {i_mot:.2f}A exceeded {self.max_current_cutoff:.1f}A")
                            break
                        if t.get("fault_code", 0) != 0:
                            self.trigger_emergency_stop(f"HARDWARE FAULT: {fault}")
                            break

                    time.sleep(0.033) # ~30 Hz

                # Process phase metrics
                verifier = AcousticQualityVerifier()
                metrics = verifier.compute_telemetry_metrics(phase_records)
                score, rating = verifier.calculate_acoustic_smoothness_index(metrics)

                phases_data[phase_name] = {
                    "throttle_pct": round(target_rpm / 6000.0 * 100.0, 1),
                    "target_mech_rpm": target_rpm,
                    "metrics": metrics,
                    "smoothness_score": score,
                    "rating": rating
                }
                print(f"\n  [RESULT] Score: {score:.1f}/100 - {rating}")
                print(f"           RPM Mean: {metrics.get('rpm_mean', 0):.1f}, Std: {metrics.get('rpm_std', 0):.2f}, "
                      f"Duty: {metrics.get('duty_mean', 0):.1f}%, Jerks: {metrics.get('jerk_events_count', 0)}")

            # ================= DECELERATION & ANTI-HANG TEST =================
            if not self.emergency_stopped:
                print("\n" + "="*70)
                print(">>> [TESTING] DECELERATION & ANTI-HANG TEST (Thao tác thử thách treo ga)")
                print("    Verification: Instant throttle cut from full speed (6,000 RPM -> 0 RPM)")
                print("    Mechanism: COMM_SET_CURRENT(0.0) -> mc_interface_release_motor() gating")
                print("="*70)

                print("  [COMMAND] Cutting throttle at 6,000 RPM via COMM_SET_CURRENT(0.0)...")
                stop_cmd_time = time.time()
                stopped_time = None

                while time.time() - stop_cmd_time < 3.0:
                    self.ser.write(encode_packet(encode_comm_set_current(0.0)))
                    t = self.query_telemetry()
                    if t:
                        mech_rpm = t.get("rpm", 0) / float(self.pole_pairs)
                        duty_pct = t.get("duty_now", 0.0) * 100.0
                        sys.stdout.write(
                            f"\r  [DECEL] t={time.time()-stop_cmd_time:4.2f}s | Mech:{mech_rpm:5.0f} RPM | Duty:{duty_pct:4.1f}% "
                        )
                        sys.stdout.flush()

                        if abs(mech_rpm) < 50.0 and stopped_time is None:
                            stopped_time = time.time()
                            break
                    time.sleep(0.033)

                decel_duration = (stopped_time - stop_cmd_time) if stopped_time else 999.0
                print(f"\n  [DECEL RESULT] Time to reach 0 RPM from 6,000 RPM: {decel_duration:.2f} s")
                hang_detected = (stopped_time is None) or (decel_duration > 1.5)

                if hang_detected:
                    print("  [FAIL] Throttle hanging detected! Motor failed to stop within 1.5s.")
                else:
                    print(f"  [PASS] ZERO THROTTLE HANG! Full speed stop completed in {decel_duration:.2f}s (< 1.5s benchmark).")

        finally:
            # Safe motor stop
            if self.ser and self.ser.is_open:
                try:
                    for _ in range(10):
                        self.ser.write(encode_packet(encode_comm_set_current(0.0)))
                        self.ser.write(encode_packet(encode_comm_set_duty(0.0)))
                        time.sleep(0.01)
                    self.ser.close()
                except Exception:
                    pass

            if self.stlink_session:
                try:
                    if not self.emergency_stopped and self.stlink_target:
                        self.stlink_target.resume()
                    self.stlink_session.close()
                except Exception:
                    pass

        # Step 4: Evaluate overall run against baseline
        verifier = AcousticQualityVerifier()
        phase_scores = [p["smoothness_score"] for p in phases_data.values() if "smoothness_score" in p]
        overall_score = round(float(np.mean(phase_scores)), 1) if phase_scores else 0.0

        if overall_score >= 88.0:
            overall_rating = "CERTIFIED_QUIET (Chất lượng Vàng / Êm ái tuyệt đối)"
        elif overall_score >= 75.0:
            overall_rating = "PASS_ACCEPTABLE (Đạt chuẩn vận hành bám tốc)"
        else:
            overall_rating = "REJECT_JERK (Phát hiện giật cục / trượt rotor)"

        overall_metrics = verifier.compute_telemetry_metrics(all_records)

        # Save Golden Baseline & Acceptance Report
        verifier.save_golden_baseline(phases_data)

        report_path = VESC_DIR / "benchmarks" / "acoustic" / "RPM_LOOP_ACCEPTANCE_REPORT.json"
        report_data = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "status": "PASS" if (overall_score >= 88.0 and not self.emergency_stopped and not hang_detected) else "FAIL",
            "overall_score": overall_score,
            "overall_rating": overall_rating,
            "decel_anti_hang_time_s": decel_duration,
            "anti_hang_passed": not hang_detected,
            "phases": phases_data,
            "overall_metrics": overall_metrics
        }
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report_data, f, indent=2)
        print(f"\n[REPORT SAVED] Acceptance Report saved to: {report_path}")

        # Step 5: Restore dual-mode flight configuration (can_mode=1 UAVCAN)
        print("\n[RESTORING DUAL-MODE FLIGHT CONFIGURATION]")
        self.set_bench_isolation_mode(isolate=False)

        return report_data


def main():
    parser = argparse.ArgumentParser(description="Automated V4006 RPM Quality & Anti-Hang Evaluator")
    parser.add_argument("--port", default="COM33", help="VESC USB COM Port (default: COM33)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    evaluator = AutomatedRPMEvaluator(vesc_port=args.port, baud=args.baud)
    report = evaluator.run_benchmark()

    print("\n" + "="*70)
    print("                FINAL ACCEPTANCE SUMMARY")
    print("="*70)
    print(f"  Status:               {report.get('status')}")
    print(f"  Overall Score:        {report.get('overall_score')}/100")
    print(f"  Acoustic Rating:      {report.get('overall_rating')}")
    print(f"  Decel Stop Time:      {report.get('decel_anti_hang_time_s', 0):.2f} s")
    print(f"  Anti-Hang Result:     {'PASS (No throttle hang)' if report.get('anti_hang_passed') else 'FAIL'}")
    print("="*70)

    sys.exit(0 if report.get("status") == "PASS" else 1)


if __name__ == "__main__":
    main()
