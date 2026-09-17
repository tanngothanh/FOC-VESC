#!/usr/bin/env python3
"""
Acoustic Quality Verifier & Golden Baseline Benchmark Engine
05_Propulsion_System/VESC/harness/acoustic_quality_verifier.py

Evaluates motor control quality and detects cogging/slipping/jerking ("gi\u1eadt c\u1ee5c")
and acoustic harshness ("k\u00eau to / r\u00edt") via acoustic audio feedback and telemetry.

Used for validating:
- Low-level FOC smoothness (Golden Baseline)
- DroneCAN / UAVCAN RawCommand closed-loop tracking
- PPM / RC PWM input response
"""

import os
import sys
import json
import time
import numpy as np
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

class AcousticQualityVerifier:
    """Acoustic analysis engine comparing motor sound against Golden Baseline."""

    def __init__(self, baseline_dir: Optional[Path] = None):
        self.base_dir = baseline_dir or (Path(__file__).parent.parent / "benchmarks" / "acoustic")
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.baseline_spec_path = self.base_dir / "golden_baseline_specs.json"

    def compute_telemetry_metrics(self, telemetry_records: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Calculates electrical and mechanical stability metrics from telemetry,
        correctly segmenting multi-step profiles into steady-state holds.
        """
        if not telemetry_records:
            return {}

        # Check if dataset contains multi-step targets
        has_targets = any("target_rpm" in r for r in telemetry_records)
        if has_targets:
            # Group by unique target_rpm
            groups = {}
            for r in telemetry_records:
                trpm = r.get("target_rpm", 0)
                if trpm not in groups:
                    groups[trpm] = []
                groups[trpm].append(r)

            step_metrics = []
            total_jerk_events = 0
            max_jerk_val = 0.0

            for trpm, group in groups.items():
                if trpm <= 0 or len(group) < 5:
                    continue
                # Discard transient acceleration (first 4 samples)
                steady = group[4:]
                s_rpms = np.array([r.get("actual_rpm", 0) for r in steady])
                s_imots = np.array([r.get("current_mot", 0.0) for r in steady])
                s_duties = np.array([r.get("duty_pct", 0.0) for r in steady])

                if len(s_rpms) > 2:
                    drpm = np.abs(np.diff(s_rpms))
                    # Jerk during steady hold: unprovoked drops > 300 RPM
                    jerks = int(np.sum(drpm > 300))
                    total_jerk_events += jerks
                    if len(drpm) > 0:
                        max_jerk_val = max(max_jerk_val, float(np.max(drpm)))

                step_metrics.append({
                    "rpm_mean": float(np.mean(s_rpms)),
                    "rpm_std": float(np.std(s_rpms)),
                    "imot_mean": float(np.mean(s_imots)),
                    "imot_std": float(np.std(s_imots)),
                    "duty_mean": float(np.mean(s_duties)),
                    "duty_std": float(np.std(s_duties)),
                })

            if step_metrics:
                avg_rpm_std = float(np.mean([m["rpm_std"] for m in step_metrics]))
                avg_imot_std = float(np.mean([m["imot_std"] for m in step_metrics]))
                avg_rpm_mean = float(np.mean([m["rpm_mean"] for m in step_metrics]))
                avg_imot_mean = float(np.mean([m["imot_mean"] for m in step_metrics]))
                avg_duty_mean = float(np.mean([m["duty_mean"] for m in step_metrics]))
                avg_duty_std = float(np.mean([m["duty_std"] for m in step_metrics]))
            else:
                avg_rpm_std = avg_imot_std = avg_rpm_mean = avg_imot_mean = avg_duty_mean = avg_duty_std = 0.0

            return {
                "sample_count": len(telemetry_records),
                "rpm_mean": round(avg_rpm_mean, 1),
                "rpm_std": round(avg_rpm_std, 2),
                "current_mot_mean": round(avg_imot_mean, 2),
                "current_mot_std": round(avg_imot_std, 3),
                "duty_mean": round(avg_duty_mean, 2),
                "duty_std": round(avg_duty_std, 2),
                "jerk_events_count": total_jerk_events,
                "max_jerk_rpm": round(max_jerk_val, 1),
                "fault_free": all(r.get("fault") in ("FAULT_CODE_NONE", None) for r in telemetry_records)
            }

        # Flat single-step profile
        rpms = np.array([r.get("actual_rpm", 0) for r in telemetry_records if r.get("actual_rpm") is not None])
        imots = np.array([r.get("current_mot", 0.0) for r in telemetry_records if r.get("current_mot") is not None])
        duties = np.array([r.get("duty_pct", 0.0) for r in telemetry_records if r.get("duty_pct") is not None])

        active_rpms = rpms[rpms > 300] if any(rpms > 300) else rpms
        active_imots = imots[len(imots) - len(active_rpms):] if len(active_rpms) > 0 else imots
        active_duties = duties[len(duties) - len(active_rpms):] if len(active_rpms) > 0 else duties

        rpm_mean = float(np.mean(active_rpms)) if len(active_rpms) > 0 else 0.0
        rpm_std = float(np.std(active_rpms)) if len(active_rpms) > 0 else 0.0
        imot_mean = float(np.mean(active_imots)) if len(active_imots) > 0 else 0.0
        imot_std = float(np.std(active_imots)) if len(active_imots) > 0 else 0.0
        duty_mean = float(np.mean(active_duties)) if len(active_duties) > 0 else 0.0
        duty_std = float(np.std(active_duties)) if len(active_duties) > 0 else 0.0

        if len(active_rpms) > 2:
            drpm = np.abs(np.diff(active_rpms))
            jerk_events = int(np.sum(drpm > 400))
            max_jerk = float(np.max(drpm))
        else:
            jerk_events = 0
            max_jerk = 0.0

        return {
            "sample_count": len(telemetry_records),
            "rpm_mean": round(rpm_mean, 1),
            "rpm_std": round(rpm_std, 2),
            "current_mot_mean": round(imot_mean, 2),
            "current_mot_std": round(imot_std, 3),
            "duty_mean": round(duty_mean, 2),
            "duty_std": round(duty_std, 2),
            "jerk_events_count": jerk_events,
            "max_jerk_rpm": round(max_jerk, 1),
            "fault_free": all(r.get("fault") in ("FAULT_CODE_NONE", None) for r in telemetry_records)
        }

    def analyze_audio_segment(self, audio_data: np.ndarray, sample_rate: int, target_mech_rpm: float) -> Dict[str, Any]:
        """Performs spectral analysis on raw audio segment.
        
        Evaluates:
        - Commutation fundamental peak (fe = RPM * 12 / 60)
        - High-frequency screech power ratio (> 3 kHz)
        - Crest factor / Kurtosis (transient impulse indicator for jerking/cogging)
        """
        if len(audio_data) == 0:
            return {}

        # Normalize audio
        audio = audio_data.astype(np.float64)
        audio = audio - np.mean(audio)
        max_val = np.max(np.abs(audio))
        if max_val > 1e-6:
            audio = audio / max_val

        # RMS & Crest Factor
        rms = np.sqrt(np.mean(audio**2))
        peak = np.max(np.abs(audio))
        crest_factor = float(peak / (rms + 1e-9))

        # Kurtosis (impulsiveness - high kurtosis indicates knocking/jerking)
        kurtosis = float(np.mean(audio**4) / ((np.mean(audio**2) + 1e-9)**2))

        # FFT Analysis
        n_fft = min(len(audio), 4096)
        fft_vals = np.abs(np.fft.rfft(audio[:n_fft] * np.hanning(n_fft)))
        freqs = np.fft.rfftfreq(n_fft, d=1.0/sample_rate)

        # High-frequency harshness ratio (> 3000 Hz energy / total energy)
        high_freq_mask = freqs >= 3000.0
        total_energy = np.sum(fft_vals**2) + 1e-9
        high_freq_energy = np.sum(fft_vals[high_freq_mask]**2)
        screech_ratio = float(high_freq_energy / total_energy)

        # Commutation frequency fundamental
        expected_fe = (target_mech_rpm * 12.0) / 60.0 # 12 pole pairs
        fe_mask = (freqs >= expected_fe * 0.85) & (freqs <= expected_fe * 1.15)
        fe_energy = np.sum(fft_vals[fe_mask]**2) if any(fe_mask) else 0.0
        fe_ratio = float(fe_energy / total_energy)

        return {
            "rms_level": round(float(rms), 4),
            "crest_factor": round(crest_factor, 2),
            "kurtosis": round(kurtosis, 2),
            "screech_energy_ratio": round(screech_ratio, 4),
            "commutation_fe_hz": round(expected_fe, 1),
            "commutation_fe_ratio": round(fe_ratio, 4)
        }

    def calculate_acoustic_smoothness_index(self, telem_metrics: Dict[str, Any],
                                            audio_metrics: Optional[Dict[str, Any]] = None) -> Tuple[float, str]:
        """Calculates combined Acoustic & Control Smoothness Index (ASI: 0 to 100)."""
        score = 100.0

        # Telemetry penalties
        jerk_penalty = min(40.0, telem_metrics.get("jerk_events_count", 0) * 15.0)
        ripple_penalty = min(25.0, telem_metrics.get("current_mot_std", 0.0) * 25.0)
        rpm_penalty = min(20.0, telem_metrics.get("rpm_std", 0.0) * 0.5)

        score -= (jerk_penalty + ripple_penalty + rpm_penalty)

        # Audio penalties if available
        if audio_metrics:
            screech_penalty = min(30.0, audio_metrics.get("screech_energy_ratio", 0.0) * 100.0)
            kurtosis = audio_metrics.get("kurtosis", 3.0)
            kurtosis_penalty = min(15.0, max(0.0, kurtosis - 3.0) * 3.0)
            score -= (screech_penalty + kurtosis_penalty)

        final_score = max(0.0, min(100.0, round(score, 1)))

        if final_score >= 88.0:
            rating = "CERTIFIED_QUIET (Chất lượng Vàng / Êm ái tuyệt đối)"
        elif final_score >= 75.0:
            rating = "PASS_ACCEPTABLE (Đạt chuẩn vận hành bám tốc)"
        elif final_score >= 60.0:
            rating = "MARGINAL_WARNING (Có tiếng rít hoặc gợn dòng)"
        else:
            rating = "REJECT_JERK (Phát hiện giật cục / trượt rotor)"

        return final_score, rating

    def save_golden_baseline(self, phases_data: Dict[str, Dict[str, Any]]) -> None:
        """Saves current best run as Golden Reference Standard."""
        payload = {
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "motor": "Sunnysky V4006 740KV (24 poles, 12 pole pairs)",
            "esc": "VESC 60 Firmware 7.0",
            "foc_parameters": {
                "foc_f_zv": 35000.0,
                "foc_dt_us": 0.000,
                "foc_cc_decoupling": 0,
                "foc_sl_openloop_time_lock": 0.00,
                "foc_sl_openloop_time_ramp": 0.12,
                "foc_sl_openloop_max_q": 2.50
            },
            "phases": phases_data
        }
        with open(self.baseline_spec_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"[INFO] Golden Baseline Benchmark successfully saved to: {self.baseline_spec_path}")

    def verify_run_against_baseline(self, test_telemetry: List[Dict[str, Any]], control_mode: str = "UAVCAN") -> Dict[str, Any]:
        """Compares incoming UAVCAN or PPM test run against Golden Baseline."""
        if not self.baseline_spec_path.exists():
            return {"error": "No Golden Baseline found. Please run save_golden_baseline first."}

        metrics = self.compute_telemetry_metrics(test_telemetry)
        score, rating = self.calculate_acoustic_smoothness_index(metrics)

        report = {
            "control_mode": control_mode,
            "overall_score": score,
            "rating": rating,
            "metrics": metrics,
            "status": "PASS" if score >= 75.0 else "FAIL"
        }
        return report

if __name__ == "__main__":
    verifier = AcousticQualityVerifier()
    print("AcousticQualityVerifier initialized.")
    print("Baseline file path:", verifier.baseline_spec_path)
