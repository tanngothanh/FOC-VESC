import sys
import json
import pytest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from harness.acoustic_quality_verifier import AcousticQualityVerifier

def test_acoustic_verifier_golden_baseline_exists():
    verifier = AcousticQualityVerifier()
    assert verifier.baseline_spec_path.exists(), "Golden Baseline file must exist"
    
    with open(verifier.baseline_spec_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    
    assert "phases" in data
    assert len(data["phases"]) == 5
    assert data["foc_parameters"]["foc_f_zv"] in (25000.0, 35000.0)
    assert data["foc_parameters"]["foc_dt_us"] == 0.000

def test_verify_clean_run_passes():
    verifier = AcousticQualityVerifier()
    # Simulated steady state clean run telemetry
    clean_telem = [
        {"actual_rpm": 2500 + (i % 3), "current_mot": 0.25 + 0.01 * (i % 2), "duty_pct": 20.0, "fault": "FAULT_CODE_NONE"}
        for i in range(50)
    ]
    report = verifier.verify_run_against_baseline(clean_telem, control_mode="UAVCAN")
    assert report["status"] == "PASS"
    assert report["overall_score"] >= 90.0

def test_verify_jerking_run_fails():
    verifier = AcousticQualityVerifier()
    # Simulated jerky run with sudden 800 RPM drops and huge current spikes
    jerky_telem = [
        {"actual_rpm": 2500 if i % 5 != 0 else 1400, "current_mot": 0.25 if i % 5 != 0 else 3.50, "duty_pct": 20.0, "fault": "FAULT_CODE_NONE"}
        for i in range(50)
    ]
    report = verifier.verify_run_against_baseline(jerky_telem, control_mode="UAVCAN")
    assert report["metrics"]["jerk_events_count"] > 0
    assert report["overall_score"] < 70.0
    assert "REJECT_JERK" in report["rating"]
