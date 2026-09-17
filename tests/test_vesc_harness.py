"""Comprehensive Pytest Suite for VESC Agent Harness (PROP-SUB-05).

Covers:
1. VESCConfigEngine mathematical modeling, duty clamp, and safety validators.
2. VESCAutoTuner modes (Bench, Flight Nominal, Flight Maxload, HV 800V).
3. MockVESCHardware and EvalHarness aerospace verification suite.
4. VESCLearningLoop experiment ledger, benchmarking, and parameter recommendations.
5. HubConnector OpenAPI schemas and tool dispatching for LangGraph/n8n.
6. VESCMotorEngineerHarness master facade.
"""

import sys
import json
import math
import tempfile
import pytest
from pathlib import Path

# Add VESC root to sys.path
VESC_DIR = Path(__file__).resolve().parent.parent
if str(VESC_DIR) not in sys.path:
    sys.path.insert(0, str(VESC_DIR))

from harness.vesc_config_engine import (
    MotorSpec,
    HardwareLimits,
    OperatingEnvelope,
    VESCConfigEngine,
    SafetyViolationError
)
from harness.auto_tuner import (
    VESCAutoTuner,
    OperatingMode,
    VESCHardwareTier,
    MOTOR_LIBRARY,
    HARDWARE_TIERS
)
from harness.eval_harness import (
    EvalHarness,
    MockVESCHardware,
    EvaluationReport
)
from harness.learning_loop import (
    VESCLearningLoop,
    ExperimentRecord,
    TuningRecommendation
)
from harness.hub_connector import HubConnector
from harness.agent_interface import VESCMotorEngineerHarness


# =====================================================================
# 1. VESCConfigEngine Tests
# =====================================================================
class TestVESCConfigEngine:
    def test_motor_spec_flux_linkage(self):
        motor = MotorSpec(kv=740.0, poles=24)
        assert motor.pole_pairs == 12
        # lambda = 60 / (sqrt(3) * 2 * pi * 740 * 12) = 0.00062087...
        expected = 60.0 / (math.sqrt(3.0) * 2.0 * math.pi * 740.0 * 12.0)
        assert pytest.approx(motor.flux_linkage_wb, rel=1e-4) == expected
        assert round(motor.flux_linkage_wb, 8) == 0.00062087

    def test_duty_ceiling_calculation(self):
        motor = MotorSpec(kv=740.0, poles=24)
        hw = HardwareLimits(v_bus_nominal=20.5)
        env = OperatingEnvelope(max_mech_rpm=6000.0)
        duty = VESCConfigEngine.calculate_duty_ceiling(motor, hw, env, margin=1.06)
        # (6000 * 1.06) / (20.5 * 740) = 6360 / 15170 = 0.4192
        assert 0.40 <= duty <= 0.50

    def test_generate_tuning_profile_structure(self):
        motor = MotorSpec(kv=740.0, poles=24, r_phase_ohm=0.047, l_phase_h=1.413e-5)
        hw = HardwareLimits(v_bus_nominal=20.5, i_dc_max=27.5, i_phase_max=35.0)
        env = OperatingEnvelope(min_mech_rpm=1000.0, max_mech_rpm=6000.0)
        profile = VESCConfigEngine.generate_tuning_profile(motor, hw, env, "test_prof")

        assert profile["metadata"]["profile_name"] == "test_prof"
        assert profile["motor_parameters"]["kv"] == 740.0
        assert profile["motor_parameters"]["pole_pairs"] == 12
        assert profile["current_limits"]["l_in_current_max"] == 27.5
        assert profile["current_limits"]["l_in_current_min"] == 0.0
        assert profile["current_limits"]["l_current_max"] == 35.0
        assert profile["speed_limits"]["l_max_erpm"] == 72000.0
        assert profile["speed_limits"]["l_min_erpm"] == 0.0
        assert profile["speed_limits"]["s_pid_kd"] == 0.0
        assert profile["foc_settings"]["foc_sensor_mode"] == 0

    def test_validation_passes_valid_profile(self):
        motor = MotorSpec()
        hw = HardwareLimits()
        env = OperatingEnvelope()
        profile = VESCConfigEngine.generate_tuning_profile(motor, hw, env)
        violations = VESCConfigEngine.validate_configuration(profile, hw)
        assert len(violations) == 0

    def test_validation_catches_anti_hfi_violation(self):
        motor = MotorSpec()
        hw = HardwareLimits()
        env = OperatingEnvelope()
        profile = VESCConfigEngine.generate_tuning_profile(motor, hw, env)
        profile["foc_settings"]["foc_sensor_mode"] = 3 # HFI
        violations = VESCConfigEngine.validate_configuration(profile, hw)
        assert any("HFI" in v for v in violations)

    def test_validation_catches_anti_backfeed_violation(self):
        motor = MotorSpec()
        hw = HardwareLimits()
        env = OperatingEnvelope()
        profile = VESCConfigEngine.generate_tuning_profile(motor, hw, env)
        profile["current_limits"]["l_in_current_min"] = -5.0 # Backfeed
        violations = VESCConfigEngine.validate_configuration(profile, hw)
        assert any("l_in_current_min" in v for v in violations)

    def test_validation_catches_unidirectional_violation(self):
        motor = MotorSpec()
        hw = HardwareLimits()
        env = OperatingEnvelope()
        profile = VESCConfigEngine.generate_tuning_profile(motor, hw, env)
        profile["speed_limits"]["l_min_erpm"] = -12000.0 # Reverse
        violations = VESCConfigEngine.validate_configuration(profile, hw)
        assert any("l_min_erpm" in v for v in violations)

    def test_validation_catches_sensorless_kd_violation(self):
        motor = MotorSpec()
        hw = HardwareLimits()
        env = OperatingEnvelope()
        profile = VESCConfigEngine.generate_tuning_profile(motor, hw, env)
        profile["speed_limits"]["s_pid_kd"] = 0.001 # Noise amplifier
        violations = VESCConfigEngine.validate_configuration(profile, hw)
        assert any("s_pid_kd" in v for v in violations)

    def test_validation_catches_watchdog_timeout_violation(self):
        motor = MotorSpec()
        hw = HardwareLimits()
        env = OperatingEnvelope()
        profile = VESCConfigEngine.generate_tuning_profile(motor, hw, env)
        profile["can_uavcan"]["timeout_msec"] = 1000 # Too long
        violations = VESCConfigEngine.validate_configuration(profile, hw)
        assert any("timeout_msec" in v for v in violations)


# =====================================================================
# 2. VESCAutoTuner Tests
# =====================================================================
class TestVESCAutoTuner:
    def test_tuner_modes(self):
        tuner = VESCAutoTuner()

        bench_prof = tuner.tune_for_mode(OperatingMode.BENCH_SAFE)
        assert bench_prof["current_limits"]["l_in_current_max"] <= 5.0
        assert bench_prof["current_limits"]["l_current_min"] == 0.0 # Freewheel

        flight_nom = tuner.tune_for_mode(OperatingMode.FLIGHT_NOMINAL)
        assert flight_nom["current_limits"]["l_in_current_max"] <= 22.0
        assert flight_nom["current_limits"]["l_current_max"] <= 25.0

        flight_max = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)
        assert flight_max["current_limits"]["l_in_current_max"] == 27.5
        assert flight_max["current_limits"]["l_current_max"] == 35.0
        assert flight_max["current_limits"]["l_watt_max"] <= 560.0

    def test_tuner_heavy_lift_and_hv(self):
        motor = MOTOR_LIBRARY["TMOTOR_U15_80KV"]
        hw = HARDWARE_TIERS[VESCHardwareTier.VESC_HV_800V]
        env = OperatingEnvelope(min_mech_rpm=200.0, max_mech_rpm=2400.0)

        tuner = VESCAutoTuner(motor=motor, hardware=hw, envelope=env)
        prof = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)

        assert prof["motor_parameters"]["kv"] == 80.0
        assert prof["motor_parameters"]["pole_pairs"] == 21
        assert prof["current_limits"]["l_in_current_max"] <= 100.0
        assert prof["current_limits"]["l_current_max"] <= 120.0
        assert prof["speed_limits"]["l_max_erpm"] == 2400.0 * 21


# =====================================================================
# 3. MockHardware and EvalHarness Tests
# =====================================================================
class TestMockHardwareAndEvalHarness:
    def test_mock_hardware_step_and_telemetry(self):
        tuner = VESCAutoTuner()
        config = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)
        mock = MockVESCHardware(config)

        assert mock.mech_rpm == 0.0
        assert mock.duty_now == 0.0
        assert mock.motor_current == 0.0

        # Step at standstill
        mock.step(0.02)
        telem = mock.get_telemetry()
        assert telem["rpm"] == 0
        assert telem["duty_now"] == 0.0

        # Set RPM and stream steps
        for _ in range(25):
            mock.set_rpm(12000) # 1000 Mech RPM
            mock.step(0.02)
        telem = mock.get_telemetry()
        actual_mech_rpm = telem["rpm"] // 12
        assert 950 <= actual_mech_rpm <= 1050
        assert telem["observer_locked"] is True
        assert telem["duty_now"] > 0.0

    def test_mock_hardware_anti_runaway_clamp(self):
        tuner = VESCAutoTuner()
        config = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)
        mock = MockVESCHardware(config)

        # Command extreme 12,000 Mech RPM
        mock.set_rpm(144000)
        for _ in range(30):
            mock.step(0.02)
        telem = mock.get_telemetry()
        actual_mech_rpm = telem["rpm"] // 12

        # Verify physical duty clamp stopped it at ~6,045 RPM max!
        assert actual_mech_rpm <= 6100
        assert telem["duty_now"] <= config["current_limits"]["l_max_duty"] + 0.01

    def test_mock_hardware_watchdog_shutdown(self):
        tuner = VESCAutoTuner()
        config = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)
        mock = MockVESCHardware(config)

        mock.set_rpm(12000)
        for _ in range(10):
            mock.step(0.02)
        assert mock.gate_enabled is True

        # Simulate timeout by moving last_cmd_time backwards
        mock.last_cmd_time -= 0.300
        mock.step(0.02)
        assert mock.gate_enabled is False
        assert mock.duty_now == 0.0

    def test_eval_harness_full_suite(self):
        tuner = VESCAutoTuner()
        config = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)
        harness = EvalHarness(config=config)
        report = harness.run_full_suite()

        assert report.all_passed is True
        assert report.summary == "LEVEL 1 CERTIFIED"
        assert len(report.metrics) == 6
        for m in report.metrics:
            assert m.passed is True


# =====================================================================
# 4. LearningLoop Tests
# =====================================================================
class TestLearningLoop:
    def test_learning_loop_record_and_summary(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            hist_file = Path(tmpdir) / "test_history.json"
            loop = VESCLearningLoop(history_file=hist_file)

            tuner = VESCAutoTuner()
            config = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)
            harness = EvalHarness(config=config)
            report = harness.run_full_suite()

            record = loop.record_run(config, report)
            assert record.all_passed is True
            assert record.motor_name == "Sunnysky V4006 740KV"
            assert hist_file.exists()

            summary = loop.get_benchmark_summary()
            assert summary["total_runs"] == 1
            assert summary["passed_runs"] == 1
            assert summary["pass_rate_pct"] == 100.0


# =====================================================================
# 5. HubConnector Tests
# =====================================================================
class TestHubConnector:
    def test_schemas(self):
        schemas = HubConnector.get_tool_schemas()
        assert len(schemas) >= 5
        names = [s["name"] for s in schemas]
        assert "vesc_calculate_config" in names
        assert "vesc_validate_config" in names
        assert "vesc_run_eval_suite" in names
        assert "vesc_get_learning_history" in names
        assert "vesc_export_xml" in names

    def test_dispatch_calculate_and_validate(self):
        res = HubConnector.dispatch("vesc_calculate_config", {
            "motor_name": "Sunnysky V4006 740KV",
            "poles": 24,
            "slots": 18,
            "kv": 740.0,
            "r_phase_ohm": 0.047,
            "l_phase_h": 1.413e-5,
            "v_bus_nominal": 20.5,
            "i_dc_max": 27.5,
            "i_motor_max": 35.0,
            "mode": "flight_maxload"
        })
        assert res["status"] == "success"
        profile = res["profile"]

        val_res = HubConnector.dispatch("vesc_validate_config", {"config": profile})
        assert val_res["status"] == "success"
        assert val_res["valid"] is True
        assert len(val_res["violations"]) == 0

    def test_dispatch_eval_suite_and_export(self):
        tuner = VESCAutoTuner()
        config = tuner.tune_for_mode(OperatingMode.FLIGHT_MAXLOAD)

        eval_res = HubConnector.dispatch("vesc_run_eval_suite", {"config": config})
        assert eval_res["status"] == "success"
        assert eval_res["all_passed"] is True

        with tempfile.TemporaryDirectory() as tmpdir:
            exp_res = HubConnector.dispatch("vesc_export_xml", {
                "config": config,
                "output_dir": tmpdir
            })
            assert exp_res["status"] == "success"
            assert Path(exp_res["mcconf_xml_path"]).exists()
            assert Path(exp_res["appconf_xml_path"]).exists()


# =====================================================================
# 6. VESCMotorEngineerHarness Facade Tests
# =====================================================================
class TestAgentInterfaceFacade:
    def test_facade_end_to_end(self):
        harness = VESCMotorEngineerHarness()
        prof = harness.generate_profile("flight_maxload")

        is_valid, violations = harness.validate(prof)
        assert is_valid is True
        assert len(violations) == 0

        report, record = harness.run_evaluation(prof)
        assert report.all_passed is True
        assert record.all_passed is True

        with tempfile.TemporaryDirectory() as tmpdir:
            paths = harness.export(prof, tmpdir)
            assert Path(paths["json_profile"]).exists()
            assert Path(paths["mcconf_xml"]).exists()
            assert Path(paths["appconf_xml"]).exists()
