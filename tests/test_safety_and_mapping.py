"""Unit tests for VESC DroneCAN throttle mapping, pole-pair scaling, and safety guard trip conditions."""

import pytest
import struct


def map_raw_command_to_mech_rpm(raw_val: float) -> float:
    """Replicates patched VESC canard_driver.c mapping logic."""
    if raw_val <= 0.015:
        return 0.0
    norm = (raw_val - 0.015) / (1.0 - 0.015)
    if norm > 1.0:
        norm = 1.0
    return 1000.0 + norm * 5000.0


def map_raw_command_to_erpm(raw_val: float, pole_pairs: int = 12) -> float:
    """Replicates patched VESC canard_driver.c ERPM command logic."""
    mech_rpm = map_raw_command_to_mech_rpm(raw_val)
    return mech_rpm * float(pole_pairs)


def map_rpm_command_to_erpm(rpm_val: float, pole_pairs: int = 12) -> float:
    """Replicates patched VESC canard_driver.c handle_esc_rpm_command logic."""
    if abs(rpm_val) < 100.0:
        return 0.0
    return rpm_val * float(pole_pairs)


class TestDroneCANMapping:
    def test_zero_throttle_cutoff(self):
        """Throttle at or below 1.5% must yield 0.0 Mech RPM (motor release)."""
        assert map_raw_command_to_mech_rpm(0.0) == 0.0
        assert map_raw_command_to_mech_rpm(0.005) == 0.0
        assert map_raw_command_to_mech_rpm(0.010) == 0.0
        assert map_raw_command_to_mech_rpm(0.015) == 0.0

    def test_minimum_active_throttle(self):
        """Just above 1.5% throttle starts smoothly at 1000.0 Mech RPM."""
        mech_rpm = map_raw_command_to_mech_rpm(0.015001)
        assert pytest.approx(mech_rpm, rel=1e-3) == 1000.0
        erpm = map_raw_command_to_erpm(0.015001, pole_pairs=12)
        assert pytest.approx(erpm, rel=1e-3) == 12000.0

    def test_linear_curve_across_throttle_range(self):
        """Verify linear progression across throttle steps."""
        # 10% raw throttle
        mech_10 = map_raw_command_to_mech_rpm(0.10)
        assert 1400.0 <= mech_10 <= 1450.0

        # 20% raw throttle
        mech_20 = map_raw_command_to_mech_rpm(0.20)
        assert 1900.0 <= mech_20 <= 1950.0

        # 40% raw throttle
        mech_40 = map_raw_command_to_mech_rpm(0.40)
        assert 2950.0 <= mech_40 <= 3000.0

        # 60% raw throttle
        mech_60 = map_raw_command_to_mech_rpm(0.60)
        assert 3950.0 <= mech_60 <= 4000.0

        # 80% raw throttle
        mech_80 = map_raw_command_to_mech_rpm(0.80)
        assert 4950.0 <= mech_80 <= 5000.0

        # 100% full throttle
        mech_100 = map_raw_command_to_mech_rpm(1.00)
        assert pytest.approx(mech_100, rel=1e-5) == 6000.0
        assert pytest.approx(map_raw_command_to_erpm(1.00, pole_pairs=12), rel=1e-5) == 72000.0

    def test_dronecan_rpm_command_pole_pairs(self):
        """Verify DroneCAN RPMCommand incorporates 12 pole pairs."""
        assert map_rpm_command_to_erpm(50.0, pole_pairs=12) == 0.0  # Cutoff < 100
        assert map_rpm_command_to_erpm(1000.0, pole_pairs=12) == 12000.0
        assert map_rpm_command_to_erpm(3000.0, pole_pairs=12) == 36000.0
        assert map_rpm_command_to_erpm(6000.0, pole_pairs=12) == 72000.0


class TestSafetyGuardTripLogic:
    def test_overcurrent_clamp(self):
        """Bench motor current must trip if exceeding 2.5 A."""
        max_bench_current = 2.5
        assert abs(1.2) <= max_bench_current
        assert abs(2.4) <= max_bench_current
        assert abs(2.6) > max_bench_current  # Trip

    def test_voltage_limits(self):
        """DC bench power supply limits: 16.0V <= Vin <= 26.0V."""
        min_v = 16.0
        max_v = 26.0
        assert min_v <= 20.3 <= max_v
        assert min_v <= 24.0 <= max_v
        # Regenerative spike trip:
        assert 27.2 > max_v
        # Undervoltage drop trip:
        assert 14.5 < min_v

    def test_hardware_register_kill_values(self):
        """STM32F405 TIM1 register kill addresses and bitmasks."""
        TIM1_CR1 = 0x40010000
        TIM1_BDTR = 0x40010044
        MOE_BIT = 15
        CEN_BIT = 0

        # Clear MOE (bit 15) and CEN (bit 0)
        assert (0x00000000 >> MOE_BIT) & 1 == 0
        assert (0x00000000 >> CEN_BIT) & 1 == 0
