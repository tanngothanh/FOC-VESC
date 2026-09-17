"""VESC Configuration & Tuning Engine (PROP-SUB-05).

Provides mathematical modeling, parameter generation, validation, and serialization
for VESC motor controllers across arbitrary BLDC motors and power tiers.
Enforces all aerospace safety invariants:
1. Anti-HFI for SPM Outrunners (foc_sensor_mode = 0).
2. Anti-Backfeed for DC Supplies (l_in_current_min >= 0.0).
3. Physical Duty Cycle Anti-Runaway Clamping.
4. Unidirectional Forward Rotation Invariant (l_min_erpm = 0.0).
5. Sensorless Speed Derivative Invariant (s_pid_kd = 0.0).
6. Communication Watchdog Invariant (timeout_msec <= 300).
"""

import math
import json
import xml.etree.ElementTree as ET
from xml.dom import minidom
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, List, Optional, Tuple


class SafetyViolationError(ValueError):
    """Raised when a proposed VESC configuration violates critical hardware safety invariants."""
    pass


@dataclass
class MotorSpec:
    """Brushless DC / Permanent Magnet Synchronous Motor Specification."""
    name: str = "Sunnysky V4006 740KV"
    poles: int = 24             # Total magnetic poles (2P)
    slots: int = 18             # Stator slots (e.g. 18N24P)
    kv: float = 740.0           # Velocity constant (RPM/V)
    r_phase_ohm: float = 0.0470 # Phase resistance in Ohms (47 mOhm)
    l_phase_h: float = 1.413e-5 # Phase inductance in Henry (14.13 uH)
    i_0: float = 0.9            # No-load current in Amperes
    v_0: float = 10.0           # No-load test voltage in Volts
    i_max_cont: float = 35.0    # Max continuous current (Amperes, 30s rating)
    p_max_cont: float = 560.0   # Max continuous power (Watts)
    propeller_recommended: str = "15x5.5"
    thermal_time_const_s: float = 30.0

    @property
    def pole_pairs(self) -> int:
        return self.poles // 2

    @property
    def flux_linkage_wb(self) -> float:
        """Calculates PM flux linkage lambda (Weber) from Kv and pole pairs.
        Formula: lambda = 60 / (sqrt(3) * 2 * pi * Kv * pole_pairs)
        """
        if self.kv <= 0 or self.pole_pairs <= 0:
            return 0.0
        return 60.0 / (math.sqrt(3.0) * 2.0 * math.pi * self.kv * self.pole_pairs)


@dataclass
class HardwareLimits:
    """VESC Controller and Power Supply Hardware Constraints."""
    v_bus_nominal: float = 20.5       # Operating DC bus voltage (Volts)
    v_bus_max: float = 25.2           # Max DC bus voltage (6S full charge: 25.2V)
    v_bus_min: float = 16.0           # Undervoltage cutoff (Volts)
    i_dc_max: float = 27.5            # Max DC supply current in Amps (< 28A bench specification)
    i_dc_min: float = 0.0             # Min DC supply current (Strict Anti-Backfeed: 0.0A)
    i_phase_max: float = 35.0         # Max motor phase current in Amps
    i_phase_abs_trip: float = 55.0    # Absolute hardware shoot-through trip current
    power_max_w: float = 560.0        # Continuous electrical power ceiling (Watts)
    shunt_resistance_ohm: float = 0.001 # Shunt resistor value for current measurement
    mosfet_max_voltage: float = 60.0  # VESC bridge voltage rating


@dataclass
class OperatingEnvelope:
    """Aircraft Propulsion Operational Speed and Control Boundaries."""
    min_mech_rpm: float = 1000.0      # Minimum operational mechanical RPM (12,000 ERPM)
    max_mech_rpm: float = 6000.0      # Maximum operational mechanical RPM (72,000 ERPM)
    allow_reverse: bool = False       # Unidirectional propeller invariant
    allow_braking: bool = True        # Allow active speed deceleration
    ramp_mech_rpm_s: float = 2500.0   # Acceleration rate: 2,500 RPM/s (30,000 ERPM/s)
    watchdog_timeout_ms: int = 200    # Failsafe shutdown timeout in ms (< 300 ms)


class VESCConfigEngine:
    """Mathematical parameter synthesizer and safety validator for VESC."""

    @staticmethod
    def calculate_duty_ceiling(motor: MotorSpec, hw: HardwareLimits, env: OperatingEnvelope, margin: float = 1.0075) -> float:
        """Calculates the physical duty cycle ceiling required to reach max RPM without runaway.
        
        Formula:
            No-load ideal speed = V_bus * Kv (Mech RPM)
            Duty_max = (max_mech_rpm * margin) / (V_bus * Kv)
        Capped to 0.95 maximum.
        """
        theoretical_no_load_rpm = hw.v_bus_nominal * motor.kv
        if theoretical_no_load_rpm <= 0:
            return 0.95
        calculated_duty = (env.max_mech_rpm * margin) / theoretical_no_load_rpm
        return round(min(0.95, max(0.10, calculated_duty)), 4)

    @classmethod
    def generate_tuning_profile(
        cls,
        motor: MotorSpec,
        hw: HardwareLimits,
        env: OperatingEnvelope,
        profile_name: str = "custom_tuning",
        description: str = ""
    ) -> Dict[str, Any]:
        """Generates a complete, mathematically verified VESC configuration dictionary."""
        pole_pairs = motor.pole_pairs
        flux_linkage = motor.flux_linkage_wb
        duty_ceiling = cls.calculate_duty_ceiling(motor, hw, env, margin=1.0075)

        # FOC Current loop gains based on motor R and L (bandwidth ~ 1500 rad/s)
        current_bw_rad_s = 1500.0
        current_kp = round(motor.l_phase_h * current_bw_rad_s, 6)
        current_ki = round(motor.r_phase_ohm * current_bw_rad_s, 2)

        # Observer gain calculation
        observer_gain = round(9.0e7 * (1.413e-5 / max(1e-6, motor.l_phase_h)), 1)

        # Speed loop gains (conservative stable baseline for outrunners)
        speed_kp = 0.0020
        speed_ki = 0.0005
        speed_kd = 0.0  # STRICT INVARIANT: Must remain 0.0 for sensorless observer stability

        # ERPM calculations
        max_erpm = env.max_mech_rpm * pole_pairs
        min_erpm = 0.0 if not env.allow_reverse else -(env.max_mech_rpm * pole_pairs)
        ramp_erpm_s = env.ramp_mech_rpm_s * pole_pairs
        min_active_erpm = round(env.min_mech_rpm * pole_pairs * 0.075, 1) # Idle cut-off threshold

        # Current limits
        motor_curr_max = min(motor.i_max_cont, hw.i_phase_max)
        motor_curr_min = -3.0 if env.allow_braking else 0.0
        dc_curr_max = min(hw.i_dc_max, motor_curr_max)
        dc_curr_min = 0.0  # Strict Anti-Backfeed

        profile = {
            "metadata": {
                "subsystem": "PROP-SUB-05",
                "motor_model": motor.name,
                "profile_name": profile_name,
                "description": description or f"Auto-generated profile for {motor.name} with anti-runaway clamping.",
                "duty_clamp": duty_ceiling,
                "poles": motor.poles,
                "slots": motor.slots
            },
            "motor_parameters": {
                "motor_type": 2, # MOTOR_TYPE_FOC
                "motor_type_str": "MOTOR_TYPE_FOC",
                "poles": motor.poles,
                "pole_pairs": pole_pairs,
                "kv": motor.kv,
                "r_phase_ohm": motor.r_phase_ohm,
                "l_phase_h": motor.l_phase_h,
                "flux_linkage_wb": round(flux_linkage, 8)
            },
            "current_limits": {
                "l_current_max": motor_curr_max,
                "l_current_min": motor_curr_min,
                "l_in_current_max": dc_curr_max,
                "l_in_current_min": dc_curr_min,
                "l_abs_current_max": hw.i_phase_abs_trip,
                "l_watt_max": min(motor.p_max_cont, hw.power_max_w),
                "l_watt_min": 0.0,
                "l_min_duty": 0.0050,
                "l_max_duty": duty_ceiling,
                "l_slow_abs_current": 1
            },
            "speed_limits": {
                "min_mech_rpm": env.min_mech_rpm,
                "max_mech_rpm": env.max_mech_rpm,
                "l_min_erpm": min_erpm,
                "l_max_erpm": max_erpm,
                "l_erpm_start": 0.98,
                "s_pid_min_erpm": min_active_erpm,
                "s_pid_kp": speed_kp,
                "s_pid_ki": speed_ki,
                "s_pid_kd": speed_kd,
                "s_pid_kd_filter": 0.0,
                "s_pid_allow_braking": 1 if env.allow_braking else 0,
                "s_pid_ramp_erpms_s": ramp_erpm_s
            },
            "openloop_settings": {
                "foc_openloop_rpm": round(env.min_mech_rpm * pole_pairs * 0.32, 1),
                "foc_openloop_rpm_low": 0.85,
                "foc_sl_openloop_hyst": 0.10,
                "foc_sl_openloop_time_lock": 0.10,
                "foc_sl_openloop_time_ramp": 0.35,
                "foc_sl_openloop_time": 0.15,
                "foc_sl_openloop_boost_q": round(min(5.0, motor_curr_max * 0.10), 2),
                "foc_sl_openloop_max_q": round(min(8.0, motor_curr_max * 0.20), 2)
            },
            "foc_settings": {
                "foc_sensor_mode": 0, # FOC_SENSOR_MODE_SENSORLESS
                "foc_sensor_mode_str": "FOC_SENSOR_MODE_SENSORLESS",
                "foc_f_zv": 24000.0,
                "foc_dt_us": 0.12,
                "foc_observer_type": 3, # MXLEMMING_LAMBDA_COMP
                "foc_observer_type_str": "FOC_OBSERVER_MXLEMMING_LAMBDA_COMP",
                "foc_cc_decoupling": 0,
                "foc_cc_decoupling_str": "FOC_CC_DECOUPLING_DISABLED",
                "foc_observer_gain": observer_gain,
                "foc_current_kp": current_kp,
                "foc_current_ki": current_ki
            },
            "can_uavcan": {
                "controller_id": 103,
                "can_baud_rate": 3, # CAN_BAUD_1M
                "can_baud_rate_str": "CAN_BAUD_1M",
                "can_mode": 4, # CAN_MODE_VESC_UAVCAN
                "can_mode_str": "CAN_MODE_VESC_UAVCAN",
                "timeout_msec": env.watchdog_timeout_ms,
                "uavcan_esc_index": 0,
                "uavcan_raw_mode": 3, # RPM mode
                "uavcan_raw_mode_str": "RPM Control",
                "uavcan_raw_rpm_min": env.min_mech_rpm * pole_pairs,
                "uavcan_raw_rpm_max": max_erpm,
                "uavcan_status_current_mode": 0,
                "uavcan_status_current_mode_str": "Motor Current",
                "send_can_status": 1,
                "send_can_status_rate_hz": 50
            }
        }
        return profile

    @staticmethod
    def validate_configuration(config: Dict[str, Any], hw: Optional[HardwareLimits] = None) -> List[str]:
        """Audits a VESC configuration dictionary against strict aerospace invariants.
        Returns a list of violation messages. If empty, the configuration is PASS.
        """
        violations = []
        cl = config.get("current_limits", {})
        sl = config.get("speed_limits", {})
        foc = config.get("foc_settings", {})
        can = config.get("can_uavcan", {})
        mp = config.get("motor_parameters", {})

        # 1. Anti-HFI Invariant
        sensor_mode = foc.get("foc_sensor_mode", 0)
        if sensor_mode == 3: # HFI
            violations.append("CRITICAL: foc_sensor_mode=3 (HFI) is strictly forbidden on SPM outrunners!")

        # 2. Anti-Backfeed Invariant
        in_curr_min = cl.get("l_in_current_min", 0.0)
        if in_curr_min < 0.0:
            violations.append(f"SAFETY: l_in_current_min ({in_curr_min}A) < 0.0A allows reverse regenerative power into DC supply!")

        # 3. DC Supply Protection
        if hw is not None:
            in_curr_max = cl.get("l_in_current_max", 0.0)
            if in_curr_max > hw.i_dc_max:
                violations.append(f"OVERCURRENT: l_in_current_max ({in_curr_max}A) exceeds hardware supply limit ({hw.i_dc_max}A)!")

        # 4. Unidirectional Invariant
        min_erpm = sl.get("l_min_erpm", 0.0)
        if min_erpm < 0.0:
            violations.append(f"SAFETY: l_min_erpm ({min_erpm}) < 0.0 violates unidirectional propeller invariant!")

        # 5. Anti-Runaway Duty Clamping
        max_duty = cl.get("l_max_duty", 0.95)
        if max_duty > 0.85 and sl.get("max_mech_rpm", 6000) <= 6500:
            violations.append(f"RUNAWAY_RISK: l_max_duty ({max_duty}) is unclamped for high-speed bench testing!")

        # 6. Sensorless Speed Kd Invariant
        speed_kd = sl.get("s_pid_kd", 0.0)
        if speed_kd != 0.0:
            violations.append(f"STABILITY: s_pid_kd ({speed_kd}) != 0.0 amplifies observer PLL noise and causes chattering!")

        # 7. Watchdog Failsafe Invariant
        timeout = can.get("timeout_msec", 1000)
        if timeout > 300:
            violations.append(f"FAILSAFE: timeout_msec ({timeout}ms) exceeds 300ms maximum safety cutoff!")

        return violations

    @staticmethod
    def export_mcconf_xml(profile: Dict[str, Any]) -> str:
        """Exports profile to valid VESC Tool MCConfiguration XML string."""
        from scripts.export_vesc_xml import build_mcconf_dict, dict_to_pretty_xml
        mc_dict = build_mcconf_dict(profile)
        return dict_to_pretty_xml("MCConfiguration", mc_dict)

    @staticmethod
    def export_appconf_xml(profile: Dict[str, Any]) -> str:
        """Exports profile to valid VESC Tool APPConfiguration XML string."""
        from scripts.export_vesc_xml import build_appconf_dict, dict_to_pretty_xml
        app_dict = build_appconf_dict(profile)
        return dict_to_pretty_xml("APPConfiguration", app_dict)
