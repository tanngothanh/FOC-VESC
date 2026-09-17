"""VESC Generalized Auto-Tuner (PROP-SUB-05).

Provides systematic, multi-tier motor tuning for arbitrary BLDC/PMSM motors and
arbitrary VESC power stages (from small 4S UAVs up to 800V HV eVTOL propulsion).
"""

from enum import Enum
from typing import Dict, Any, Optional
from dataclasses import dataclass

from .vesc_config_engine import (
    MotorSpec,
    HardwareLimits,
    OperatingEnvelope,
    VESCConfigEngine,
    SafetyViolationError
)


class OperatingMode(Enum):
    BENCH_SAFE = "bench_safe"         # Safe testing with bench DC supply (5A limit, freewheel stop)
    FLIGHT_NOMINAL = "flight_nominal" # Cruise flight with propeller (25A continuous)
    FLIGHT_MAXLOAD = "flight_maxload" # Maximum continuous load (35A/30s, 560W clamp, active duty clamp)


class VESCHardwareTier(Enum):
    VESC_4_12 = "vesc_4_12"           # 12S, 50A peak, 1.5 kW
    VESC_6_MKV = "vesc_6_mkv"         # 12S, 80A continuous, 3 kW
    VESC_75_300 = "vesc_75_300"       # 18S, 300A continuous, 20 kW
    VESC_HV_800V = "vesc_hv_800v"     # 200S (800V), 100A continuous, 80 kW (eVTOL Main Tilt/Lift)


# Standard Motor Library
MOTOR_LIBRARY = {
    "SUNNYSKY_V4006_740KV": MotorSpec(
        name="Sunnysky V4006 740KV",
        poles=24,
        slots=18,
        kv=740.0,
        r_phase_ohm=0.0470,
        l_phase_h=1.413e-5,
        i_0=0.9,
        v_0=10.0,
        i_max_cont=35.0,
        p_max_cont=560.0,
        propeller_recommended="15x5.5",
        thermal_time_const_s=30.0
    ),
    "SUNNYSKY_V4008_380KV": MotorSpec(
        name="Sunnysky V4008 380KV",
        poles=24,
        slots=18,
        kv=380.0,
        r_phase_ohm=0.0920,
        l_phase_h=3.25e-5,
        i_0=0.6,
        v_0=10.0,
        i_max_cont=28.0,
        p_max_cont=620.0,
        propeller_recommended="17x5.8",
        thermal_time_const_s=35.0
    ),
    "TMOTOR_U8II_100KV": MotorSpec(
        name="T-Motor U8II 100KV",
        poles=36,
        slots=36,
        kv=100.0,
        r_phase_ohm=0.1050,
        l_phase_h=8.50e-5,
        i_0=1.1,
        v_0=24.0,
        i_max_cont=45.0,
        p_max_cont=2100.0,
        propeller_recommended="28x9.2",
        thermal_time_const_s=45.0
    ),
    "TMOTOR_U15_80KV": MotorSpec(
        name="T-Motor U15 80KV",
        poles=42,
        slots=36,
        kv=80.0,
        r_phase_ohm=0.0380,
        l_phase_h=5.20e-5,
        i_0=2.5,
        v_0=48.0,
        i_max_cont=120.0,
        p_max_cont=6500.0,
        propeller_recommended="40x13",
        thermal_time_const_s=60.0
    )
}

# Standard Hardware Tier Library
HARDWARE_TIERS = {
    VESCHardwareTier.VESC_4_12: HardwareLimits(
        v_bus_nominal=22.2,
        v_bus_max=50.0,
        v_bus_min=12.0,
        i_dc_max=25.0,
        i_dc_min=0.0,
        i_phase_max=50.0,
        i_phase_abs_trip=70.0,
        power_max_w=1500.0,
        shunt_resistance_ohm=0.001,
        mosfet_max_voltage=60.0
    ),
    VESCHardwareTier.VESC_6_MKV: HardwareLimits(
        v_bus_nominal=20.5,
        v_bus_max=54.0,
        v_bus_min=14.0,
        i_dc_max=27.5, # Bench safe limit < 28A
        i_dc_min=0.0,  # Zero DC supply backfeed
        i_phase_max=35.0,
        i_phase_abs_trip=55.0,
        power_max_w=560.0,
        shunt_resistance_ohm=0.001,
        mosfet_max_voltage=60.0
    ),
    VESCHardwareTier.VESC_75_300: HardwareLimits(
        v_bus_nominal=66.6,
        v_bus_max=75.0,
        v_bus_min=30.0,
        i_dc_max=200.0,
        i_dc_min=0.0,
        i_phase_max=300.0,
        i_phase_abs_trip=400.0,
        power_max_w=20000.0,
        shunt_resistance_ohm=0.0005,
        mosfet_max_voltage=100.0
    ),
    VESCHardwareTier.VESC_HV_800V: HardwareLimits(
        v_bus_nominal=800.0,
        v_bus_max=900.0,
        v_bus_min=600.0,
        i_dc_max=100.0,
        i_dc_min=0.0,
        i_phase_max=150.0,
        i_phase_abs_trip=250.0,
        power_max_w=80000.0,
        shunt_resistance_ohm=0.0002,
        mosfet_max_voltage=1200.0
    )
}


class VESCAutoTuner:
    """End-to-End Automatic Tuning Engine."""

    def __init__(
        self,
        motor: Optional[MotorSpec] = None,
        hardware: Optional[HardwareLimits] = None,
        envelope: Optional[OperatingEnvelope] = None
    ):
        self.motor = motor or MOTOR_LIBRARY["SUNNYSKY_V4006_740KV"]
        self.hardware = hardware or HARDWARE_TIERS[VESCHardwareTier.VESC_6_MKV]
        self.envelope = envelope or OperatingEnvelope()

    def tune_for_mode(self, mode: OperatingMode = OperatingMode.FLIGHT_MAXLOAD) -> Dict[str, Any]:
        """Synthesizes and verifies a configuration tailored to the specified operational mode."""
        env = OperatingEnvelope(
            min_mech_rpm=self.envelope.min_mech_rpm,
            max_mech_rpm=self.envelope.max_mech_rpm,
            allow_reverse=False, # Invariant: Unidirectional forward rotation
            allow_braking=True,
            ramp_mech_rpm_s=self.envelope.ramp_mech_rpm_s,
            watchdog_timeout_ms=self.envelope.watchdog_timeout_ms
        )

        hw = HardwareLimits(
            v_bus_nominal=self.hardware.v_bus_nominal,
            v_bus_max=self.hardware.v_bus_max,
            v_bus_min=self.hardware.v_bus_min,
            i_dc_max=self.hardware.i_dc_max,
            i_dc_min=0.0, # Anti-Backfeed invariant
            i_phase_max=self.hardware.i_phase_max,
            i_phase_abs_trip=self.hardware.i_phase_abs_trip,
            power_max_w=self.hardware.power_max_w,
            shunt_resistance_ohm=self.hardware.shunt_resistance_ohm,
            mosfet_max_voltage=self.hardware.mosfet_max_voltage
        )

        if mode == OperatingMode.BENCH_SAFE:
            hw.i_dc_max = min(5.0, hw.i_dc_max)
            hw.i_phase_max = min(8.0, hw.i_phase_max)
            env.allow_braking = False # Freewheel stop for bench supplies
            profile_name = f"{self.motor.name.lower().replace(' ', '_')}_bench_5a"
            desc = f"Bench safe profile for {self.motor.name} (5A DC limit, freewheeling stop)."
        elif mode == OperatingMode.FLIGHT_NOMINAL:
            hw.i_dc_max = min(22.0, hw.i_dc_max)
            hw.i_phase_max = min(25.0, hw.i_phase_max)
            profile_name = f"{self.motor.name.lower().replace(' ', '_')}_flight_25a"
            desc = f"Flight nominal profile for {self.motor.name} (25A continuous)."
        else: # FLIGHT_MAXLOAD
            hw.i_dc_max = min(27.5, hw.i_dc_max) # Hard limit < 28A
            hw.i_phase_max = min(self.motor.i_max_cont, hw.i_phase_max)
            profile_name = f"{self.motor.name.lower().replace(' ', '_')}_flight_35a_maxload"
            desc = f"Flight maximum continuous profile for {self.motor.name} (35A/30s, 560W clamp, duty clamp active)."

        profile = VESCConfigEngine.generate_tuning_profile(
            motor=self.motor,
            hw=hw,
            env=env,
            profile_name=profile_name,
            description=desc
        )

        # Audit and validate
        violations = VESCConfigEngine.validate_configuration(profile, hw)
        if violations:
            raise SafetyViolationError(f"Auto-tuning validation failed with safety violations: {violations}")

        return profile

    def tune_with_propeller(
        self,
        propeller_diameter_inch: float = 14.0,
        propeller_pitch_inch: float = 4.8,
        mode: OperatingMode = OperatingMode.FLIGHT_MAXLOAD
    ) -> Dict[str, Any]:
        """Synthesizes an optimal FOC profile tailored to a specific propeller load."""
        profile = self.tune_for_mode(mode)

        # Approximate propeller inertia and drag torque
        # Carbon prop mass estimation: M ~ 0.0011 * D^1.3 kg
        prop_mass_kg = 0.0011 * (propeller_diameter_inch ** 1.3)
        radius_m = (propeller_diameter_inch * 0.0254) / 2.0
        prop_inertia_j = (1.0 / 3.0) * prop_mass_kg * (radius_m ** 2)

        # Aerodynamic torque coefficient: Tau_aero = k_drag * (RPM/1000)^2
        k_drag = 0.020 * ((propeller_diameter_inch / 14.0) ** 4) * (propeller_pitch_inch / 4.8)

        # Enhance startup breakaway torque for heavy propeller inertia
        profile["openloop_settings"]["foc_openloop_rpm"] = max(1000.0, profile["openloop_settings"].get("foc_openloop_rpm", 800.0) * 1.25)
        profile["openloop_settings"]["foc_sl_openloop_boost_q"] = round(profile["openloop_settings"].get("foc_sl_openloop_boost_q", 2.5) * 1.35, 2)

        # Scale speed PI to account for higher inertia without inducing hunting
        inertia_ratio = max(1.0, prop_inertia_j / 3.5e-5)
        profile["speed_limits"]["s_pid_kp"] = round(profile["speed_limits"]["s_pid_kp"] * min(2.5, 1.0 + 0.15 * inertia_ratio), 5)
        profile["speed_limits"]["s_pid_ramp_erpms_s"] = round(min(profile["speed_limits"].get("s_pid_ramp_erpms_s", 25000.0), 30000.0), 1)

        # Attach propeller load spec to profile
        profile["propeller_load"] = {
            "diameter_inch": propeller_diameter_inch,
            "pitch_inch": propeller_pitch_inch,
            "inertia_j": round(prop_inertia_j, 6),
            "k_drag": round(k_drag, 4)
        }
        profile["metadata"]["description"] += f" (Optimized for {propeller_diameter_inch}x{propeller_pitch_inch} Propeller)"

        return profile
