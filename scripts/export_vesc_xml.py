"""VESC XML Configuration Exporter for Sunnysky V4006.

Reads JSON profiles and generates valid VESC Tool XML configuration files:
- Motor configuration XML (mcconf, root: <MCConfiguration>)
- App configuration XML (appconf, root: <APPConfiguration>)
"""

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path

DEFAULT_PROFILE = Path(__file__).resolve().parent.parent / "profiles" / "v4006_vesc_bench_5a.json"
DEFAULT_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"

def dict_to_pretty_xml(root_tag: str, data_dict: dict) -> str:
    """Converts dictionary key-values to formatted VESC XML string."""
    root = ET.Element(root_tag)
    for k, v in data_dict.items():
        elem = ET.SubElement(root, k)
        elem.text = str(v)
    rough = ET.tostring(root, "utf-8")
    reparsed = minidom.parseString(rough)
    return reparsed.toprettyxml(indent="    ")

def build_mcconf_dict(profile: dict) -> dict:
    """Extracts motor configuration parameters from profile."""
    mp = profile.get("motor_parameters", {})
    cl = profile.get("current_limits", {})
    sl = profile.get("speed_limits", {})
    ol = profile.get("openloop_settings", {})
    foc = profile.get("foc_settings", {})
    hfi = profile.get("hfi_settings", {})

    return {
        "pwm_mode": 1,
        "comm_mode": 0,
        "motor_type": mp.get("motor_type", 2),
        "sensor_mode": 0,
        "l_current_max": cl.get("l_current_max", 8.0),
        "l_current_min": cl.get("l_current_min", 0.0),
        "l_in_current_max": cl.get("l_in_current_max", 5.0),
        "l_in_current_min": cl.get("l_in_current_min", 0.0),
        "l_abs_current_max": cl.get("l_abs_current_max", 15.0),
        "l_min_erpm": sl.get("l_min_erpm", 0.0),
        "l_max_erpm": sl.get("l_max_erpm", 72000.0),
        "l_erpm_start": sl.get("l_erpm_start", 0.8),
        "l_max_erpm_fbrake": 300.0,
        "l_max_erpm_fbrake_cc": 1500.0,
        "l_slow_abs_current": cl.get("l_slow_abs_current", 1),
        "s_pid_min_erpm": sl.get("s_pid_min_erpm", 1800.0),
        "s_pid_kp": sl.get("s_pid_kp", 0.002),
        "s_pid_ki": sl.get("s_pid_ki", 0.004),
        "s_pid_kd": sl.get("s_pid_kd", 0.0001),
        "s_pid_kd_filter": sl.get("s_pid_kd_filter", 0.2),
        "s_pid_ramp_erpms_s": sl.get("s_pid_ramp_erpms_s", 20000.0),
        "foc_openloop_rpm": ol.get("foc_openloop_rpm", 3000.0),
        "foc_openloop_rpm_low": ol.get("foc_openloop_rpm_low", 0.70),
        "foc_sl_openloop_hyst": ol.get("foc_sl_openloop_hyst", 0.10),
        "foc_sl_openloop_time_lock": ol.get("foc_sl_openloop_time_lock", 0.10),
        "foc_sl_openloop_time_ramp": ol.get("foc_sl_openloop_time_ramp", 0.30),
        "foc_sl_openloop_time": ol.get("foc_sl_openloop_time", 0.12),
        "foc_sl_openloop_boost_q": ol.get("foc_sl_openloop_boost_q", 4.0),
        "foc_sl_openloop_max_q": ol.get("foc_sl_openloop_max_q", 8.0),
        "foc_sensor_mode": foc.get("foc_sensor_mode", 0),
        "foc_f_zv": foc.get("foc_f_zv", 24000.0),
        "foc_dt_us": foc.get("foc_dt_us", 0.12),
        "foc_motor_r": mp.get("r_phase_ohm", 0.0505),
        "foc_motor_l": mp.get("l_phase_h", 1.413e-5),
        "foc_motor_flux_linkage": mp.get("flux_linkage_wb", 0.0006207),
        "foc_observer_type": foc.get("foc_observer_type", 3),
        "foc_observer_gain": foc.get("foc_observer_gain", 90000000.0),
        "foc_cc_decoupling": foc.get("foc_cc_decoupling", 0),
        "foc_current_kp": foc.get("foc_current_kp", 0.0141),
        "foc_current_ki": foc.get("foc_current_ki", 50.5),
        "foc_hfi_voltage_start": hfi.get("foc_hfi_voltage_start", 6.0),
        "foc_hfi_voltage_run": hfi.get("foc_hfi_voltage_run", 4.5),
        "foc_hfi_voltage_max": hfi.get("foc_hfi_voltage_max", 7.0),
        "foc_sl_erpm_hfi": hfi.get("foc_sl_erpm_hfi", 3000.0),
        "foc_sl_erpm": hfi.get("foc_sl_erpm", 4500.0),
        "foc_hfi_amb_curr": hfi.get("foc_hfi_amb_curr", 8.0),
        "foc_hfi_start_samples": hfi.get("foc_hfi_start_samples", 65),
        "foc_hfi_obs_ovr_sec": hfi.get("foc_hfi_obs_ovr_sec", 0.001),
        "foc_hfi_samples": hfi.get("foc_hfi_samples", 1),
        "si_motor_poles": mp.get("poles", 24)
    }

def build_appconf_dict(profile: dict) -> dict:
    """Extracts app and UAVCAN configuration parameters from profile."""
    can = profile.get("can_uavcan", {})
    return {
        "controller_id": can.get("controller_id", 103),
        "app_to_use": 0,
        "can_baud_rate": can.get("can_baud_rate", 3),
        "can_mode": can.get("can_mode", 4),
        "uavcan_esc_index": can.get("uavcan_esc_index", 0),
        "uavcan_raw_mode": can.get("uavcan_raw_mode", 3),
        "uavcan_raw_rpm_min": can.get("uavcan_raw_rpm_min", 12000.0),
        "uavcan_raw_rpm_max": can.get("uavcan_raw_rpm_max", 72000.0),
        "uavcan_status_current_mode": can.get("uavcan_status_current_mode", 0),
        "send_can_status": can.get("send_can_status", 1),
        "send_can_status_rate_hz": can.get("send_can_status_rate_hz", 50)
    }

def export_xml(profile_path: Path, motor_xml_path: Path, app_xml_path: Path) -> None:
    with open(profile_path, "r", encoding="utf-8") as f:
        profile = json.load(f)

    mcconf = build_mcconf_dict(profile)
    appconf = build_appconf_dict(profile)

    motor_xml = dict_to_pretty_xml("MCConfiguration", mcconf)
    app_xml = dict_to_pretty_xml("APPConfiguration", appconf)

    motor_xml_path.parent.mkdir(parents=True, exist_ok=True)
    app_xml_path.parent.mkdir(parents=True, exist_ok=True)

    with open(motor_xml_path, "w", encoding="utf-8") as f:
        f.write(motor_xml)
    print(f"[OK] Motor Config XML exported: {motor_xml_path}")

    with open(app_xml_path, "w", encoding="utf-8") as f:
        f.write(app_xml)
    print(f"[OK] App UAVCAN XML exported:   {app_xml_path}")

def main():
    parser = argparse.ArgumentParser(description="Export VESC Tool XML configuration from JSON profile.")
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE,
                        help=f"Path to JSON profile (default: {DEFAULT_PROFILE.name})")
    parser.add_argument("--motor-xml", type=Path, default=None,
                        help="Destination path for Motor Config XML")
    parser.add_argument("--app-xml", type=Path, default=None,
                        help="Destination path for App Config XML")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_PROFILES_DIR,
                        help="Output directory if specific paths not set")

    args = parser.parse_args()

    motor_xml = args.motor_xml or (args.out_dir / "v4006_motor_config.xml")
    app_xml = args.app_xml or (args.out_dir / "v4006_app_uavcan.xml")

    export_xml(args.profile, motor_xml, app_xml)

if __name__ == "__main__":
    main()
