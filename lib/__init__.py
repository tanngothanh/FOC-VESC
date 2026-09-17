"""VESC Auto-Tuning and Provisioning Tool Suite."""

from .vesc_protocol import (
    crc16,
    encode_packet,
    decode_packet,
    encode_comm_get_values,
    encode_comm_set_duty,
    encode_comm_set_current,
    encode_comm_set_current_brake,
    encode_comm_set_rpm,
    encode_comm_reboot,
    encode_comm_alive,
    decode_telemetry,
    encode_telemetry_payload,
    COMM_FW_VERSION,
    COMM_GET_VALUES,
    COMM_SET_DUTY,
    COMM_SET_CURRENT,
    COMM_SET_RPM,
    COMM_REBOOT,
    COMM_ALIVE,
    FAULT_CODES,
)

from .vesc_interface import (
    VESCInterface,
    VESCSLCANInterface,
)

__all__ = [
    "crc16",
    "encode_packet",
    "decode_packet",
    "encode_comm_get_values",
    "encode_comm_set_duty",
    "encode_comm_set_current",
    "encode_comm_set_current_brake",
    "encode_comm_set_rpm",
    "encode_comm_reboot",
    "encode_comm_alive",
    "decode_telemetry",
    "encode_telemetry_payload",
    "COMM_FW_VERSION",
    "COMM_GET_VALUES",
    "COMM_SET_DUTY",
    "COMM_SET_CURRENT",
    "COMM_SET_RPM",
    "COMM_REBOOT",
    "COMM_ALIVE",
    "FAULT_CODES",
    "VESCInterface",
    "VESCSLCANInterface",
]
