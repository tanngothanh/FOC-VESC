"""VESC Communication Protocol Implementation.

Implements CRC16 CCITT (0x1021), packet framing (short <=256, long >256),
command encoding, and telemetry decoding for VESC motor controllers.
"""

import struct
from typing import Optional, Tuple, Dict, Any

# Precompute CRC-16 CCITT table (poly 0x1021, init 0x0000)
CRC16_TABLE = []
_POLY = 0x1021
for _i in range(256):
    _curr = _i << 8
    for _ in range(8):
        if _curr & 0x8000:
            _curr = ((_curr << 1) ^ _POLY) & 0xFFFF
        else:
            _curr = (_curr << 1) & 0xFFFF
    CRC16_TABLE.append(_curr)


def crc16(data: bytes) -> int:
    """Calculate 16-bit CRC CCITT (0x1021) with initial value 0."""
    cksum = 0
    for b in data:
        cksum = CRC16_TABLE[((cksum >> 8) ^ b) & 0xFF] ^ ((cksum << 8) & 0xFFFF)
    return cksum


# VESC Command IDs
COMM_FW_VERSION = 0
COMM_JUMP_TO_BOOTLOADER = 1
COMM_ERASE_NEW_APP = 2
COMM_WRITE_NEW_APP_DATA = 3
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_POS = 9
COMM_SET_HANDBRAKE = 10
COMM_SET_DETECT = 11
COMM_ROTOR_POSITION = 21
COMM_ALIVE = 30
COMM_REBOOT = 31
COMM_CUSTOM_APP_DATA = 32
COMM_GET_VALUES_SETUP = 50

# VESC Fault Codes
FAULT_CODES = {
    0: "FAULT_CODE_NONE",
    1: "FAULT_CODE_OVER_VOLTAGE",
    2: "FAULT_CODE_UNDER_VOLTAGE",
    3: "FAULT_CODE_DRV",
    4: "FAULT_CODE_ABS_OVER_CURRENT",
    5: "FAULT_CODE_OVER_TEMP_FET",
    6: "FAULT_CODE_OVER_TEMP_MOTOR",
    7: "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE",
    8: "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE",
    9: "FAULT_CODE_MCU_UNDER_VOLTAGE",
    10: "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET",
    11: "FAULT_CODE_ENCODER_SPI",
    12: "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FAULT_CODE_FLASH_CORRUPTION",
    15: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "FAULT_CODE_UNBALANCED_CURRENTS",
    19: "FAULT_CODE_BRK",
    20: "FAULT_CODE_RESOLVER_LOT",
    21: "FAULT_CODE_RESOLVER_DOS",
    22: "FAULT_CODE_RESOLVER_LOS",
    23: "FAULT_CODE_FLASH_CORRUPTION_APP_CFG",
    24: "FAULT_CODE_FLASH_CORRUPTION_MC_CFG",
    25: "FAULT_CODE_ENCODER_NO_MAGNET",
    26: "FAULT_CODE_ENCODER_MAGNET_TOO_STRONG",
    27: "FAULT_CODE_PHASE_FILTER",
    28: "FAULT_CODE_ENCODER_FAULT",
    29: "FAULT_CODE_LV_OUTPUT_FAULT",
    30: "FAULT_CODE_ENCODER_SLIP",
    31: "FAULT_CODE_OVERSPEED",
    32: "FAULT_CODE_UNDERSPEED",
    33: "FAULT_CODE_ABS_OVERSPEED",
}


def encode_packet(payload: bytes) -> bytes:
    """Encodes a payload into a framed VESC packet with CRC16.
    
    - Short packet (len <= 256): 0x02, len, payload, crc_hi, crc_lo, 0x03
      (For len == 256, len & 0xFF is 0 per VESC protocol specification)
    - Long packet (len > 256): 0x03, len_hi, len_lo, payload, crc_hi, crc_lo, 0x03
    """
    plen = len(payload)
    crc = crc16(payload)
    crc_bytes = struct.pack('>H', crc)
    
    if plen <= 256:
        header = bytes([0x02, plen & 0xFF])
    else:
        header = bytes([0x03, (plen >> 8) & 0xFF, plen & 0xFF])
        
    return header + payload + crc_bytes + bytes([0x03])


def decode_packet(buffer: bytes) -> Tuple[Optional[bytes], bytes]:
    """Extracts the first valid VESC packet from buffer stream.
    
    Returns:
        (payload, remaining_buffer) if valid packet found,
        (None, remaining_buffer) if packet is incomplete or invalid.
    """
    while len(buffer) >= 5:
        start_byte = buffer[0]
        if start_byte == 0x02:
            # Short packet
            raw_len = buffer[1]
            plen = 256 if raw_len == 0 else raw_len
            total_len = 1 + 1 + plen + 2 + 1
            if len(buffer) < total_len:
                return None, buffer
            if buffer[total_len - 1] != 0x03:
                buffer = buffer[1:]
                continue
            payload = buffer[2:2 + plen]
            expected_crc = struct.unpack('>H', buffer[2 + plen:4 + plen])[0]
            if crc16(payload) == expected_crc:
                return payload, buffer[total_len:]
            else:
                buffer = buffer[1:]
                continue

        elif start_byte == 0x03:
            # Long packet
            if len(buffer) < 6:
                return None, buffer
            plen = (buffer[1] << 8) | buffer[2]
            total_len = 1 + 2 + plen + 2 + 1
            if len(buffer) < total_len:
                return None, buffer
            if buffer[total_len - 1] != 0x03:
                buffer = buffer[1:]
                continue
            payload = buffer[3:3 + plen]
            expected_crc = struct.unpack('>H', buffer[3 + plen:5 + plen])[0]
            if crc16(payload) == expected_crc:
                return payload, buffer[total_len:]
            else:
                buffer = buffer[1:]
                continue
        else:
            buffer = buffer[1:]

    return None, buffer


# Command Payload Builders
def encode_comm_get_values() -> bytes:
    """Payload to request telemetry values."""
    return bytes([COMM_GET_VALUES])


def encode_comm_set_duty(duty: float) -> bytes:
    """Payload to set duty cycle (-1.0 to 1.0, scaled by 100,000)."""
    val = int(round(duty * 100000.0))
    return bytes([COMM_SET_DUTY]) + struct.pack('>i', val)


def encode_comm_set_current(current: float) -> bytes:
    """Payload to set motor current in Amps (scaled by 1,000)."""
    val = int(round(current * 1000.0))
    return bytes([COMM_SET_CURRENT]) + struct.pack('>i', val)


def encode_comm_set_current_brake(current: float) -> bytes:
    """Payload to set braking current in Amps (scaled by 1,000)."""
    val = int(round(abs(current) * 1000.0))
    return bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack('>i', val)


def encode_comm_set_rpm(rpm: int) -> bytes:
    """Payload to set electrical RPM (ERPM as signed int32)."""
    return bytes([COMM_SET_RPM]) + struct.pack('>i', int(rpm))


def encode_comm_reboot() -> bytes:
    """Payload to reboot VESC MCU."""
    return bytes([COMM_REBOOT])


def encode_comm_alive() -> bytes:
    """Payload to keep VESC watchdog alive."""
    return bytes([COMM_ALIVE])


# Telemetry Parsing and Encoding
def decode_telemetry(payload: bytes) -> Dict[str, Any]:
    """Decodes COMM_GET_VALUES payload into dictionary of telemetry values."""
    if len(payload) < 53:
        raise ValueError(f"Payload too short for COMM_GET_VALUES: {len(payload)} bytes (min 53)")

    offset = 0
    if payload[0] == COMM_GET_VALUES:
        offset = 1

    if len(payload) - offset < 53:
        raise ValueError(f"Insufficient data bytes in telemetry payload: {len(payload) - offset}")

    data = payload[offset:]

    temp_mos_raw = struct.unpack_from('>h', data, 0)[0]
    temp_motor_raw = struct.unpack_from('>h', data, 2)[0]
    current_motor_raw = struct.unpack_from('>i', data, 4)[0]
    current_in_raw = struct.unpack_from('>i', data, 8)[0]
    id_raw = struct.unpack_from('>i', data, 12)[0]
    iq_raw = struct.unpack_from('>i', data, 16)[0]
    duty_now_raw = struct.unpack_from('>h', data, 20)[0]
    rpm = struct.unpack_from('>i', data, 22)[0]
    v_in_raw = struct.unpack_from('>h', data, 26)[0]
    amphours_raw = struct.unpack_from('>i', data, 28)[0]
    amphours_charged_raw = struct.unpack_from('>i', data, 32)[0]
    watt_hours_raw = struct.unpack_from('>i', data, 36)[0]
    watt_hours_charged_raw = struct.unpack_from('>i', data, 40)[0]
    tachometer = struct.unpack_from('>i', data, 44)[0]
    tachometer_abs = struct.unpack_from('>i', data, 48)[0]
    fault_code = data[52]

    return {
        'temp_mos': round(temp_mos_raw / 10.0, 1),
        'temp_motor': round(temp_motor_raw / 10.0, 1),
        'current_motor': round(current_motor_raw / 100.0, 2),
        'current_in': round(current_in_raw / 100.0, 2),
        'id': round(id_raw / 100.0, 2),
        'iq': round(iq_raw / 100.0, 2),
        'duty_now': round(duty_now_raw / 1000.0, 3),
        'rpm': int(rpm),
        'v_in': round(v_in_raw / 10.0, 2),
        'amphours': round(amphours_raw / 10000.0, 4),
        'amphours_charged': round(amphours_charged_raw / 10000.0, 4),
        'watt_hours': round(watt_hours_raw / 10000.0, 4),
        'watt_hours_charged': round(watt_hours_charged_raw / 10000.0, 4),
        'tachometer': int(tachometer),
        'tachometer_abs': int(tachometer_abs),
        'fault_code': int(fault_code),
        'fault_str': FAULT_CODES.get(int(fault_code), f"FAULT_CODE_UNKNOWN_{fault_code}")
    }


def encode_telemetry_payload(t: Dict[str, Any], include_cmd_id: bool = True) -> bytes:
    """Encodes a telemetry dictionary into COMM_GET_VALUES binary payload (useful for testing/mocking)."""
    buf = bytearray()
    if include_cmd_id:
        buf.append(COMM_GET_VALUES)
    buf.extend(struct.pack('>h', int(round(t.get('temp_mos', 25.0) * 10))))
    buf.extend(struct.pack('>h', int(round(t.get('temp_motor', 25.0) * 10))))
    buf.extend(struct.pack('>i', int(round(t.get('current_motor', 0.0) * 100))))
    buf.extend(struct.pack('>i', int(round(t.get('current_in', 0.0) * 100))))
    buf.extend(struct.pack('>i', int(round(t.get('id', 0.0) * 100))))
    buf.extend(struct.pack('>i', int(round(t.get('iq', 0.0) * 100))))
    buf.extend(struct.pack('>h', int(round(t.get('duty_now', 0.0) * 1000))))
    buf.extend(struct.pack('>i', int(round(t.get('rpm', 0)))))
    buf.extend(struct.pack('>h', int(round(t.get('v_in', 24.0) * 10))))
    buf.extend(struct.pack('>i', int(round(t.get('amphours', 0.0) * 10000))))
    buf.extend(struct.pack('>i', int(round(t.get('amphours_charged', 0.0) * 10000))))
    buf.extend(struct.pack('>i', int(round(t.get('watt_hours', 0.0) * 10000))))
    buf.extend(struct.pack('>i', int(round(t.get('watt_hours_charged', 0.0) * 10000))))
    buf.extend(struct.pack('>i', int(t.get('tachometer', 0))))
    buf.extend(struct.pack('>i', int(t.get('tachometer_abs', 0))))
    buf.append(int(t.get('fault_code', 0)) & 0xFF)
    return bytes(buf)
