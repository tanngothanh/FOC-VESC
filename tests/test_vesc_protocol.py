"""Comprehensive Pytest Suite for VESC Protocol, Interfaces, and Profiles.

Covers:
1. CRC-16 CCITT polynomial 0x1021 test vectors.
2. Short and long packet framing, delimiters, and corrupted stream recovery.
3. Command payload generation (Duty, Current, RPM, Reboot, Alive).
4. COMM_GET_VALUES telemetry serialization & deserialization.
5. JSON profile loading and VESC XML schema compliance.
6. Mock VESC serial interface & SLCAN DroneCAN frame encoding.
"""

import sys
import json
import struct
import pytest
import xml.etree.ElementTree as ET
from pathlib import Path

# Add VESC root to sys.path
VESC_DIR = Path(__file__).resolve().parent.parent
if str(VESC_DIR) not in sys.path:
    sys.path.insert(0, str(VESC_DIR))

from lib.vesc_protocol import (
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
    FAULT_CODES,
)
from lib.vesc_interface import (
    VESCInterface,
    VESCSLCANInterface,
    throttle_to_mech_rpm,
    mech_rpm_to_erpm
)
from scripts.export_vesc_xml import export_xml, build_mcconf_dict, build_appconf_dict


# =====================================================================
# 1. CRC-16 CCITT Tests
# =====================================================================
class TestCRC16:
    def test_crc16_standard_vector(self):
        """Standard CCITT test vector b'123456789' -> 0x31C3."""
        assert crc16(b"123456789") == 0x31C3

    def test_crc16_empty(self):
        """Empty buffer should return 0x0000."""
        assert crc16(b"") == 0x0000

    def test_crc16_single_byte(self):
        """Single byte COMM_GET_VALUES (0x04) test vector."""
        assert crc16(bytes([0x04])) == 0x4084

    def test_crc16_deterministic(self):
        """Calculations must be deterministic across repeated calls."""
        data = b"\x05\x00\x01\x86\xa0"
        crc1 = crc16(data)
        crc2 = crc16(data)
        assert crc1 == crc2


# =====================================================================
# 2. Packet Framing Tests
# =====================================================================
class TestPacketFraming:
    def test_short_packet_encoding(self):
        """Short packet (<= 256 bytes): 0x02, len, payload, crc_hi, crc_lo, 0x03."""
        payload = bytes([0x04])
        packet = encode_packet(payload)
        assert len(packet) == 6  # 1(start) + 1(len) + 1(payload) + 2(crc) + 1(stop)
        assert packet[0] == 0x02
        assert packet[1] == 0x01
        assert packet[2] == 0x04
        crc = (packet[3] << 8) | packet[4]
        assert crc == crc16(payload)
        assert packet[5] == 0x03

    def test_short_packet_256_bytes(self):
        """Packet with exactly 256 bytes payload uses 0x02 header with 0x00 len byte."""
        payload = bytes(range(256))
        packet = encode_packet(payload)
        assert packet[0] == 0x02
        assert packet[1] == 0x00  # 256 & 0xFF == 0
        assert packet[-1] == 0x03

        # Decode should recover 256 bytes
        decoded, remaining = decode_packet(packet)
        assert decoded == payload
        assert len(remaining) == 0

    def test_long_packet_encoding_and_decoding(self):
        """Long packet (> 256 bytes): 0x03, len_hi, len_lo, payload, crc_hi, crc_lo, 0x03."""
        payload = b"X" * 350
        packet = encode_packet(payload)
        assert packet[0] == 0x03
        plen = (packet[1] << 8) | packet[2]
        assert plen == 350
        assert packet[-1] == 0x03

        decoded, remaining = decode_packet(packet)
        assert decoded == payload
        assert len(remaining) == 0

    def test_stream_with_preceding_noise(self):
        """Decoder must discard noise bytes until valid frame start."""
        payload = bytes([COMM_GET_VALUES])
        valid_packet = encode_packet(payload)
        stream = b"\xFF\xAA\x00\x12" + valid_packet
        decoded, remaining = decode_packet(stream)
        assert decoded == payload
        assert len(remaining) == 0

    def test_incomplete_packet_returns_none(self):
        """Incomplete packet must return (None, buffer) without discarding valid bytes."""
        payload = b"Hello VESC"
        packet = encode_packet(payload)
        truncated = packet[:5]
        decoded, remaining = decode_packet(truncated)
        assert decoded is None
        assert remaining == truncated

    def test_corrupt_crc_rejected(self):
        """Packet with corrupted CRC must be rejected."""
        payload = bytes([COMM_GET_VALUES])
        packet = bytearray(encode_packet(payload))
        packet[3] ^= 0xFF  # Corrupt CRC byte
        decoded, remaining = decode_packet(bytes(packet))
        assert decoded is None

    def test_corrupt_stop_byte_rejected(self):
        """Packet with non-0x03 stop byte must be rejected."""
        payload = bytes([COMM_GET_VALUES])
        packet = bytearray(encode_packet(payload))
        packet[-1] = 0xAA  # Corrupt stop byte
        decoded, remaining = decode_packet(bytes(packet))
        assert decoded is None


# =====================================================================
# 3. Command Encoding Tests
# =====================================================================
class TestCommandEncoders:
    def test_encode_comm_get_values(self):
        assert encode_comm_get_values() == bytes([4])

    def test_encode_comm_set_duty(self):
        payload = encode_comm_set_duty(0.5)
        assert payload[0] == COMM_SET_DUTY
        val = struct.unpack('>i', payload[1:5])[0]
        assert val == 50000

    def test_encode_comm_set_current(self):
        payload = encode_comm_set_current(12.5)
        assert payload[0] == COMM_SET_CURRENT
        val = struct.unpack('>i', payload[1:5])[0]
        assert val == 12500

    def test_encode_comm_set_current_brake(self):
        payload = encode_comm_set_current_brake(-5.0)
        assert payload[0] == 7  # COMM_SET_CURRENT_BRAKE
        val = struct.unpack('>i', payload[1:5])[0]
        assert val == 5000  # Stored positive magnitude

    def test_encode_comm_set_rpm(self):
        payload = encode_comm_set_rpm(24000)
        assert payload[0] == COMM_SET_RPM
        val = struct.unpack('>i', payload[1:5])[0]
        assert val == 24000

    def test_encode_comm_reboot(self):
        assert encode_comm_reboot() == bytes([COMM_REBOOT])

    def test_encode_comm_alive(self):
        assert encode_comm_alive() == bytes([30])


# =====================================================================
# 4. Telemetry Decoding Tests (COMM_GET_VALUES)
# =====================================================================
class TestTelemetryDecoding:
    def test_telemetry_roundtrip(self):
        mock_data = {
            'temp_mos': 42.5,
            'temp_motor': 35.1,
            'current_motor': 7.85,
            'current_in': 4.92,
            'id': 0.25,
            'iq': 7.82,
            'duty_now': 0.450,
            'rpm': 24000,
            'v_in': 24.85,
            'amphours': 1.5020,
            'amphours_charged': 0.0510,
            'watt_hours': 37.2500,
            'watt_hours_charged': 1.2500,
            'tachometer': 12500,
            'tachometer_abs': 15200,
            'fault_code': 0
        }
        payload = encode_telemetry_payload(mock_data, include_cmd_id=True)
        assert payload[0] == COMM_GET_VALUES
        assert len(payload) == 54

        decoded = decode_telemetry(payload)
        assert decoded['temp_mos'] == pytest.approx(42.5, abs=0.1)
        assert decoded['temp_motor'] == pytest.approx(35.1, abs=0.1)
        assert decoded['current_motor'] == pytest.approx(7.85, abs=0.01)
        assert decoded['current_in'] == pytest.approx(4.92, abs=0.01)
        assert decoded['id'] == pytest.approx(0.25, abs=0.01)
        assert decoded['iq'] == pytest.approx(7.82, abs=0.01)
        assert decoded['duty_now'] == pytest.approx(0.450, abs=0.001)
        assert decoded['rpm'] == 24000
        assert decoded['v_in'] == pytest.approx(24.85, abs=0.1)
        assert decoded['amphours'] == pytest.approx(1.5020, abs=0.0001)
        assert decoded['fault_code'] == 0
        assert decoded['fault_str'] == "FAULT_CODE_NONE"

    def test_fault_code_translation(self):
        mock_data = {'fault_code': 1}
        payload = encode_telemetry_payload(mock_data, include_cmd_id=True)
        decoded = decode_telemetry(payload)
        assert decoded['fault_code'] == 1
        assert decoded['fault_str'] == "FAULT_CODE_OVER_VOLTAGE"

    def test_payload_too_short_raises_value_error(self):
        short_buf = bytes([COMM_GET_VALUES, 0x01, 0x02])
        with pytest.raises(ValueError):
            decode_telemetry(short_buf)


# =====================================================================
# 5. Profiles and XML Schema Compliance Tests
# =====================================================================
class TestProfilesAndXML:
    @pytest.fixture
    def profiles_dir(self):
        return VESC_DIR / "profiles"

    def test_bench_profile_json(self, profiles_dir):
        path = profiles_dir / "v4006_vesc_bench_5a.json"
        assert path.exists()
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        assert data["motor_parameters"]["poles"] == 24
        assert data["motor_parameters"]["pole_pairs"] == 12
        assert data["current_limits"]["l_current_max"] == 8.0
        assert data["current_limits"]["l_in_current_max"] == 5.0
        assert data["current_limits"]["l_current_min"] == 0.0  # Freewheel, no reverse
        assert data["speed_limits"]["min_mech_rpm"] == 1000
        assert data["speed_limits"]["max_mech_rpm"] == 6000
        assert data["speed_limits"]["l_max_erpm"] == 72000.0
        assert data["speed_limits"]["s_pid_min_erpm"] == 1800.0
        assert data["openloop_settings"]["foc_openloop_rpm"] == 3000.0
        assert data["hfi_settings"]["foc_hfi_voltage_start"] == 6.0
        assert data["hfi_settings"]["foc_sl_erpm_hfi"] == 3000.0
        assert data["can_uavcan"]["controller_id"] == 103
        assert data["can_uavcan"]["uavcan_raw_rpm_max"] == 72000.0

    def test_flight_profile_json(self, profiles_dir):
        path = profiles_dir / "v4006_vesc_flight_25a.json"
        assert path.exists()
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        assert data["current_limits"]["l_current_max"] == 25.0
        assert data["current_limits"]["l_in_current_max"] == 20.0
        assert data["current_limits"]["l_current_min"] == -5.0
        assert data["speed_limits"]["min_mech_rpm"] == 1000
        assert data["speed_limits"]["max_mech_rpm"] == 6000
        assert data["speed_limits"]["l_max_erpm"] == 72000.0
        assert data["speed_limits"]["s_pid_min_erpm"] == 1800.0
        assert data["openloop_settings"]["foc_openloop_rpm"] == 3000.0

    def test_motor_config_xml(self, profiles_dir):
        path = profiles_dir / "v4006_motor_config.xml"
        assert path.exists()
        tree = ET.parse(path)
        root = tree.getroot()
        assert root.tag == "MCConfiguration"
        elem_map = {child.tag: child.text for child in root}

        assert elem_map["motor_type"] == "2"
        assert float(elem_map["l_current_max"]) == 8.0
        assert float(elem_map["l_in_current_max"]) == 5.0
        assert float(elem_map["l_max_erpm"]) == 72000.0
        assert float(elem_map["s_pid_min_erpm"]) == 1800.0
        assert float(elem_map["foc_openloop_rpm"]) == 3000.0
        assert float(elem_map["foc_sl_openloop_time_ramp"]) == 0.3
        assert float(elem_map["foc_f_zv"]) == 24000.0
        assert float(elem_map["foc_dt_us"]) == 0.12
        assert float(elem_map["foc_sl_erpm_hfi"]) == 3000.0
        assert int(elem_map["si_motor_poles"]) == 24

    def test_app_config_xml(self, profiles_dir):
        path = profiles_dir / "v4006_app_uavcan.xml"
        assert path.exists()
        tree = ET.parse(path)
        root = tree.getroot()
        assert root.tag == "APPConfiguration"
        elem_map = {child.tag: child.text for child in root}

        assert elem_map["controller_id"] == "103"
        assert elem_map["can_baud_rate"] == "3"  # CAN_BAUD_1M
        assert elem_map["can_mode"] in ("1", "4")       # CAN_MODE_UAVCAN (1) or CAN_MODE_VESC_UAVCAN (4)
        assert elem_map["uavcan_esc_index"] == "0"
        assert elem_map["uavcan_raw_mode"] in ("2", "3")  # Duty (2) or RPM (3)
        assert float(elem_map["uavcan_raw_rpm_min"]) == 12000.0
        assert float(elem_map["uavcan_raw_rpm_max"]) == 72000.0
        assert elem_map["send_can_status_rate_hz"] == "50"


# =====================================================================
# 6. Mock Serial and Interface Tests
# =====================================================================
class MockSerialPort:
    def __init__(self):
        self.is_open = True
        self.written_bytes = bytearray()
        self.read_buffer = bytearray()

    def write(self, data: bytes):
        self.written_bytes.extend(data)
        return len(data)

    def read(self, size: int = 1):
        chunk = self.read_buffer[:size]
        self.read_buffer = self.read_buffer[size:]
        return bytes(chunk)

    @property
    def in_waiting(self):
        return len(self.read_buffer)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.read_buffer.clear()

    def reset_output_buffer(self):
        self.written_bytes.clear()

    def close(self):
        self.is_open = False


class TestVESCInterfaceMocked:
    def test_interface_send_set_rpm(self):
        iface = VESCInterface(port="COM33")
        mock_port = MockSerialPort()
        iface.ser = mock_port

        iface.set_rpm(12000)
        decoded, _ = decode_packet(bytes(mock_port.written_bytes))
        assert decoded is not None
        assert decoded[0] == COMM_SET_RPM
        val = struct.unpack('>i', decoded[1:5])[0]
        assert val == 12000

    def test_interface_get_telemetry_success(self):
        iface = VESCInterface(port="COM33")
        mock_port = MockSerialPort()
        iface.ser = mock_port

        mock_data = {
            'temp_mos': 30.0,
            'temp_motor': 28.0,
            'current_motor': 2.5,
            'current_in': 1.2,
            'rpm': 14400,
            'v_in': 22.2,
            'fault_code': 0
        }
        telem_payload = encode_telemetry_payload(mock_data, include_cmd_id=True)
        telem_packet = encode_packet(telem_payload)
        mock_port.read_buffer.extend(telem_packet)

        telem = iface.get_telemetry(timeout=0.1)
        assert telem is not None
        assert telem['rpm'] == 14400
        assert telem['v_in'] == pytest.approx(22.2, abs=0.1)

    def test_slcan_send_frame(self):
        slcan = VESCSLCANInterface(port="COM16", node_id=103, esc_index=0)
        mock_port = MockSerialPort()
        slcan.ser = mock_port

        slcan.send_can_frame(0x107D5567, b"\x01\x02\x03\x04", is_extended=True)
        expected = b"T107D5567401020304\r"
        assert bytes(mock_port.written_bytes) == expected

    def test_slcan_set_rpm(self):
        slcan = VESCSLCANInterface(port="COM16", node_id=103, esc_index=0)
        mock_port = MockSerialPort()
        slcan.ser = mock_port

        slcan.set_rpm(2000)
        # Should generate both VESC Native CAN and DroneCAN frames
        sent = bytes(mock_port.written_bytes)
        assert b"T" in sent
        assert b"\r" in sent

    def test_slcan_send_raw_command(self):
        slcan = VESCSLCANInterface(port="COM16", node_id=103, esc_index=0)
        mock_port = MockSerialPort()
        slcan.ser = mock_port

        # 5% throttle -> raw = 409 (0x0199) -> TAO: Byte 0 = 0x99, Byte 1 = 0x04, Tail = 0xC0
        slcan.send_raw_command(0.05)
        sent = bytes(mock_port.written_bytes)
        # SLCAN format: T<can_id:8><dlc:1><data_hex>\r
        # Can ID: (20 << 24) | (1030 << 8) | 127 = 0x1404067F
        assert sent.startswith(b"T1404067F3")
        assert b"9904C0" in sent or b"9904c0" in sent.lower()

    def test_slcan_stop(self):
        slcan = VESCSLCANInterface(port="COM16", node_id=103, esc_index=0)
        mock_port = MockSerialPort()
        slcan.ser = mock_port

        slcan.stop()
        sent = bytes(mock_port.written_bytes)
        # Must send 0 throttle RawCommand (0000C0) and RPM 0
        assert b"T1404067F30000C0\r" in sent

    def test_smart_depa_usb_two_phase(self):
        iface = VESCInterface(port="COM33", smart_depa=True)
        mock_port = MockSerialPort()
        iface.ser = mock_port

        # Phase 1: Call at t=0 (<0.85s) should clamp to 250 Mech RPM (3000 ERPM)
        iface.set_mech_rpm(1000)
        decoded, _ = decode_packet(bytes(mock_port.written_bytes))
        assert decoded is not None
        assert decoded[0] == COMM_SET_RPM
        val = struct.unpack('>i', decoded[1:5])[0]
        assert val == 3000  # Clamped to 250 Mech RPM * 12

        # Phase 2: Simulate elapsed >= 0.85s
        mock_port.written_bytes.clear()
        iface._spinup_start_time = 0.0  # Force elapsed > 0.85s
        iface.set_mech_rpm(1000)
        decoded, _ = decode_packet(bytes(mock_port.written_bytes))
        assert decoded is not None
        val = struct.unpack('>i', decoded[1:5])[0]
        assert val == 12000  # Full setpoint 1000 Mech RPM * 12

        # Stopping resets spinup timer
        iface.stop()
        assert iface._spinup_start_time is None

    def test_smart_depa_slcan_two_phase(self):
        slcan = VESCSLCANInterface(port="COM16", node_id=103, smart_depa=True)
        mock_port = MockSerialPort()
        slcan.ser = mock_port

        # Phase 1: Call at t=0 (<0.85s) should clamp to 250 Mech RPM (3000 ERPM)
        slcan.set_mech_rpm(1500)
        sent = bytes(mock_port.written_bytes)
        # VESC CAN ID: (3 << 8) | 103 = 0x00000367 -> T000003674...
        # 3000 ERPM in hex: 00000BB8
        assert b"T00000367400000BB8\r" in sent

        # Phase 2: Simulate elapsed >= 0.85s
        mock_port.written_bytes.clear()
        slcan._spinup_start_time = 0.0
        slcan.set_mech_rpm(1500)
        sent2 = bytes(mock_port.written_bytes)
        # 18000 ERPM in hex: 00004650
        assert b"T00000367400004650\r" in sent2

        # Stopping resets spinup timer
        slcan.stop()
        assert slcan._spinup_start_time is None

    def test_smart_depa_disabled(self):
        iface = VESCInterface(port="COM33", smart_depa=False)
        mock_port = MockSerialPort()
        iface.ser = mock_port

        iface.set_mech_rpm(1000)
        decoded, _ = decode_packet(bytes(mock_port.written_bytes))
        val = struct.unpack('>i', decoded[1:5])[0]
        assert val == 12000  # Directly 12000 ERPM without de-pa clamping


# =====================================================================
# 7. Linear Throttle Mapping (1000 - 6000 Mechanical RPM) Tests
# =====================================================================
class TestThrottleMapping:
    def test_throttle_deadband_under_1_percent(self):
        assert throttle_to_mech_rpm(0.00) == 0
        assert throttle_to_mech_rpm(0.005) == 0
        assert throttle_to_mech_rpm(0.009) == 0

    def test_throttle_boundary_1_percent(self):
        # 1% throttle must map exactly to 1000 mechanical RPM
        assert throttle_to_mech_rpm(0.01) == 1000
        assert mech_rpm_to_erpm(1000) == 12000

    def test_throttle_low_range_1_to_5_percent(self):
        # Verify 1-5% monotonic increase and exact values
        rpm_1 = throttle_to_mech_rpm(0.01)
        rpm_2 = throttle_to_mech_rpm(0.02)
        rpm_3 = throttle_to_mech_rpm(0.03)
        rpm_4 = throttle_to_mech_rpm(0.04)
        rpm_5 = throttle_to_mech_rpm(0.05)

        assert rpm_1 == 1000
        assert 1045 <= rpm_2 <= 1055
        assert 1095 <= rpm_3 <= 1105
        assert 1145 <= rpm_4 <= 1155
        assert 1195 <= rpm_5 <= 1205

        assert rpm_1 < rpm_2 < rpm_3 < rpm_4 < rpm_5

    def test_throttle_boundary_100_percent(self):
        # 100% throttle must map exactly to 6000 mechanical RPM
        assert throttle_to_mech_rpm(1.00) == 6000
        assert mech_rpm_to_erpm(6000) == 72000

    def test_throttle_midpoint_50_percent(self):
        rpm_50 = throttle_to_mech_rpm(0.50)
        # Expected: 1000 + (0.49 / 0.99) * 5000 = 3474.7 -> 3475
        assert 3470 <= rpm_50 <= 3480

    def test_interface_set_throttle(self):
        iface = VESCInterface(port="COM33")
        mock_port = MockSerialPort()
        iface.ser = mock_port

        # 1% throttle -> 1000 mech RPM -> 12000 ERPM
        mech_rpm = iface.set_throttle(0.01)
        assert mech_rpm == 1000
        sent = bytes(mock_port.written_bytes)
        assert len(sent) > 0
        assert sent[0] == 0x02  # Short packet framing
        assert sent[2] == COMM_SET_RPM
        erpm_val = struct.unpack_from(">i", sent, 3)[0]
        assert erpm_val == 12000

    def test_slcan_set_throttle(self):
        slcan = VESCSLCANInterface(port="COM16", node_id=103, esc_index=0)
        mock_port = MockSerialPort()
        slcan.ser = mock_port

        mech_rpm = slcan.set_throttle(0.05)
        # 5% throttle -> ~1202 mech RPM
        assert 1195 <= mech_rpm <= 1205
        sent = bytes(mock_port.written_bytes)
        assert b"T" in sent
