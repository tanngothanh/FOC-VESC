"""VESC Communication Interface Library.

Provides high-level interfaces for:
1. VESCInterface: Direct USB/UART serial connection using VESC protocol (default COM33).
2. VESCSLCANInterface: CAN bus connection via SLCAN adapter for DroneCAN & VESC CAN (default COM16, Node ID 103, ESC Index 0).
"""

import time
import struct
import logging
from typing import Optional, Dict, Any, Tuple

try:
    import serial
except ImportError:
    serial = None

try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2
except ImportError:
    try:
        from pymavlink.dialects.v10 import ardupilotmega as mavlink2
    except ImportError:
        mavlink2 = None

from .vesc_protocol import (
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
    COMM_FW_VERSION,
    COMM_GET_VALUES,
    COMM_REBOOT,
)

logger = logging.getLogger("VESCInterface")


def throttle_to_mech_rpm(throttle: float, min_rpm: float = 1000.0, max_rpm: float = 6000.0) -> int:
    """Linearly maps throttle (0.0 to 1.0) to mechanical RPM (1000 to 6000 RPM).
    
    Throttle < 0.01 (1%): 0 RPM (stopped).
    Throttle 0.01 to 1.00: linearly maps 1000 to 6000 mechanical RPM.
    """
    if throttle < 0.01:
        return 0
    clamped = min(1.0, max(0.01, float(throttle)))
    rpm = min_rpm + ((clamped - 0.01) / 0.99) * (max_rpm - min_rpm)
    return int(round(rpm))


def mech_rpm_to_erpm(mech_rpm: int, pole_pairs: int = 12) -> int:
    """Converts mechanical RPM to electrical RPM (ERPM) for 12 pole pairs."""
    return int(mech_rpm * pole_pairs)


class VESCInterface:
    """Direct USB/UART interface for VESC controllers."""

    def __init__(self, port: str = "COM33", baudrate: int = 115200, timeout: float = 0.5, smart_depa: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.smart_depa = smart_depa
        self._spinup_start_time: Optional[float] = None
        self.ser: Optional[Any] = None
        self._rx_buffer = bytearray()

    def connect(self) -> bool:
        """Opens serial port connection to VESC."""
        if serial is None:
            raise RuntimeError("pyserial is not installed. Please install pyserial.")
        for attempt in range(3):
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self._rx_buffer.clear()
                # Flush any stale incoming data
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                return True
            except Exception as e:
                if attempt < 2:
                    time.sleep(0.3)
                else:
                    logger.error(f"Failed to connect to VESC on {self.port}: {e}")
        self.ser = None
        return False

    def disconnect(self) -> None:
        """Closes serial connection."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_connected(self) -> bool:
        """Checks if serial port is open."""
        return self.ser is not None and self.ser.is_open

    def send_packet(self, payload: bytes) -> None:
        """Frames and writes packet payload to VESC."""
        if not self.is_connected():
            raise ConnectionError(f"VESC not connected on {self.port}")
        frame = encode_packet(payload)
        self.ser.write(frame)
        self.ser.flush()

    def read_packet(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Reads and extracts next framed packet payload from VESC."""
        if not self.is_connected():
            return None

        deadline = time.time() + (timeout if timeout is not None else self.timeout)
        while time.time() < deadline:
            # Check for existing packet in buffer first
            payload, remaining = decode_packet(bytes(self._rx_buffer))
            self._rx_buffer = bytearray(remaining)
            if payload is not None:
                return payload

            # Read new bytes if available
            waiting = self.ser.in_waiting
            if waiting > 0:
                chunk = self.ser.read(waiting)
                if chunk:
                    self._rx_buffer.extend(chunk)
                    payload, remaining = decode_packet(bytes(self._rx_buffer))
                    self._rx_buffer = bytearray(remaining)
                    if payload is not None:
                        return payload
            else:
                time.sleep(0.002)

        return None

    def ping(self) -> bool:
        """Pings VESC by requesting telemetry or firmware version."""
        if not self.is_connected():
            return False
        try:
            telemetry = self.get_telemetry(timeout=0.3)
            return telemetry is not None
        except Exception:
            return False

    def get_telemetry(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Sends COMM_GET_VALUES and parses returned sensor values."""
        cmd = encode_comm_get_values()
        self.send_packet(cmd)
        resp = self.read_packet(timeout=timeout)
        if resp is None:
            return None
        try:
            return decode_telemetry(resp)
        except Exception as e:
            logger.warning(f"Error decoding telemetry: {e}")
            return None

    def get_fw_version(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Sends COMM_FW_VERSION to query firmware version info."""
        self.send_packet(bytes([COMM_FW_VERSION]))
        resp = self.read_packet(timeout=timeout)
        if resp is None or len(resp) < 3:
            return None
        major = resp[1]
        minor = resp[2]
        hw_name = ""
        if len(resp) > 3:
            hw_name = resp[3:].split(b'\x00')[0].decode('ascii', errors='replace')
        return {"major": major, "minor": minor, "hw_name": hw_name}

    def set_rpm(self, erpm: int) -> None:
        """Sets electrical RPM (ERPM)."""
        self.send_packet(encode_comm_set_rpm(int(erpm)))

    def set_mech_rpm(self, mech_rpm: int) -> None:
        """Sets mechanical RPM (converts to ERPM for 12 pole pairs) with Smart De-Pa.
        
        When starting from 0, commands 250 Mech RPM (3,000 ERPM) for 0.85s to ensure clean
        sensorless FOC observer handover before accelerating to the full target speed.
        """
        mech_rpm = int(mech_rpm)
        if mech_rpm == 0:
            self._spinup_start_time = None
            effective_rpm = 0
        elif self.smart_depa:
            if self._spinup_start_time is None:
                self._spinup_start_time = time.time()
            elapsed = time.time() - self._spinup_start_time
            if elapsed < 0.85:
                effective_rpm = min(mech_rpm, 250)
            else:
                effective_rpm = mech_rpm
        else:
            effective_rpm = mech_rpm

        self.set_rpm(int(effective_rpm) * 12)

    def set_throttle(self, throttle: float, min_rpm: float = 1000.0, max_rpm: float = 6000.0) -> int:
        """Linearly maps throttle (0.0 to 1.0) to mechanical RPM (1000 to 6000) and sends ERPM."""
        mech_rpm = throttle_to_mech_rpm(throttle, min_rpm, max_rpm)
        erpm = mech_rpm_to_erpm(mech_rpm)
        self.set_rpm(erpm)
        return mech_rpm

    def set_duty(self, duty: float) -> None:
        """Sets duty cycle (-1.0 to 1.0)."""
        self.send_packet(encode_comm_set_duty(float(duty)))

    def set_current(self, current: float) -> None:
        """Sets motor current in Amps."""
        self.send_packet(encode_comm_set_current(float(current)))

    def set_current_brake(self, current: float) -> None:
        """Sets braking current in Amps."""
        self.send_packet(encode_comm_set_current_brake(float(current)))

    def stop(self) -> None:
        """Safely stops motor (current = 0, duty = 0)."""
        self._spinup_start_time = None
        if self.is_connected():
            try:
                self.set_current(0.0)
                self.set_duty(0.0)
            except Exception:
                pass

    def reboot(self) -> None:
        """Sends reboot command to VESC."""
        if self.is_connected():
            self.send_packet(encode_comm_reboot())

    def send_alive(self) -> None:
        """Sends keepalive command."""
        if self.is_connected():
            self.send_packet(encode_comm_alive())


class VESCSLCANInterface:
    """SLCAN interface for DroneCAN and VESC CAN communication.
    
    Supports:
    - Port: default COM16 (SLCAN adapter)
    - Node ID: 103 (VESC)
    - ESC Index: 0
    - DroneCAN ESC RPMCommand (Msg ID 1031)
    - DroneCAN ESC Status (Msg ID 1034)
    - VESC native CAN commands (SET_RPM, STATUS)
    """

    CAN_PACKET_SET_DUTY = 0
    CAN_PACKET_SET_CURRENT = 1
    CAN_PACKET_SET_CURRENT_BRAKE = 2
    CAN_PACKET_SET_RPM = 3
    CAN_PACKET_STATUS = 9
    CAN_PACKET_STATUS_4 = 16
    CAN_PACKET_STATUS_5 = 27

    def __init__(self, port: str = "COM16", baudrate: int = 115200,
                 node_id: int = 103, esc_index: int = 0, timeout: float = 0.5,
                 smart_depa: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.node_id = node_id
        self.esc_index = esc_index
        self.timeout = timeout
        self.smart_depa = smart_depa
        self._spinup_start_time: Optional[float] = None
        self.ser: Optional[Any] = None
        self._transfer_id = 0
        self._is_mavlink = False
        self._mav = mavlink2.MAVLink(None) if mavlink2 is not None else None
        self._last_telemetry: Dict[str, Any] = {
            'temp_mos': 0.0,
            'temp_motor': 0.0,
            'current_motor': 0.0,
            'current_in': 0.0,
            'duty_now': 0.0,
            'rpm': 0,
            'v_in': 0.0,
            'fault_code': 0,
            'fault_str': 'FAULT_CODE_NONE'
        }

    def connect(self) -> bool:
        """Initializes SLCAN adapter or MAVLink CAN bridge (e.g. Durandal/Pixhawk)."""
        if serial is None:
            raise RuntimeError("pyserial is not installed.")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(0.05)
            # Check for MAVLink magic (0xFD for MAVLink 2, 0xFE for MAVLink 1)
            initial = self.ser.read(128)
            if b"\xfd" in initial or b"\xfe" in initial:
                self._is_mavlink = True
                logger.info(f"Detected MAVLink stream on {self.port}")
            else:
                self._is_mavlink = False
                # Send close, set baud S8 (1M), open for SLCAN
                self.ser.write(b"C\r")
                time.sleep(0.02)
                self.ser.write(b"S8\r")
                time.sleep(0.02)
                self.ser.write(b"O\r")
                time.sleep(0.02)
                self.ser.reset_input_buffer()
            return True
        except Exception as e:
            logger.error(f"Failed to connect SLCAN on {self.port}: {e}")
            self.ser = None
            return False

    def disconnect(self) -> None:
        """Closes SLCAN channel and port."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"C\r")
                time.sleep(0.02)
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send_can_frame(self, can_id: int, data: bytes, is_extended: bool = True) -> None:
        """Sends CAN frame via SLCAN ASCII protocol or MAVLink CAN bridge."""
        if not self.is_connected():
            raise ConnectionError(f"SLCAN not connected on {self.port}")

        if self._is_mavlink and self._mav is not None:
            raw_id = (can_id | 0x80000000) if is_extended else can_id
            padded = list(data) + [0] * (8 - len(data))
            try:
                # Transmit over CAN bus 0 and bus 1 for reliability
                self._mav.can_frame_send(1, 1, 0, len(data), raw_id, padded[:8])
                self._mav.can_frame_send(1, 1, 1, len(data), raw_id, padded[:8])
            except Exception as e:
                logger.warning(f"Failed to send CAN frame via MAVLink: {e}")
        else:
            dlc = len(data)
            data_hex = data.hex().upper()
            if is_extended:
                cmd = f"T{can_id:08X}{dlc}{data_hex}\r"
            else:
                cmd = f"t{can_id:03X}{dlc}{data_hex}\r"
            self.ser.write(cmd.encode("ascii"))
            self.ser.flush()

    def set_mech_rpm(self, mech_rpm: int) -> None:
        """Sends mechanical RPM command over CAN using both DroneCAN and VESC Native CAN.
        
        When starting from 0, holds 250 Mech RPM for 0.85s (Smart De-Pa) to ensure clean
        sensorless FOC observer handover before accelerating to the full target speed.
        """
        mech_rpm = int(mech_rpm)
        if mech_rpm == 0:
            self._spinup_start_time = None
            effective_rpm = 0
        elif self.smart_depa:
            if self._spinup_start_time is None:
                self._spinup_start_time = time.time()
            elapsed = time.time() - self._spinup_start_time
            if elapsed < 0.85:
                effective_rpm = min(mech_rpm, 250)
            else:
                effective_rpm = mech_rpm
        else:
            effective_rpm = mech_rpm

        erpm = effective_rpm * 12

        # 1. Send VESC Native CAN Packet SET_RPM (extended 29-bit CAN frame)
        vesc_can_id = (self.CAN_PACKET_SET_RPM << 8) | (self.node_id & 0xFF)
        vesc_data = struct.pack(">i", int(erpm))
        self.send_can_frame(vesc_can_id, vesc_data, is_extended=True)

        # 2. Send DroneCAN uavcan.equipment.esc.RPMCommand (Msg ID 1031)
        # Priority: 20 (0x14), Msg ID: 1031 (0x0407), Source Node: 127
        # Note: VESC canard_driver.c passes rpm_val directly to mc_interface_set_pid_speed, expecting ERPM
        dronecan_id = (20 << 24) | (1031 << 8) | 127
        val = int(erpm) & 0x3FFFF
        bitfield = (1 & 0x1F) | (val << 5)
        b0 = bitfield & 0xFF
        b1 = (bitfield >> 8) & 0xFF
        b2 = (bitfield >> 16) & 0xFF
        tail = 0xC0 | (self._transfer_id & 0x1F)
        self._transfer_id = (self._transfer_id + 1) % 32
        dronecan_data = bytes([b0, b1, b2, tail])
        self.send_can_frame(dronecan_id, dronecan_data, is_extended=True)

    def set_rpm(self, rpm_val: int) -> None:
        """Sends RPM command over CAN. Automatically converts ERPM to Mech RPM if > 10000."""
        if abs(rpm_val) > 10000:
            mech_rpm = int(rpm_val) // 12
        else:
            mech_rpm = int(rpm_val)
        self.set_mech_rpm(mech_rpm)

    def set_throttle(self, throttle: float, min_rpm: float = 1000.0, max_rpm: float = 6000.0) -> int:
        """Linearly maps throttle (0.0 to 1.0) to mechanical RPM (1000 to 6000) and sends over CAN."""
        mech_rpm = throttle_to_mech_rpm(throttle, min_rpm, max_rpm)
        self.set_rpm(mech_rpm)
        return mech_rpm

    def send_raw_command(self, raw_throttle: float) -> None:
        """Sends DroneCAN uavcan.equipment.esc.RawCommand (Msg ID 1030).
        
        raw_throttle: 0.0 to 1.0 normalized thrust command.
        Uses Tail Array Optimization (TAO) conforming to DroneCAN DSDL specification:
        - Byte 0: lower 8 bits of 14-bit scalar
        - Byte 1: upper 6 bits shifted by 2 (canardDecodeScalar format)
        - Byte 2: Tail byte (0xC0 | transfer_id)
        """
        # DroneCAN Msg ID 1030 (0x0406), Priority 20 (0x14), Source Node 127
        dronecan_id = (20 << 24) | (1030 << 8) | 127
        val = int(max(0.0, min(1.0, float(raw_throttle))) * 8191.0) & 0x3FFF
        b0 = val & 0xFF
        b1 = ((val >> 8) & 0x3F) << 2
        tail = 0xC0 | (self._transfer_id & 0x1F)
        self._transfer_id = (self._transfer_id + 1) % 32
        dronecan_data = bytes([b0, b1, tail])
        self.send_can_frame(dronecan_id, dronecan_data, is_extended=True)

    def stop(self) -> None:
        """Safely stops motor via CAN (sends zero RawCommand and zero RPM)."""
        self._spinup_start_time = None
        self.send_raw_command(0.0)
        self.set_rpm(0)

    def get_telemetry(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Reads incoming CAN frames from SLCAN or MAVLink bridge and extracts telemetry."""
        if not self.is_connected():
            return None

        deadline = time.time() + (timeout if timeout is not None else self.timeout)
        found = False

        while time.time() < deadline:
            if self._is_mavlink and self._mav is not None:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                for b in chunk:
                    try:
                        m = self._mav.parse_char(bytes([b]))
                        if m and m.get_msgId() == 11030:  # ESC_TELEMETRY_1_TO_4
                            idx = self.esc_index
                            if idx < len(m.voltage):
                                self._last_telemetry['v_in'] = round(m.voltage[idx] / 100.0, 2)
                                self._last_telemetry['current_motor'] = round(m.current[idx] / 100.0, 2)
                                self._last_telemetry['temp_mos'] = float(m.temperature[idx])
                                self._last_telemetry['rpm'] = int(m.rpm[idx])
                                found = True
                                break
                    except Exception:
                        pass
            else:
                line = self.ser.read_until(b'\r').decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("T") and len(line) >= 10:
                    try:
                        can_id = int(line[1:9], 16)
                        dlc = int(line[9:10])
                        payload = bytes.fromhex(line[10:10 + dlc * 2])

                        # Check VESC CAN Status 1 (0x09)
                        cmd = (can_id >> 8) & 0xFF
                        node = can_id & 0xFF
                        if node == self.node_id and cmd == self.CAN_PACKET_STATUS and len(payload) >= 8:
                            erpm = struct.unpack_from(">i", payload, 0)[0]
                            current = struct.unpack_from(">h", payload, 4)[0] / 10.0
                            duty = struct.unpack_from(">h", payload, 6)[0] / 1000.0
                            self._last_telemetry['rpm'] = erpm
                            self._last_telemetry['current_motor'] = current
                            self._last_telemetry['duty_now'] = duty
                            found = True

                        # Check VESC CAN Status 4 (0x10)
                        elif node == self.node_id and cmd == self.CAN_PACKET_STATUS_4 and len(payload) >= 6:
                            temp_fet = struct.unpack_from(">h", payload, 0)[0] / 10.0
                            temp_motor = struct.unpack_from(">h", payload, 2)[0] / 10.0
                            current_in = struct.unpack_from(">h", payload, 4)[0] / 10.0
                            self._last_telemetry['temp_mos'] = temp_fet
                            self._last_telemetry['temp_motor'] = temp_motor
                            self._last_telemetry['current_in'] = current_in
                            found = True

                        # Check VESC CAN Status 5 (0x1B)
                        elif node == self.node_id and cmd == self.CAN_PACKET_STATUS_5 and len(payload) >= 6:
                            v_in = struct.unpack_from(">h", payload, 4)[0] / 10.0
                            self._last_telemetry['v_in'] = v_in
                            found = True

                        # Check DroneCAN Status Message (Msg ID 1034 = 0x040A)
                        msg_id = (can_id >> 8) & 0xFFFF
                        src_node = can_id & 0x7F
                        if src_node == self.node_id and msg_id == 1034:
                            found = True

                    except Exception:
                        pass

                if found:
                    return dict(self._last_telemetry)

        return dict(self._last_telemetry) if found else None
