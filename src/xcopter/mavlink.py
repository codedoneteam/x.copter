import os
import select
import socket
import struct
import time
from types import SimpleNamespace

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None


class MAVLinkError(Exception):
    pass


class mavlink:
    MAV_AUTOPILOT_INVALID = 8
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_CMD_CONDITION_YAW = 115
    MAV_CMD_DO_BUZZER = 30001
    MAV_CMD_DO_SET_MODE = 176
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_FRAME_GLOBAL = 0
    MAV_FRAME_LOCAL_NED = 1
    MAV_FRAME_BODY_NED = 8
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_PARAM_TYPE_INT32 = 6
    MAV_PARAM_TYPE_REAL32 = 9
    MAV_RESULT_ACCEPTED = 0
    MAV_RESULT_FAILED = 4
    MAV_SEVERITY_CRITICAL = 2
    MAV_SEVERITY_ERROR = 3
    MAV_SEVERITY_WARNING = 4
    MAV_SEVERITY_INFO = 6
    MAV_SEVERITY_DEBUG = 7
    MAV_TYPE_GCS = 6


ARDUCOPTER_MODES = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "DRIFT": 11,
    "SPORT": 13,
    "FLIP": 14,
    "AUTOTUNE": 15,
    "POSHOLD": 16,
    "BRAKE": 17,
    "THROW": 18,
    "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20,
    "SMART_RTL": 21,
    "FLOWHOLD": 22,
    "FOLLOW": 23,
    "ZIGZAG": 24,
    "SYSTEMID": 25,
    "AUTOROTATE": 26,
    "AUTO_RTL": 27,
}

_ARDUCOPTER_MODES_BY_ID = {mode_id: name for name, mode_id in ARDUCOPTER_MODES.items()}


class MAVLinkMessage(SimpleNamespace):
    def __init__(self, msg_type, msg_id, sysid, compid, **fields):
        super().__init__(**fields)
        self._type = msg_type
        self._msg_id = msg_id
        self._sysid = sysid
        self._compid = compid

    def get_type(self):
        return self._type


def mode_string_v10(heartbeat):
    return _ARDUCOPTER_MODES_BY_ID.get(
        getattr(heartbeat, "custom_mode", None),
        f"base_mode={getattr(heartbeat, 'base_mode', None)}, custom_mode={getattr(heartbeat, 'custom_mode', None)}",
    )


def mavlink_connection(connection_string, baud=115200, source_system=255, source_component=0):
    transport = _open_transport(connection_string, baud)
    return MAVLinkConnection(transport, source_system, source_component)


class MAVLinkConnection:
    def __init__(self, transport, source_system=255, source_component=0):
        self.transport = transport
        self.source_system = source_system
        self.source_component = source_component
        self.target_system = 0
        self.target_component = 0
        self.mav = MAVLinkSender(self)
        self._seq = 0
        self._buffer = bytearray()

    def mode_mapping(self):
        return dict(ARDUCOPTER_MODES)

    def set_mode(self, mode_id):
        self.mav.set_mode_send(
            self.target_system,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def wait_heartbeat(self, timeout=None):
        msg = self.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if msg is None:
            raise MAVLinkError("HEARTBEAT not received")
        return msg

    def recv_match(self, type=None, blocking=False, timeout=None):
        types = _normalize_type_filter(type)
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            msg = self._next_message()
            if msg is not None:
                if not self.target_system:
                    self.target_system = msg._sysid
                if not self.target_component:
                    self.target_component = msg._compid
                if types is None or msg.get_type() in types:
                    return msg

            if not blocking:
                return None

            read_timeout = None
            if deadline is not None:
                read_timeout = max(0.0, deadline - time.monotonic())
                if read_timeout <= 0:
                    return None

            chunk = self.transport.read(1, read_timeout)
            if not chunk:
                return None
            self._buffer.extend(chunk)

    def write(self, packet):
        self.transport.write(packet)

    def close(self):
        self.transport.close()

    def _send_message(self, msg_id, payload, crc_extra):
        packet = self._pack_v1(msg_id, payload, crc_extra)
        self.write(packet)

    def _pack_v1(self, msg_id, payload, crc_extra):
        seq = self._seq
        self._seq = (self._seq + 1) % 256
        header = struct.pack(
            "<BBBBB",
            len(payload),
            seq,
            self.source_system,
            self.source_component,
            msg_id,
        )
        checksum = _x25_crc(header + payload, crc_extra)
        return b"\xfe" + header + payload + struct.pack("<H", checksum)

    def _next_message(self):
        while True:
            if not self._buffer:
                return None

            if self._buffer[0] not in (0xFE, 0xFD):
                del self._buffer[0]
                continue

            stx = self._buffer[0]
            min_header = 6 if stx == 0xFE else 10
            if len(self._buffer) < min_header:
                return None

            payload_len = self._buffer[1]
            signature_len = 13 if stx == 0xFD and (self._buffer[2] & 0x01) else 0
            packet_len = (8 + payload_len) if stx == 0xFE else (12 + payload_len + signature_len)
            if len(self._buffer) < packet_len:
                return None

            packet = bytes(self._buffer[:packet_len])
            del self._buffer[:packet_len]

            try:
                return _parse_packet(packet)
            except MAVLinkError:
                continue


class MAVLinkSender:
    def __init__(self, connection):
        self.connection = connection

    def heartbeat_send(self, mav_type, autopilot, base_mode, custom_mode, system_status, mavlink_version=3):
        payload = struct.pack("<IBBBBB", custom_mode, mav_type, autopilot, base_mode, system_status, mavlink_version)
        self.connection._send_message(0, payload, 50)

    def command_long_send(
        self,
        target_system,
        target_component,
        command,
        confirmation,
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7,
    ):
        payload = struct.pack(
            "<fffffffHBBB",
            float(param1),
            float(param2),
            float(param3),
            float(param4),
            float(param5),
            float(param6),
            float(param7),
            int(command),
            int(target_system),
            int(target_component),
            int(confirmation),
        )
        self.connection._send_message(76, payload, 152)

    def set_mode_send(self, target_system, base_mode, custom_mode):
        payload = struct.pack("<IBB", int(custom_mode), int(target_system), int(base_mode))
        self.connection._send_message(11, payload, 89)

    def param_request_read_send(self, target_system, target_component, param_id, param_index):
        payload = struct.pack(
            "<hBB16s",
            int(param_index),
            int(target_system),
            int(target_component),
            _fixed_bytes(param_id, 16),
        )
        self.connection._send_message(20, payload, 214)

    def param_set_send(self, target_system, target_component, param_id, param_value, param_type):
        payload = struct.pack(
            "<fBB16sB",
            float(param_value),
            int(target_system),
            int(target_component),
            _fixed_bytes(param_id, 16),
            int(param_type),
        )
        self.connection._send_message(23, payload, 168)

    def set_position_target_local_ned_send(
        self,
        time_boot_ms,
        target_system,
        target_component,
        coordinate_frame,
        type_mask,
        x,
        y,
        z,
        vx,
        vy,
        vz,
        afx,
        afy,
        afz,
        yaw,
        yaw_rate,
    ):
        payload = struct.pack(
            "<IfffffffffffHBBB",
            int(time_boot_ms),
            float(x),
            float(y),
            float(z),
            float(vx),
            float(vy),
            float(vz),
            float(afx),
            float(afy),
            float(afz),
            float(yaw),
            float(yaw_rate),
            int(type_mask),
            int(target_system),
            int(target_component),
            int(coordinate_frame),
        )
        self.connection._send_message(84, payload, 143)

    def set_position_target_global_int_send(
        self,
        time_boot_ms,
        target_system,
        target_component,
        coordinate_frame,
        type_mask,
        lat_int,
        lon_int,
        alt,
        vx,
        vy,
        vz,
        afx,
        afy,
        afz,
        yaw,
        yaw_rate,
    ):
        payload = struct.pack(
            "<IiifffffffffHBBB",
            int(time_boot_ms),
            int(lat_int),
            int(lon_int),
            float(alt),
            float(vx),
            float(vy),
            float(vz),
            float(afx),
            float(afy),
            float(afz),
            float(yaw),
            float(yaw_rate),
            int(type_mask),
            int(target_system),
            int(target_component),
            int(coordinate_frame),
        )
        self.connection._send_message(86, payload, 5)

    def statustext_send(self, severity, text):
        payload = struct.pack("<B50s", int(severity), _fixed_bytes(text, 50))
        self.connection._send_message(253, payload, 83)


def _parse_packet(packet):
    stx = packet[0]
    payload_len = packet[1]

    if stx == 0xFE:
        header = packet[1:6]
        sysid = packet[3]
        compid = packet[4]
        msg_id = packet[5]
        payload_start = 6
    elif stx == 0xFD:
        incompat_flags = packet[2]
        if incompat_flags & ~0x01:
            raise MAVLinkError("unsupported MAVLink2 incompatibility flags")
        header = packet[1:10]
        sysid = packet[5]
        compid = packet[6]
        msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16)
        payload_start = 10
    else:
        raise MAVLinkError("invalid MAVLink start byte")

    payload = packet[payload_start : payload_start + payload_len]
    received_crc = struct.unpack("<H", packet[payload_start + payload_len : payload_start + payload_len + 2])[0]
    crc_extra = _CRC_EXTRA.get(msg_id)
    if crc_extra is not None and _x25_crc(header + payload, crc_extra) != received_crc:
        raise MAVLinkError("invalid MAVLink checksum")

    return _decode_message(msg_id, sysid, compid, payload)


def _decode_message(msg_id, sysid, compid, payload):
    if msg_id == 0:
        custom_mode, mav_type, autopilot, base_mode, system_status, mavlink_version = _unpack("<IBBBBB", payload)
        return MAVLinkMessage(
            "HEARTBEAT",
            msg_id,
            sysid,
            compid,
            custom_mode=custom_mode,
            type=mav_type,
            autopilot=autopilot,
            base_mode=base_mode,
            system_status=system_status,
            mavlink_version=mavlink_version,
        )

    if msg_id == 1:
        values = _unpack("<IIIHHhHHHHHHB", payload)
        return MAVLinkMessage(
            "SYS_STATUS",
            msg_id,
            sysid,
            compid,
            onboard_control_sensors_present=values[0],
            onboard_control_sensors_enabled=values[1],
            onboard_control_sensors_health=values[2],
            load=values[3],
            voltage_battery=values[4],
            current_battery=values[5],
            drop_rate_comm=values[6],
            errors_comm=values[7],
            errors_count1=values[8],
            errors_count2=values[9],
            errors_count3=values[10],
            errors_count4=values[11],
            battery_remaining=_int8(values[12]),
        )

    if msg_id == 22:
        param_value, param_count, param_index, param_id, param_type = _unpack("<fHH16sB", payload)
        return MAVLinkMessage(
            "PARAM_VALUE",
            msg_id,
            sysid,
            compid,
            param_value=param_value,
            param_count=param_count,
            param_index=param_index,
            param_id=param_id,
            param_type=param_type,
        )

    if msg_id == 30:
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = _unpack("<Iffffff", payload)
        return MAVLinkMessage(
            "ATTITUDE",
            msg_id,
            sysid,
            compid,
            time_boot_ms=time_boot_ms,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rollspeed=rollspeed,
            pitchspeed=pitchspeed,
            yawspeed=yawspeed,
        )

    if msg_id == 32:
        time_boot_ms, x, y, z, vx, vy, vz = _unpack("<Iffffff", payload)
        return MAVLinkMessage(
            "LOCAL_POSITION_NED",
            msg_id,
            sysid,
            compid,
            time_boot_ms=time_boot_ms,
            x=x,
            y=y,
            z=z,
            vx=vx,
            vy=vy,
            vz=vz,
        )

    if msg_id == 33:
        values = _unpack("<IiiiihhhH", payload)
        return MAVLinkMessage(
            "GLOBAL_POSITION_INT",
            msg_id,
            sysid,
            compid,
            time_boot_ms=values[0],
            lat=values[1],
            lon=values[2],
            alt=values[3],
            relative_alt=values[4],
            vx=values[5],
            vy=values[6],
            vz=values[7],
            hdg=values[8],
        )

    if msg_id == 74:
        airspeed, groundspeed, alt, climb, heading, throttle = _unpack("<ffffhH", payload)
        return MAVLinkMessage(
            "VFR_HUD",
            msg_id,
            sysid,
            compid,
            airspeed=airspeed,
            groundspeed=groundspeed,
            heading=heading,
            throttle=throttle,
            alt=alt,
            climb=climb,
        )

    if msg_id == 77:
        command, result = _unpack("<HB", payload)
        return MAVLinkMessage("COMMAND_ACK", msg_id, sysid, compid, command=command, result=result)

    return MAVLinkMessage(f"UNKNOWN_{msg_id}", msg_id, sysid, compid, payload=payload)


def _x25_crc(data, crc_extra):
    crc = 0xFFFF
    for byte in data:
        crc = _x25_accumulate(byte, crc)
    return _x25_accumulate(crc_extra, crc)


def _x25_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def _fixed_bytes(value, length):
    if isinstance(value, str):
        value = value.encode("ascii")
    value = bytes(value)
    return value[:length].ljust(length, b"\x00")


def _int8(value):
    return value - 256 if value > 127 else value


def _normalize_type_filter(value):
    if value is None:
        return None
    if isinstance(value, str):
        return {value}
    return set(value)


def _unpack(fmt, payload):
    size = struct.calcsize(fmt)
    padded = payload[:size].ljust(size, b"\x00")
    return struct.unpack(fmt, padded)


_CRC_EXTRA = {
    0: 50,
    1: 124,
    11: 89,
    20: 214,
    22: 220,
    23: 168,
    30: 39,
    32: 185,
    33: 104,
    74: 20,
    76: 152,
    77: 143,
    84: 143,
    86: 5,
    253: 83,
}


def _open_transport(connection_string, baud):
    if connection_string.startswith("udpout:"):
        host, port = _parse_host_port(connection_string, "udpout")
        return _UDPTransport(host, port, bind=False)
    if connection_string.startswith("udpin:"):
        host, port = _parse_host_port(connection_string, "udpin")
        return _UDPTransport(host, port, bind=True)
    if connection_string.startswith("udp:"):
        host, port = _parse_host_port(connection_string, "udp")
        return _UDPTransport(host, port, bind=True)
    if connection_string.startswith("tcpin:"):
        host, port = _parse_host_port(connection_string, "tcpin")
        return _TCPServerTransport(host, port)
    if connection_string.startswith("tcp:") or connection_string.startswith("tcpout:"):
        scheme = "tcpout" if connection_string.startswith("tcpout:") else "tcp"
        host, port = _parse_host_port(connection_string, scheme)
        return _TCPClientTransport(host, port)

    device = connection_string.removeprefix("serial:")
    return _SerialTransport(device, baud)


def _parse_host_port(connection_string, scheme):
    rest = connection_string[len(scheme) + 1 :]
    host, port = rest.rsplit(":", 1)
    return host, int(port)


class _UDPTransport:
    def __init__(self, host, port, bind):
        self.peer = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(None)
        if bind:
            self.socket.bind((host if host not in ("", "0.0.0.0") else "", port))
        else:
            self.socket.connect(self.peer)

    def read(self, size, timeout):
        self.socket.settimeout(timeout)
        try:
            data, addr = self.socket.recvfrom(4096)
        except socket.timeout:
            return b""
        self.peer = addr
        return data

    def write(self, data):
        self.socket.sendto(data, self.peer)

    def close(self):
        self.socket.close()


class _TCPClientTransport:
    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port))

    def read(self, size, timeout):
        self.socket.settimeout(timeout)
        try:
            return self.socket.recv(size)
        except socket.timeout:
            return b""

    def write(self, data):
        self.socket.sendall(data)

    def close(self):
        self.socket.close()


class _TCPServerTransport(_TCPClientTransport):
    def __init__(self, host, port):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        self.socket, _ = server.accept()
        server.close()


class _SerialTransport:
    def __init__(self, device, baud):
        if termios is None or tty is None:
            raise MAVLinkError("Serial connections are supported only on POSIX platforms")

        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        speed = _baud_constant(baud)
        attrs[4] = speed
        attrs[5] = speed
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        tty.setraw(self.fd)

    def read(self, size, timeout):
        readable, _, _ = select.select([self.fd], [], [], timeout)
        if not readable:
            return b""
        return os.read(self.fd, size)

    def write(self, data):
        os.write(self.fd, data)

    def close(self):
        os.close(self.fd)


def _baud_constant(baud):
    name = f"B{int(baud)}"
    if not hasattr(termios, name):
        raise MAVLinkError(f"Unsupported baud rate: {baud}")
    return getattr(termios, name)
