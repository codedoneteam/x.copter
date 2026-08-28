import struct
import unittest

from src.xcopter.mavlink import MAVLinkConnection, mavlink, mode_string_v10


class MemoryTransport:
    def __init__(self):
        self.writes = []
        self.reads = []

    def read(self, size, timeout):
        if not self.reads:
            return b""
        data = self.reads.pop(0)
        return data[:size]

    def write(self, data):
        self.writes.append(data)

    def close(self):
        pass


class TestMAVLink(unittest.TestCase):
    def test_heartbeat_roundtrip(self):
        transport = MemoryTransport()
        sender = MAVLinkConnection(transport, source_system=255, source_component=0)
        sender.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 4, 0)

        receiver = MAVLinkConnection(MemoryTransport())
        receiver._buffer.extend(transport.writes[0])

        msg = receiver.recv_match(type="HEARTBEAT")

        self.assertEqual(msg.get_type(), "HEARTBEAT")
        self.assertEqual(msg._sysid, 255)
        self.assertEqual(msg.custom_mode, 4)
        self.assertEqual(msg.type, mavlink.MAV_TYPE_GCS)
        self.assertEqual(mode_string_v10(msg), "GUIDED")

    def test_command_long_packet_contains_wire_order_payload(self):
        transport = MemoryTransport()
        connection = MAVLinkConnection(transport, source_system=42, source_component=10)

        connection.mav.command_long_send(1, 1, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)

        packet = transport.writes[0]
        self.assertEqual(packet[0], 0xFE)
        self.assertEqual(packet[5], 76)
        payload = packet[6:-2]
        values = struct.unpack("<fffffffHBBB", payload)
        self.assertEqual(values[0], 1.0)
        self.assertEqual(values[1], 21196.0)
        self.assertEqual(values[7], mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(values[8], 1)
        self.assertEqual(values[9], 1)

    def test_command_ack_roundtrip(self):
        transport = MemoryTransport()
        sender = MAVLinkConnection(transport, source_system=1, source_component=1)
        payload = struct.pack("<HB", mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_RESULT_ACCEPTED)
        sender._send_message(77, payload, 143)

        receiver = MAVLinkConnection(MemoryTransport())
        receiver._buffer.extend(transport.writes[0])

        msg = receiver.recv_match(type="COMMAND_ACK")

        self.assertEqual(msg.command, mavlink.MAV_CMD_DO_SET_MODE)
        self.assertEqual(msg.result, mavlink.MAV_RESULT_ACCEPTED)


if __name__ == "__main__":
    unittest.main()
