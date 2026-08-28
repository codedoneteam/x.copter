import unittest
import asyncio
from unittest.mock import Mock, patch

from src.xcopter.xarducopter import XArduCopter


class TestXArduCopter(unittest.IsolatedAsyncioTestCase):
    def test_init_initializes_pid_defaults(self):
        copter = XArduCopter()

        self.assertEqual(copter.P, (0.8, 0.8, 0.8, 0.5))
        self.assertEqual(copter.I, (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(copter.D, (0.2, 0.2, 0.2, 0.1))
        self.assertEqual(copter.pid_loop, 10)
        self.assertEqual(copter.epsilon, 0.05)
        self.assertIsNone(copter._heartbeat_task)

    @patch("src.xcopter.xarducopter.mavlink_connection")
    async def test_connect(self, mock_mavlink_connection):
        mock_master = Mock()
        mock_master.wait_heartbeat = Mock()
        mock_mavlink_connection.return_value = mock_master

        copter = XArduCopter()

        with patch("src.xcopter.xarducopter.time.time", return_value=123.0):
            await copter.connect("udp:127.0.0.1:14550")

        mock_mavlink_connection.assert_called_once_with(
            "udp:127.0.0.1:14550",
            baud=115200,
            source_system=255,
            source_component=0,
        )
        mock_master.wait_heartbeat.assert_called_once_with()
        self.assertEqual(copter.master, mock_master)
        self.assertEqual(copter.start_time, 123.0)

    @patch("src.xcopter.xarducopter.mavlink_connection")
    async def test_connect_accepts_positional_baud(self, mock_mavlink_connection):
        mock_master = Mock()
        mock_master.wait_heartbeat = Mock()
        mock_mavlink_connection.return_value = mock_master

        copter = XArduCopter()

        await copter.connect("tcp:127.0.0.1:5760", 57600)

        mock_mavlink_connection.assert_called_once_with(
            "tcp:127.0.0.1:5760",
            baud=57600,
            source_system=255,
            source_component=0,
        )

    @patch("src.xcopter.xarducopter.mavlink_connection")
    async def test_connect_accepts_uart_connection(self, mock_mavlink_connection):
        mock_master = Mock()
        mock_master.wait_heartbeat = Mock()
        mock_mavlink_connection.return_value = mock_master

        copter = XArduCopter()

        await copter.connect("uart:/dev/ttyUSB0", 57600)

        mock_mavlink_connection.assert_called_once_with(
            "/dev/ttyUSB0",
            baud=57600,
            source_system=255,
            source_component=0,
        )

    @patch("src.xcopter.xarducopter.mavlink_connection")
    async def test_connect_rejects_unsupported_connection_type(self, mock_mavlink_connection):
        copter = XArduCopter()

        with self.assertRaisesRegex(ValueError, "Supported types: UDP, TCP, UART"):
            await copter.connect("serial:/dev/ttyUSB0")

        mock_mavlink_connection.assert_not_called()

    @patch("src.xcopter.xarducopter.asyncio.create_task")
    @patch("src.xcopter.xarducopter.mavlink_connection")
    async def test_connect_gcs_starts_heartbeat_task(self, mock_mavlink_connection, mock_create_task):
        mock_master = Mock()
        mock_master.wait_heartbeat = Mock()
        mock_mavlink_connection.return_value = mock_master
        mock_task = Mock()
        mock_create_task.return_value = mock_task

        copter = XArduCopter()

        await copter.connect("udpout:127.0.0.1:14550", gcs=True, source_system=42)

        mock_mavlink_connection.assert_called_once_with(
            "udpout:127.0.0.1:14550",
            baud=115200,
            source_system=255,
            source_component=0,
        )
        mock_create_task.assert_called_once()
        self.assertIs(copter._heartbeat_task, mock_task)

        created_coro = mock_create_task.call_args.args[0]
        created_coro.close()

    @patch("src.xcopter.xarducopter.logging.info")
    async def test_close_cancels_heartbeat_and_closes_master(self, mock_logging_info):
        copter = XArduCopter()
        master = Mock()
        copter.master = master
        heartbeat_task = asyncio.create_task(asyncio.sleep(10))
        copter._heartbeat_task = heartbeat_task

        await copter.close()

        master.close.assert_called_once_with()
        mock_logging_info.assert_called_once_with("Connection to the copter closed")
        self.assertTrue(heartbeat_task.cancelled())
        self.assertIsNone(copter.master)
        self.assertIsNone(copter._heartbeat_task)


if __name__ == "__main__":
    unittest.main()
