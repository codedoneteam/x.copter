import unittest
from unittest.mock import AsyncMock, Mock, patch
from src.xcopter.modules import Modes
from src.xcopter.modules.errors.error import CopterError


class TestModes(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.modes = Modes()
        self.modes.master = Mock()
        self.modes.master.target_system = 1
        self.modes.master.target_component = 1
        self.modes.master.mav = Mock()

    async def test_guided_true(self):
        self.modes.mode = AsyncMock(return_value="guided")

        with patch.object(self.modes, "info") as mock_info:
            result = await self.modes.guided()

        self.assertTrue(result)
        mock_info.assert_any_call("GUIDED mode is active")

    async def test_guided_false(self):
        self.modes.mode = AsyncMock(return_value="STABILIZE")

        with self.assertRaises(CopterError) as ctx:
            await self.modes.guided()

        self.assertEqual(str(ctx.exception), "GUIDED mode is inactive")

    async def test_guided_none(self):
        self.modes.mode = AsyncMock(return_value=None)

        with self.assertRaises(CopterError) as ctx:
            await self.modes.guided()

        self.assertEqual(str(ctx.exception), "Failed to determine current mode")

    async def test_guide_success(self):
        self.modes.master.mode_mapping.return_value = {"GUIDED": 4}
        ack = Mock()
        ack.command = 176
        ack.result = 0
        self.modes.master.recv_match.return_value = ack

        result = await self.modes.guide()

        self.assertTrue(result)
        self.modes.master.mav.command_long_send.assert_called_once_with(
            self.modes.master.target_system,
            self.modes.master.target_component,
            176,
            0,
            1,
            4,
            0, 0, 0, 0, 0
        )

    async def test_mode_success(self):
        mock_heartbeat = Mock()
        mock_heartbeat.base_mode = 209
        mock_heartbeat.custom_mode = 0
        self.modes.master.recv_match.return_value = mock_heartbeat

        with patch("src.xcopter.modules.modes.mode_string_v10", return_value="STABILIZE"):
            result = await self.modes.mode()

        self.assertEqual(result, "STABILIZE")

    async def test_mode_heartbeat_timeout(self):
        self.modes.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.modes.mode()

        self.assertEqual(str(ctx.exception), "Exception while getting flight mode: Failed to get mode: heartbeat not received")

    async def test_mode_fallback_mode_string(self):
        mock_heartbeat = Mock()
        mock_heartbeat.base_mode = 209
        mock_heartbeat.custom_mode = 2
        self.modes.master.recv_match.return_value = mock_heartbeat

        with patch("src.xcopter.modules.modes.mode_string_v10", side_effect=RuntimeError("fail")):
            result = await self.modes.mode()

        self.assertEqual(result, "base_mode=209, custom_mode=2")

    async def test_guide_no_ack(self):
        self.modes.master.mode_mapping.return_value = {"GUIDED": 4}
        self.modes.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.modes.guide()

        self.assertEqual(str(ctx.exception), "Mode change failed: no response received")
        self.modes.master.mav.command_long_send.assert_called_once_with(
            self.modes.master.target_system,
            self.modes.master.target_component,
            176,
            0,
            1,
            4,
            0, 0, 0, 0, 0
        )


if __name__ == "__main__":
    unittest.main()
