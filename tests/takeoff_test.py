import unittest
from unittest.mock import Mock, patch
from src.xcopter.modules import Takeoff
from src.xcopter.modules.errors.error import CopterError


class TakeoffTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.takeoff = Takeoff()
        self.takeoff.master = Mock()
        self.takeoff.master.target_system = 1
        self.takeoff.master.target_component = 1
        self.takeoff.master.mav = Mock()

    async def test_takeoff_success(self):
        msg = Mock()
        msg.relative_alt = 10000
        self.takeoff.master.recv_match.side_effect = [msg, msg]

        with patch(
            "src.xcopter.modules.takeoff.time.time",
            side_effect=[0.0, 0.1, 0.2, 0.3, 0.4],
        ):
            result = await self.takeoff.takeoff(10, hold_time=0.0)

        self.assertTrue(result)
        self.takeoff.master.mav.command_long_send.assert_called_once()

    async def test_takeoff_timeout(self):
        msg = Mock()
        msg.relative_alt = 0
        self.takeoff.master.recv_match.return_value = msg

        with patch("src.xcopter.modules.takeoff.time.time", side_effect=[0.0, 0.1, 1.0]), \
             patch.object(self.takeoff, "error"):
            with self.assertRaises(CopterError) as ctx:
                await self.takeoff.takeoff(10, takeoff_timeout=0.5)

        self.assertEqual(str(ctx.exception), "TAKEOFF timed out: altitude not reached")


if __name__ == "__main__":
    unittest.main()
