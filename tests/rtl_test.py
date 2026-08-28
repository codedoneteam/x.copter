import unittest
from unittest.mock import Mock, patch
from src.xcopter.mavlink import mavlink
from src.xcopter.modules import Rtl
from src.xcopter.modules.errors.error import CopterError

class TestRtl(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.rtl = Rtl()
        self.rtl.master = Mock()
        self.rtl.master.target_system = 1
        self.rtl.master.target_component = 1
        self.rtl.master.mav = Mock()

    def test_setup_success(self):
        result = self.rtl.setup(speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2)
        self.assertTrue(result)
        self.assertTrue(self.rtl.master.mav.param_set_send.called)

    def test_setup_exception(self):
        self.rtl.master.mav.param_set_send.side_effect = Exception("fail")
        with self.assertRaises(CopterError) as ctx:
            self.rtl.setup()
        self.assertEqual(str(ctx.exception), "Error configuring SMART_RTL/LAND: fail")

    async def test_rtl_retries_transient_heartbeat_timeout(self):
        armed_heartbeat = Mock()
        armed_heartbeat.base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        disarmed_heartbeat = Mock()
        disarmed_heartbeat.base_mode = 0
        self.rtl.master.recv_match.side_effect = [armed_heartbeat, None, disarmed_heartbeat]

        with patch("src.xcopter.modules.rtl.asyncio.sleep", return_value=None):
            result = await self.rtl.rtl()

        self.assertTrue(result)

    async def test_rtl_fails_after_repeated_heartbeat_timeouts(self):
        self.rtl.master.recv_match.return_value = None

        with patch("src.xcopter.modules.rtl.asyncio.sleep", return_value=None):
            with self.assertRaises(CopterError) as ctx:
                await self.rtl.rtl()

        self.assertEqual(
            str(ctx.exception),
            "RTL status heartbeat missed 5 times while waiting for landing"
        )

if __name__ == "__main__":
    unittest.main()
