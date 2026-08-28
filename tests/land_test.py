import unittest
from unittest.mock import Mock
from src.xcopter.modules import Land
from src.xcopter.modules.errors.error import CopterError


class TestLand(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.land = Land()
        self.land.master = Mock()
        self.land.master.target_system = 1
        self.land.master.target_component = 1
        self.land.master.mav = Mock()

    async def test_land_success(self):
        from src.xcopter.mavlink import mavlink

        result = await self.land.land(descend=2, land=0.5)

        self.assertTrue(result)
        self.land.master.mav.param_set_send.assert_any_call(
            self.land.master.target_system,
            self.land.master.target_component,
            b"WPNAV_SPEED_DN",
            200,
            mavlink.MAV_PARAM_TYPE_REAL32,
        )
        self.land.master.mav.param_set_send.assert_any_call(
            self.land.master.target_system,
            self.land.master.target_component,
            b"LAND_SPEED",
            50,
            mavlink.MAV_PARAM_TYPE_REAL32,
        )
        self.land.master.mav.set_mode_send.assert_called_once()

    async def test_land_exception(self):
        self.land.master.mav.param_set_send.side_effect = Exception("fail")

        with self.assertRaises(CopterError) as ctx:
            await self.land.land()

        self.assertEqual(str(ctx.exception), "Error configuring LAND: fail")


if __name__ == "__main__":
    unittest.main()
