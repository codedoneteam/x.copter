import unittest
from unittest.mock import AsyncMock, Mock, patch
from src.xcopter.modules.gps import Gps
from src.xcopter.modules.errors.error import CopterError


class GpsTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.gps = Gps()
        self.gps.master = Mock()

    async def test_altitude_success(self):
        mock_msg = Mock()
        mock_msg.alt = 123.45
        self.gps.master.recv_match.return_value = mock_msg

        result = await self.gps.altitude()

        self.assertEqual(result, 123.45)

    async def test_altitude_timeout(self):
        self.gps.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.gps.altitude(timeout=0.1)

        self.assertEqual(str(ctx.exception), "Exception while getting altitude: Failed to get altitude: VFR_HUD not received")

    async def test_altitude_exception(self):
        self.gps.master.recv_match.side_effect = Exception("fail")

        with self.assertRaises(CopterError) as ctx:
            await self.gps.altitude()

        self.assertEqual(str(ctx.exception), "Exception while getting altitude: fail")

    async def test_position_success(self):
        mock_msg = Mock()
        mock_msg.lat = int(55.7558 * 1e7)
        mock_msg.lon = int(37.6173 * 1e7)
        mock_msg.alt = int(200 * 1000)
        self.gps.master.recv_match.return_value = mock_msg

        result = await self.gps.position()

        self.assertEqual(result, {"lat": 55.7558, "lon": 37.6173, "alt": 200.0})

    async def test_position_timeout(self):
        self.gps.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.gps.position(timeout=0.1)

        self.assertEqual(str(ctx.exception), "Position error: Failed to get position: GLOBAL_POSITION_INT not received")

    async def test_position_exception(self):
        self.gps.master.recv_match.side_effect = Exception("fail")

        with self.assertRaises(CopterError) as ctx:
            await self.gps.position()

        self.assertEqual(str(ctx.exception), "Position error: fail")

    async def test_navigate_success(self):
        self.gps.master.target_system = 1
        self.gps.master.target_component = 1
        self.gps.master.mav = Mock()
        self.gps.start_time = 0
        self.gps.position = AsyncMock(
            side_effect=[
                {"lat": 54.0, "lon": 36.0, "alt": 90.0},
                {"lat": 55.0, "lon": 37.0, "alt": 100.0},
            ]
        )

        with patch("src.xcopter.modules.gps.asyncio.sleep", return_value=None):
            result = await self.gps.navigate(55.0, 37.0, 100.0, speed=5, timeout=1)

        self.assertTrue(result)
        self.gps.master.mav.param_set_send.assert_called_once()
        self.gps.master.mav.set_position_target_global_int_send.assert_called_once()

    async def test_navigate_timeout(self):
        self.gps.master.target_system = 1
        self.gps.master.target_component = 1
        self.gps.master.mav = Mock()
        self.gps.start_time = 0
        self.gps.position = AsyncMock(side_effect=CopterError("no position"))

        with patch("src.xcopter.modules.gps.time.time", side_effect=[0, 1, 2]):
            with self.assertRaises(CopterError) as ctx:
                await self.gps.navigate(55.0, 37.0, 100.0, speed=5, timeout=0.1, navigation_timeout=0.5)

        self.assertEqual(str(ctx.exception), "Timed out while reaching target")


if __name__ == "__main__":
    unittest.main()
