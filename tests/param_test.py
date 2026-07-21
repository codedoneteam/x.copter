import unittest
from unittest.mock import ANY, Mock

from pymavlink import mavutil

from src.xcopter.modules import Params
from src.xcopter.modules.errors.error import CopterError


class TestParams(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.params = Params()
        self.params.master = Mock()
        self.params.master.target_system = 1
        self.params.master.target_component = 1
        self.params.master.mav = Mock()

    async def test_read_success(self):
        msg = Mock()
        msg.param_id = "WPNAV_SPEED"
        msg.param_value = 500.0
        self.params.master.recv_match.return_value = msg

        result = await self.params.read("WPNAV_SPEED")

        self.assertEqual(result, 500.0)
        self.params.master.mav.param_request_read_send.assert_called_once_with(
            self.params.master.target_system,
            self.params.master.target_component,
            b"WPNAV_SPEED",
            -1,
        )
        self.params.master.recv_match.assert_called_once_with(
            type='PARAM_VALUE',
            blocking=True,
            timeout=ANY,
        )

    async def test_read_accepts_bytes_param(self):
        msg = Mock()
        msg.param_id = "WPNAV_SPEED"
        msg.param_value = 500.0
        self.params.master.recv_match.return_value = msg

        result = await self.params.read(b"WPNAV_SPEED")

        self.assertEqual(result, 500.0)
        self.params.master.mav.param_request_read_send.assert_called_once_with(
            self.params.master.target_system,
            self.params.master.target_component,
            b"WPNAV_SPEED",
            -1,
        )

    async def test_read_skips_unrelated_parameter(self):
        wrong = Mock()
        wrong.param_id = "LAND_SPEED"
        wrong.param_value = 50.0
        expected = Mock()
        expected.param_id = "WPNAV_SPEED"
        expected.param_value = 500.0
        self.params.master.recv_match.side_effect = [wrong, expected]

        result = await self.params.read("WPNAV_SPEED")

        self.assertEqual(result, 500.0)
        self.assertEqual(self.params.master.recv_match.call_count, 2)

    async def test_read_timeout(self):
        self.params.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.params.read("WPNAV_SPEED")

        self.assertEqual(str(ctx.exception), "Parameter WPNAV_SPEED response not received")

    async def test_write_success(self):
        msg = Mock()
        msg.param_id = b"WPNAV_SPEED\x00\x00\x00\x00\x00"
        msg.param_value = 700.0
        self.params.master.recv_match.return_value = msg

        result = await self.params.write("WPNAV_SPEED", 700.0)

        self.assertEqual(result, 700.0)
        self.params.master.mav.param_set_send.assert_called_once_with(
            self.params.master.target_system,
            self.params.master.target_component,
            b"WPNAV_SPEED",
            700.0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    async def test_write_accepts_param_type(self):
        msg = Mock()
        msg.param_id = "LAND_SPEED"
        msg.param_value = 75
        self.params.master.recv_match.return_value = msg

        result = await self.params.write(
            "LAND_SPEED",
            75,
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )

        self.assertEqual(result, 75)
        self.params.master.mav.param_set_send.assert_called_once_with(
            self.params.master.target_system,
            self.params.master.target_component,
            b"LAND_SPEED",
            75,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )

    async def test_write_timeout(self):
        self.params.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.params.write("WPNAV_SPEED", 700.0)

        self.assertEqual(str(ctx.exception), "Parameter WPNAV_SPEED response not received")
        self.params.master.mav.param_set_send.assert_called_once_with(
            self.params.master.target_system,
            self.params.master.target_component,
            b"WPNAV_SPEED",
            700.0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    async def test_param_name_empty(self):
        with self.assertRaises(CopterError) as ctx:
            await self.params.read("")

        self.assertEqual(str(ctx.exception), "Parameter name is empty")

    async def test_param_name_too_long(self):
        with self.assertRaises(CopterError) as ctx:
            await self.params.read("PARAMETER_NAME_IS_TOO_LONG")

        self.assertEqual(str(ctx.exception), "Parameter name is too long: PARAMETER_NAME_IS_TOO_LONG")


if __name__ == "__main__":
    unittest.main()
