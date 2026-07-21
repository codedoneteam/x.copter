import unittest
from unittest.mock import Mock
from src.xcopter.modules.local import Local
from src.xcopter.modules.errors.error import CopterError


class TestLocal(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.local = Local()
        self.local.master = Mock()
        self.local.master.target_system = 1
        self.local.master.target_component = 1
        self.local.master.mav = Mock()

    async def test_location_success(self):
        msg = Mock()
        msg.x = 10
        msg.y = 20
        self.local.master.recv_match.return_value = msg

        result = await self.local.location()

        self.assertEqual(result, {"x": 10, "y": 20})

    async def test_location_none(self):
        self.local.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.local.location()

        self.assertEqual(str(ctx.exception), "Exception while getting local position: Failed to get local position: LOCAL_POSITION_NED not received")

    async def test_location_exception(self):
        self.local.master.recv_match.side_effect = Exception("fail")

        with self.assertRaises(CopterError) as ctx:
            await self.local.location()

        self.assertEqual(str(ctx.exception), "Exception while getting local position: fail")


if __name__ == "__main__":
    unittest.main()
