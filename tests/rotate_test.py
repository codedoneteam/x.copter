import math
import unittest
from unittest.mock import Mock, patch
from src.xcopter.modules.rotate import Rotate
from src.xcopter.modules.errors.error import CopterError


class RotateTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.rot = Rotate()
        self.rot.master = Mock()
        self.rot.master.target_system = 1
        self.rot.master.target_component = 1
        self.rot.master.mav = Mock()

    async def test_rotate_success(self):
        yaw_sequence = [math.radians(10), math.radians(100)]
        attitude_msg = Mock()

        def recv_match(type, blocking=True, timeout=None):
            if type == "ATTITUDE":
                val = yaw_sequence.pop(0) if yaw_sequence else math.radians(100)
                attitude_msg.yaw = val
                return attitude_msg
            return None

        self.rot.master.recv_match.side_effect = recv_match

        result = await self.rot.rotate(90, 30, tolerance=5, rorate_timeout=1)

        self.assertTrue(result)
        self.rot.master.mav.command_long_send.assert_called_once()

    async def test_rotate_timeout(self):
        attitude_msg = Mock()
        attitude_msg.yaw = math.radians(10)
        self.rot.master.recv_match.return_value = attitude_msg

        with patch("src.xcopter.modules.rotate.time.time", side_effect=[0, 0.02]), \
             patch.object(self.rot, "error"):
            with self.assertRaises(CopterError) as ctx:
                await self.rot.rotate(90, 30, tolerance=1, rorate_timeout=0.01)

        self.assertEqual(str(ctx.exception), "Rotation timed out")

    async def test_rotate_exception(self):
        self.rot.master.recv_match.side_effect = Exception("fail yaw")

        with self.assertRaises(CopterError) as ctx:
            await self.rot.rotate(90, 30)

        self.assertEqual(str(ctx.exception), "Error during rotation: fail yaw")


if __name__ == "__main__":
    unittest.main()
