import math
import unittest
from unittest.mock import Mock
from src.xcopter.modules.heading import Heading
from src.xcopter.modules.errors.error import CopterError


class TestHeading(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.heading = Heading()
        self.heading.master = Mock()

    async def test_heading_success(self):
        mock_msg = Mock()
        mock_msg.yaw = math.radians(90)
        self.heading.master.recv_match.return_value = mock_msg

        result = await self.heading.heading()

        self.assertAlmostEqual(result, 90, places=2)

    async def test_heading_none_msg(self):
        self.heading.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.heading.heading()

        self.assertEqual(str(ctx.exception), "Exception while getting heading: Failed to get heading: ATTITUDE not received")

    async def test_heading_no_yaw(self):
        mock_msg = Mock(spec=[])
        self.heading.master.recv_match.return_value = mock_msg

        with self.assertRaises(CopterError) as ctx:
            await self.heading.heading()

        self.assertEqual(str(ctx.exception), "Exception while getting heading: Failed to get heading: yaw is missing")

    async def test_heading_exception(self):
        self.heading.master.recv_match.side_effect = Exception("fail")

        with self.assertRaises(CopterError) as ctx:
            await self.heading.heading()

        self.assertEqual(str(ctx.exception), "Exception while getting heading: fail")


if __name__ == "__main__":
    unittest.main()
