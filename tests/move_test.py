import unittest
from unittest.mock import Mock, patch
from src.xcopter.modules.move import Move


class TestMove(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        master = Mock()
        master.target_system = 1
        master.target_component = 1
        master.mav = Mock()
        self.move = Move(master)

    async def test_move_minimal(self):
        with patch("src.xcopter.modules.move.asyncio.sleep", return_value=None):
            result = await self.move.move(target_hold_time=0.0, error_fn=lambda: [0, 0, 0, 0])

        self.assertTrue(result)
        self.assertGreaterEqual(
            self.move.master.mav.set_position_target_local_ned_send.call_count, 2
        )

    def test_target_reached_requires_errors_within_epsilon_for_hold_time(self):
        error = [0.01, -0.02, 0.03, -0.04]

        self.assertFalse(self.move.target_reached(error, target_hold_time=1.0, now=10.0))
        self.assertFalse(self.move.target_reached(error, target_hold_time=1.0, now=10.5))
        self.assertTrue(self.move.target_reached(error, target_hold_time=1.0, now=11.0))

    def test_target_reached_resets_hold_time_when_error_exceeds_epsilon(self):
        inside_error = [0.01, -0.02, 0.03, -0.04]
        outside_error = [0.01, -0.02, 0.05, -0.04]

        self.assertFalse(self.move.target_reached(inside_error, target_hold_time=1.0, now=10.0))
        self.assertFalse(self.move.target_reached(outside_error, target_hold_time=1.0, now=10.5))
        self.assertFalse(self.move.target_reached(inside_error, target_hold_time=1.0, now=10.6))
        self.assertTrue(self.move.target_reached(inside_error, target_hold_time=1.0, now=11.6))


if __name__ == "__main__":
    unittest.main()
