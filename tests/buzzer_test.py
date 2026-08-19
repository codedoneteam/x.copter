import unittest
from unittest.mock import Mock
from src.xcopter.mavlink import mavlink
from src.xcopter.modules import Buzzer
from src.xcopter.modules.errors.error import CopterError


class TestBuzzer(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.buzzer = Buzzer()
        self.buzzer.master = Mock()
        self.buzzer.master.target_system = 1
        self.buzzer.master.target_component = 1
        self.buzzer.master.mav = Mock()

    async def test_beep_success(self):
        result = await self.buzzer.beep(tone=1500, duration=2, count=3)

        self.assertTrue(result)
        self.buzzer.master.mav.command_long_send.assert_called_once_with(
            self.buzzer.master.target_system,
            self.buzzer.master.target_component,
            mavlink.MAV_CMD_DO_BUZZER,
            0,
            1,
            1500,
            2,
            3,
            0,
            0,
            0,
        )

    async def test_beep_exception(self):
        self.buzzer.master.mav.command_long_send.side_effect = Exception("fail")

        with self.assertRaises(CopterError) as ctx:
            await self.buzzer.beep()

        self.assertEqual(str(ctx.exception), "Exception while sending buzzer command: fail")


if __name__ == "__main__":
    unittest.main()
