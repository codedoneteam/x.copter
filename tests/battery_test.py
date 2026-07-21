import unittest
from unittest.mock import Mock
from src.xcopter.modules.battery import Battery
from src.xcopter.modules.errors.error import CopterError


class BatteryTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.battery = Battery()
        self.battery.master = Mock()

    async def test_voltage_success(self):
        msg = Mock()
        msg.voltage_battery = 12345
        self.battery.master.recv_match.return_value = msg

        result = await self.battery.voltage()

        self.assertEqual(result, 12.345)

    async def test_voltage_timeout(self):
        self.battery.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.battery.voltage(timeout=0.1)

        self.assertEqual(str(ctx.exception), "Exception while getting voltage: Failed to get voltage: SYS_STATUS not received")

    async def test_voltage_not_provided(self):
        msg = Mock()
        msg.voltage_battery = 65535
        self.battery.master.recv_match.return_value = msg

        with self.assertRaises(CopterError) as ctx:
            await self.battery.voltage()

        self.assertEqual(str(ctx.exception), "Exception while getting voltage: Failed to get voltage: voltage_battery not provided")

    async def test_battery_success(self):
        msg = Mock()
        msg.battery_remaining = 87
        self.battery.master.recv_match.return_value = msg

        result = await self.battery.battery()

        self.assertEqual(result, 13)

    async def test_battery_timeout(self):
        self.battery.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.battery.battery(timeout=0.1)

        self.assertEqual(str(ctx.exception), "Exception while getting battery: Failed to get battery: SYS_STATUS not received")

    async def test_battery_not_provided(self):
        msg = Mock()
        msg.battery_remaining = -1
        self.battery.master.recv_match.return_value = msg

        with self.assertRaises(CopterError) as ctx:
            await self.battery.battery()

        self.assertEqual(str(ctx.exception), "Exception while getting battery: Failed to get battery: battery_remaining not provided")

    async def test_current_success(self):
        msg = Mock()
        msg.current_battery = 1234
        self.battery.master.recv_match.return_value = msg

        result = await self.battery.current()

        self.assertEqual(result, 12.34)

    async def test_current_timeout(self):
        self.battery.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.battery.current(timeout=0.1)

        self.assertEqual(str(ctx.exception), "Exception while getting current: Failed to get current: SYS_STATUS not received")

    async def test_current_not_provided(self):
        msg = Mock()
        msg.current_battery = -1
        self.battery.master.recv_match.return_value = msg

        with self.assertRaises(CopterError) as ctx:
            await self.battery.current()

        self.assertEqual(str(ctx.exception), "Exception while getting current: Failed to get current: current_battery not provided")


if __name__ == "__main__":
    unittest.main()
