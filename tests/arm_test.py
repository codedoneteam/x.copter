import unittest
from unittest.mock import Mock, patch
from pymavlink import mavutil
from src.xcopter.modules import Arming
from src.xcopter.modules.errors.error import CopterError


class TestArming(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.arming = Arming()
        self.arming.master = Mock()

    async def test_armed_success(self):
        heartbeat = Mock()
        heartbeat.base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        self.arming.master.recv_match.return_value = heartbeat

        result = await self.arming.armed()

        self.assertTrue(result)

    async def test_is_disarmed_success(self):
        heartbeat = Mock()
        heartbeat.base_mode = 0
        self.arming.master.recv_match.return_value = heartbeat

        result = await self.arming.armed()

        self.assertFalse(result)

    async def test_armed_heartbeat_timeout(self):
        self.arming.master.recv_match.return_value = None

        with self.assertRaises(CopterError) as ctx:
            await self.arming.armed()

        self.assertEqual(str(ctx.exception), "Exception while getting arming status: Failed to get status: heartbeat not received")

    async def test_arm_success(self):
        heartbeat = Mock()
        heartbeat.base_mode = 0
        ack = Mock()
        ack.result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        self.arming.master.recv_match.side_effect = [heartbeat, ack]
        self.arming.master.target_system = 1
        self.arming.master.target_component = 1

        with patch("src.xcopter.modules.arm.asyncio.sleep", return_value=None):
            result = await self.arming.arm()

        self.assertTrue(result)
        self.arming.master.mav.command_long_send.assert_called_once()

    async def test_arm_failed(self):
        heartbeat = Mock()
        heartbeat.base_mode = 0
        ack = Mock()
        ack.result = mavutil.mavlink.MAV_RESULT_FAILED
        self.arming.master.recv_match.side_effect = [heartbeat, ack]
        self.arming.master.target_system = 1
        self.arming.master.target_component = 1

        with patch("src.xcopter.modules.arm.asyncio.sleep", return_value=None):
            with self.assertRaises(CopterError) as ctx:
                await self.arming.arm()

        self.assertEqual(str(ctx.exception), f"Arming failed, code: {ack.result}")

    async def test_disarm_success(self):
        ack = Mock()
        ack.result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        self.arming.master.recv_match.return_value = ack
        self.arming.master.target_system = 1
        self.arming.master.target_component = 1

        result = await self.arming.disarm()

        self.assertTrue(result)
        self.arming.master.mav.command_long_send.assert_called_once_with(
            self.arming.master.target_system,
            self.arming.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            Arming.FORCE_ARM_DISARM_CODE,
            0,
            0,
            0,
            0,
            0,
        )

    async def test_regular_disarm_success(self):
        heartbeat = Mock()
        heartbeat.base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ack = Mock()
        ack.result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        self.arming.master.recv_match.side_effect = [heartbeat, ack]
        self.arming.master.target_system = 1
        self.arming.master.target_component = 1

        result = await self.arming.disarm(force=False)

        self.assertTrue(result)
        self.arming.master.mav.command_long_send.assert_called_once_with(
            self.arming.master.target_system,
            self.arming.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    async def test_force_disarm_success(self):
        ack = Mock()
        ack.result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        self.arming.master.recv_match.return_value = ack
        self.arming.master.target_system = 1
        self.arming.master.target_component = 1

        result = await self.arming.disarm(force=True)

        self.assertTrue(result)
        self.arming.master.mav.command_long_send.assert_called_once_with(
            self.arming.master.target_system,
            self.arming.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            Arming.FORCE_ARM_DISARM_CODE,
            0,
            0,
            0,
            0,
            0,
        )

    async def test_disarm_failed(self):
        ack = Mock()
        ack.result = mavutil.mavlink.MAV_RESULT_FAILED
        self.arming.master.recv_match.return_value = ack
        self.arming.master.target_system = 1
        self.arming.master.target_component = 1

        with self.assertRaises(CopterError) as ctx:
            await self.arming.disarm()

        self.assertEqual(str(ctx.exception), f"Disarming failed, code: {ack.result}")


if __name__ == "__main__":
    unittest.main()
