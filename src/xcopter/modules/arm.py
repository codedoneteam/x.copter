from pymavlink import mavutil
import asyncio
from .log import Log
from .errors.error import CopterError

class Arming(Log):
    FORCE_ARM_DISARM_CODE = 21196

    def __init__(self):
        super().__init__()
        self.master = None

    async def armed(self, timeout=5):
        try:
            heartbeat = await asyncio.to_thread(self.master.recv_match, type='HEARTBEAT', blocking=True, timeout=timeout)
            if heartbeat is None:
                raise CopterError("Failed to get status: heartbeat not received")

            armed = (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            return armed  
        except Exception as e:
            self.error(f"Exception while getting arming status: {e}")
            raise CopterError(f"Exception while getting arming status: {e}") from e

    async def arm(self, timeout=5):
        if await self.armed(timeout=timeout):
            return True
        else:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,  # arm
                    0, 0, 0, 0, 0, 0
                )
                self.info("ARM command sent")
                await asyncio.sleep(1)

                ack = await asyncio.to_thread(self.master.recv_match, type='COMMAND_ACK', blocking=True, timeout=timeout)
                if ack is None:
                    raise CopterError("ACK not received")

                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.info("Copter armed successfully (ARM)")
                    return True
                else:
                    raise CopterError(f"Arming failed, code: {ack.result}")
            except CopterError as e:
                self.error(str(e))
                raise
            except Exception as e:
                self.error(f"Exception during ARM: {e}")
                raise CopterError(f"Exception during ARM: {e}") from e

    async def disarm(self, timeout=5, force=True):
        if not force and not await self.armed(timeout=timeout):
            return True
        else:
            try:
                force_code = self.FORCE_ARM_DISARM_CODE if force else 0
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,  # 1 = arm, 0 = disarm
                    force_code,
                    0, 0, 0, 0, 0
                )

                self.info("FORCE DISARM command sent" if force else "DISARM command sent")

                ack = await asyncio.to_thread(self.master.recv_match, type='COMMAND_ACK', blocking=True, timeout=timeout)
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.info("Copter disarmed successfully (DISARM)")
                    return True
                else:
                    raise CopterError(f"Disarming failed, code: {ack.result}")
            except CopterError as e:
                self.error(str(e))
                raise
            except Exception as e:
                self.error(f"Exception during DISARM: {e}")
                raise CopterError(f"Exception during DISARM: {e}") from e
