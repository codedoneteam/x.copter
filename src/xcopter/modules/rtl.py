import asyncio
from ..mavlink import mavlink
from .log import Log
from .errors.error import CopterError

class Rtl(Log):
    HEARTBEAT_MISS_LIMIT = 5

    def __init__(self):
        super().__init__()
        self.master = None

    def setup(self, speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2):
        try:
            # Horizontal speed
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_SPEED',
                int(speed * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Climb speed
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_SPEED_UP',
                int(climb * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Descent speed
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_SPEED_DN',
                int(descend * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Final landing speed
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'LAND_SPEED',
                int(land * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Vertical acceleration (speed up / slow down on Z)
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_ACCEL_Z',
                int(vert_accel * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Return altitude
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'RTL_ALT',
                int(rtl_alt * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )

            return True
        except CopterError:
            raise
        except Exception as e:
                self.error(f"Error configuring SMART_RTL/LAND: {e}")
                raise CopterError(f"Error configuring SMART_RTL/LAND: {e}") from e
        
    async def rtl(self, speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2, timeout=5):
        try:
            self.setup(speed=speed, climb=climb, descend=descend, land=land, rtl_alt=rtl_alt, vert_accel=vert_accel)

            self.master.mav.set_mode_send(
                self.master.target_system,
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6  # RTL
            )

            self.info("RTL activated, waiting for landing...")

            await self._wait_until_disarmed("RTL", timeout)

            return True
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"RTL error: {e}")
            raise CopterError(f"RTL error: {e}") from e
        
    async def smart_rtl(self, speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2, timeout=5):
        try:
            self.setup(speed=speed, climb=climb, descend=descend, land=land, rtl_alt=rtl_alt, vert_accel=vert_accel)

            self.master.mav.set_mode_send(
                self.master.target_system,
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                21  # SMART_RTL
            )

            await self._wait_until_disarmed("SMART_RTL", timeout)

            return True
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"SMART_RTL error: {e}")
            raise CopterError(f"SMART_RTL error: {e}") from e

    async def _wait_until_disarmed(self, mode, timeout):
        missed_heartbeats = 0

        while True:
            try:
                if not await self._armed_from_heartbeat(timeout):
                    return True
                missed_heartbeats = 0
            except CopterError as e:
                if "heartbeat not received" not in str(e):
                    raise

                missed_heartbeats += 1
                if missed_heartbeats >= self.HEARTBEAT_MISS_LIMIT:
                    raise CopterError(
                        f"{mode} status heartbeat missed {missed_heartbeats} times while waiting for landing"
                    ) from e

                self.warn(
                    f"Heartbeat not received while waiting for {mode} landing "
                    f"({missed_heartbeats}/{self.HEARTBEAT_MISS_LIMIT})"
                )

            await asyncio.sleep(1)

    async def armed(self, timeout):
        return await self._armed_from_heartbeat(timeout)

    async def _armed_from_heartbeat(self, timeout):
        heartbeat = await asyncio.to_thread(self.master.recv_match, type='HEARTBEAT', blocking=True, timeout=timeout)
        if heartbeat is None:
            raise CopterError("Failed to get status: heartbeat not received")

        armed = (heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        return armed
