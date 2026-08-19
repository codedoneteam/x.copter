import asyncio

import time
from ..mavlink import mavlink
from .log import Log
from .errors.error import CopterError

class Takeoff(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def takeoff(self, target_height, epsilon=0.3, speed = 3, hold_time=3, vert_accel=2, takeoff_timeout=300, timeout=1):

        async def get_height():
            msg = await asyncio.to_thread(self.master.recv_match, type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
            if msg is None:
                raise CopterError("Failed to get altitude: GLOBAL_POSITION_INT not received")
            if getattr(msg, "relative_alt", None) is None:
                raise CopterError("Failed to get altitude: relative_alt is missing")
            return msg.relative_alt / 1000.0


        def set_takeoff_speed():
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_SPEED',
                int(speed * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )

            self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    b'WPNAV_SPEED_UP',
                    int(speed * 100),
                    mavlink.MAV_PARAM_TYPE_REAL32
            )

            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_ACCEL_Z',
                int(vert_accel * 100),
                mavlink.MAV_PARAM_TYPE_REAL32
            )           
            return True

        try:
            set_takeoff_speed()

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0,
                0, 0,
                target_height
            )

            self.info(f"TAKEOFF command sent to {target_height} m")

            reached_time = None
            start_time = time.time()

            while time.time() - start_time <= takeoff_timeout:
                current_height = await get_height()

                # Target altitude reached
                if abs(current_height - target_height) <= epsilon:
                    if reached_time is None:
                        reached_time = time.time()
                        self.info("Target altitude reached, checking hold...")

                    # Check altitude hold
                    elif time.time() - reached_time >= hold_time:
                        self.info("TAKEOFF completed, altitude is being held")
                        return True

                else:
                    reached_time = None

            raise CopterError("TAKEOFF timed out: altitude not reached")
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"Exception during TAKEOFF: {e}")
            raise CopterError(f"Exception during TAKEOFF: {e}") from e
