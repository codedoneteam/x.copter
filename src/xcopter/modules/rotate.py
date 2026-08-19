import math
import time
import asyncio
from ..mavlink import mavlink
from .log import Log
from .errors.error import CopterError

class Rotate(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def rotate(self, angle, rate, tolerance=3, rorate_timeout=100, timeout=5):
        def normalize_angle(angle):
            return (angle + 180) % 360 - 180

        async def get_yaw(master):
            msg = await asyncio.to_thread(master.recv_match, type='ATTITUDE', blocking=True, timeout=timeout)
            if msg is None:
                raise CopterError("Failed to get yaw: ATTITUDE not received")
            if getattr(msg, "yaw", None) is None:
                raise CopterError("Failed to get yaw: yaw is missing")
            return math.degrees(msg.yaw)
    
        try:
            start_yaw = await get_yaw(self.master)
            self.info(f"Initial angle: {start_yaw:.2f} degrees")

            target_yaw = normalize_angle(start_yaw + angle)

            cw = 1 if angle >= 0 else -1

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(angle),
                abs(rate),
                cw,
                1,   # relative rotation
                0, 0, 0
            )

            start_time = time.time()

            while time.time() - start_time <= rorate_timeout:
                current_yaw = await get_yaw(self.master)
                error = normalize_angle(target_yaw - current_yaw)

                if abs(error) <= tolerance:
                    self.info(f"Rotation completed. Current angle: {current_yaw:.2f} degrees, error: {error:.2f} degrees")
                    return True

            raise CopterError("Rotation timed out")
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"Error during rotation: {e}")
            raise CopterError(f"Error during rotation: {e}") from e
