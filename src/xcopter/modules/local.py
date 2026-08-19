import asyncio
import time
import math
from ..mavlink import mavlink
from .log import Log
from .errors.error import CopterError

class Local(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def height(self, timeout=1):
        try:
            msg = await asyncio.to_thread(self.master.recv_match, type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
            if msg:
                return msg.relative_alt / 1000.0  # Convert from millimeters to meters
            raise CopterError("Failed to get relative altitude: GLOBAL_POSITION_INT not received")
        except Exception as e:
            self.error(f"Exception while getting relative altitude: {e}")
            raise CopterError(f"Exception while getting relative altitude: {e}") from e
        
    async def location(self, timeout=1):
        try:
            msg = await asyncio.to_thread(self.master.recv_match, type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
            if msg:
                return {
                    "x": msg.x,
                    "y": msg.y
                }
            raise CopterError("Failed to get local position: LOCAL_POSITION_NED not received")
        except Exception as e:
            self.error(f"Exception while getting local position: {e}")
            raise CopterError(f"Exception while getting local position: {e}") from e
            
    async def shift(self, forward=0.0, right=0.0, up=0.0, speed = 0.1, epsilon=0.05, shift_timeout=300, timeout=5):

        def set_speed(speed):
            speed_cm = int(speed * 100)

            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_SPEED',
                float(speed_cm),
                mavlink.MAV_PARAM_TYPE_REAL32
            )
            self.info(f"Speed set: {speed} m/s")


        try:
            self.info(f"Shift copter: {forward}, {right}, {up}, {epsilon}, {shift_timeout}, {timeout}")

            set_speed(speed)

            # Starting position
            start = await asyncio.to_thread(self.master.recv_match, type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if start is None:
                raise CopterError("No LOCAL_POSITION_NED data")

            start_x, start_y, start_z = start.x, start.y, start.z

            # Offset in BODY frame
            x = forward
            y = right
            z = -up  # NED

            target_distance = math.sqrt(forward**2 + right**2 + up**2)

            # Send offset
            self.master.mav.set_position_target_local_ned_send(
                    0,
                    self.master.target_system,
                    self.master.target_component,
                    mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111111000,  # position only
                    x, y, z,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0
        )

            start_time = time.time()

            while time.time() - start_time <= shift_timeout:
                # Read current position
                pos = await asyncio.to_thread(self.master.recv_match, type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
                if pos is None:
                    continue

                dx = pos.x - start_x
                dy = pos.y - start_y
                dz = pos.z - start_z

                current_distance = math.sqrt(dx**2 + dy**2 + dz**2)

                self.debug(f"Current distance: {current_distance} Target distance: {target_distance}")

                if current_distance >= (target_distance - epsilon):
                    return True

                await asyncio.sleep(0.1)

            raise CopterError("Shift timed out: target not reached")
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"Error during shift: {e}")
            raise CopterError(f"Error during shift: {e}") from e
                
    async def stop(self):
        try:
            self.master.mav.set_position_target_local_ned_send(
                    0,
                    self.master.target_system,
                    self.master.target_component,
                    mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000111,  # use velocity only
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0
                )
            self.info("Copter stopped")
            return True
        except Exception as e:
            self.error(f"Error while stopping: {e}")
            raise CopterError(f"Error while stopping: {e}") from e
