import time
import asyncio
from ..mavlink import mavlink
from .log import Log
from .errors.error import CopterError

class Move(Log):
    def __init__(self, master):
        super().__init__()
        self.master = master

        # Default PID coefficients for X, Y, Z, yaw
        self.P = (0.8, 0.8, 0.8, 0.5)
        self.I = (0.0, 0.0, 0.0, 0.0)
        self.D = (0.2, 0.2, 0.2, 0.1)

        # PID loop frequency (Hz)
        self.pid_loop = 10

        # Minimum and maximum speeds (for XYZ) and yaw_rate
        self.min_speed = 0.1
        self.max_speed = 10.0
        self.min_rate = 5.0
        self.max_rate = 100.0

        # Target reach tolerance
        self.epsilon = 0.05
        self._target_reached_since = None

    def pid(self, P=(0.8, 0.8, 0.8, 0.5), I=(0.0, 0.0, 0.0, 0.0), D=(0.2, 0.2, 0.2, 0.1),
         pid_loop=10, min_speed=0.1, max_speed=10.0, min_rate=5.0, max_rate=100.0, epsilon=0.05):
        self.P = P
        self.I = I
        self.D = D
        self.pid_loop = pid_loop
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.min_rate = min_rate
        self.max_rate = max_rate  
        self.epsilon = epsilon

    def target_reached(self, error, target_hold_time=1.0, now=None):
        now = time.time() if now is None else now

        if not all(abs(e) < self.epsilon for e in error):
            self._target_reached_since = None
            return False

        if self._target_reached_since is None:
            self._target_reached_since = now

        return now - self._target_reached_since >= target_hold_time

    async def move(self, move_timeout=300, target_hold_time=1.0, error_fn=None):
        
        def minimum(v, min_value):
            if abs(v) < min_value:
                return min_value if v >= 0 else -min_value
            return v

        def send_velocity(vx, vy, vz, yaw_rate):
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavlink.MAV_FRAME_BODY_NED,  # control in the drone coordinate frame
                0b0000111111000111,  # ignore position, send velocity and yaw_rate
                0, 0, 0,             # x, y, z positions
                vx, vy, vz,          # velocities
                0, 0, 0,             # accelerations
                0, 0, yaw_rate       # yaw and yaw_rate
            )

        try:
            integral = [0.0, 0.0, 0.0, 0.0]  # for X, Y, Z, and yaw
            prev_error = [0.0, 0.0, 0.0, 0.0]
            self._target_reached_since = None

            dt = 1.0 / self.pid_loop
            start_time = time.time()

            while time.time() - start_time <= move_timeout:
                error = error_fn()

                if error is None:
                    send_velocity(0, 0, 0, 0)
                    raise CopterError("error_fn failed: no data")

                # PID for XYZ + yaw
                integral = [integral[i] + error[i]*dt for i in range(4)]
                derivative = [(error[i] - prev_error[i])/dt for i in range(4)]

                # Compute velocity commands
                vx = self.P[0]*error[0] + self.I[0]*integral[0] + self.D[0]*derivative[0]
                vy = self.P[1]*error[1] + self.I[1]*integral[1] + self.D[1]*derivative[1]
                vz = self.P[2]*error[2] + self.I[2]*integral[2] + self.D[2]*derivative[2]
                yaw_rate = self.P[3]*error[3] + self.I[3]*integral[3] + self.D[3]*derivative[3]

                # Clamp velocity
                vx = max(min(vx, self.max_speed), -self.max_speed)
                vy = max(min(vy, self.max_speed), -self.max_speed)
                vz = max(min(vz, self.max_speed), -self.max_speed)
                yaw_rate = max(min(yaw_rate, self.max_rate), -self.max_rate)

                # Minimum velocity
                vx = minimum(vx, self.min_speed) if abs(error[0]) > self.epsilon else 0
                vy = minimum(vy, self.min_speed) if abs(error[1]) > self.epsilon else 0
                vz = minimum(vz, self.min_speed) if abs(error[2]) > self.epsilon else 0
                yaw_rate = minimum(yaw_rate, self.min_rate) if abs(error[3]) > self.epsilon else 0

                self.info(f"vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")

                send_velocity(vx, vy, vz, yaw_rate)

                prev_error = error

                if self.target_reached(error, target_hold_time):
                    self.info("Target reached")
                    send_velocity(0, 0, 0, 0)
                    return True

                await asyncio.sleep(dt)

            self.info("Timeout: target not reached")
            send_velocity(0, 0, 0, 0)
            raise CopterError("Timeout: target not reached")
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"Error during movement: {e}")
            raise CopterError(f"Error during movement: {e}") from e
