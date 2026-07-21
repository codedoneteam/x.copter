from pymavlink import mavutil
from .log import Log
from .errors.error import CopterError

class Buzzer(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def beep(self, tone=2000, duration=1, count=1):
        try:
            self.info(f"Sending buzzer signal: tone={tone}Hz, duration={duration}s, count={count}")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_BUZZER,
                0,          # confirmation
                1,          # Param1: enable
                tone,       # Param2: frequency (Hz)
                duration,   # Param3: duration
                count,      # Param4: repeat count
                0, 0, 0     # Param5-7: unused
            )
            return True
        except Exception as e:
            self.error(f"Exception while sending buzzer command: {e}")
            raise CopterError(f"Exception while sending buzzer command: {e}") from e
