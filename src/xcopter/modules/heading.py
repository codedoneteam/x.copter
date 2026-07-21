import asyncio
import math
from .log import Log
from .errors.error import CopterError

class Heading(Log):        
    def __init__(self):
        super().__init__()
        self.master = None

    async def heading(self, timeout=5):
        try:
            msg = await asyncio.to_thread(self.master.recv_match, type='ATTITUDE', blocking=True, timeout=timeout)

            if not msg:
                raise CopterError("Failed to get heading: ATTITUDE not received")

            yaw = getattr(msg, "yaw", None)

            if yaw is None:
                raise CopterError("Failed to get heading: yaw is missing")

            # yaw comes in radians -> convert to degrees
            heading = math.degrees(yaw)

            # normalize to the 0-360 range
            heading = heading % 360

            return heading
        except Exception as e:
            self.error(f"Exception while getting heading: {e}")
            raise CopterError(f"Exception while getting heading: {e}") from e
