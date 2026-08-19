from ..mavlink import mavlink
from .log import Log
from .errors.error import CopterError

class Land(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def land(self, descend=2, land=0.5):
        try:
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

            self.master.mav.set_mode_send(
                self.master.target_system,
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                9  # LAND
            )

            return True

        except Exception as e:
            self.error(f"Error configuring LAND: {e}")
            raise CopterError(f"Error configuring LAND: {e}") from e
