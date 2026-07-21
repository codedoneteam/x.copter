import math
import time
from pymavlink import mavutil
import asyncio
from .log import Log
from .errors.error import CopterError

class Gps(Log):
    def __init__(self):
        super().__init__()
        self.start_time = None
        self.master = None

    async def altitude(self, timeout=5):
        try:
            msg = await asyncio.to_thread(self.master.recv_match, type='VFR_HUD', blocking=True, timeout=timeout)
            if msg:
                return msg.alt
            raise CopterError("Failed to get altitude: VFR_HUD not received")
        except Exception as e:
            self.error(f"Exception while getting altitude: {e}")
            raise CopterError(f"Exception while getting altitude: {e}") from e
        
    async def position(self, timeout=5):
        try:
            msg = await asyncio.to_thread(self.master.recv_match,
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=timeout
            )

            if not msg:
                raise CopterError("Failed to get position: GLOBAL_POSITION_INT not received")

            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.alt / 1000
            }
        except Exception as e:
            self.error(f"Position error: {e}")
            raise CopterError(f"Position error: {e}") from e

    async def navigate(self, lat, lon, alt, speed=5, epsilon=0.5, hz=10, navigation_timeout=600, timeout=60):

        def set_speed(speed):
            speed_cm = int(speed * 100)

            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_SPEED',
                float(speed_cm),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            self.info(f"Speed set: {speed} m/s")

        def goto(lat, lon, alt):
            try:
                lat_int = int(lat * 1e7)
                lon_int = int(lon * 1e7)

                type_mask = (
                    (1 << 3) | (1 << 4) | (1 << 5) |   # ignore velocity
                    (1 << 6) | (1 << 7) | (1 << 8) |   # ignore acceleration
                    (1 << 10) | (1 << 11)              # ignore yaw + yaw_rate
                )

                self.master.mav.set_position_target_global_int_send(
                    int((time.time() - self.start_time) * 1000),
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    type_mask,
                    lat_int,
                    lon_int,
                    alt,   # AMSL altitude
                    0, 0, 0,
                    0, 0, 0,
                    0,
                    0
                )
                return True

            except Exception as e:
                self.error(f"Goto error: {e}")
                raise CopterError(f"Goto error: {e}") from e

        self.info(f"Starting navigation to {lat}, {lon}, {alt}")

        start_time = time.time()
        set_speed(speed)
        dt = 1.0 / hz

        try:
            while time.time() - start_time <= navigation_timeout:
                pos = await self.position(timeout=timeout)

                # --- XY distance (meters) ---
                meters_per_deg_lat = 111320
                meters_per_deg_lon = 111320 * math.cos(math.radians(pos["lat"]))

                dx = (lat - pos["lat"]) * meters_per_deg_lat
                dy = (lon - pos["lon"]) * meters_per_deg_lon

                # --- AMSL Z ---
                dz = alt - pos["alt"]

                horizontal = math.sqrt(dx**2 + dy**2)
                vertical = abs(dz)

                # --- reached target ---
                if horizontal <= epsilon and vertical <= epsilon:
                    self.info("Target reached")
                    return True

                goto(lat, lon, alt)

                await asyncio.sleep(dt)

            raise CopterError("Timed out while reaching target")
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"Exception during navigation: {e}")
            raise CopterError(f"Exception during navigation: {e}") from e
