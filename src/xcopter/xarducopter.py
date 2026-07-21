from pymavlink.dialects.v20 import common as mavlink2
from .xcopter import XCopter
from pymavlink import mavutil
from .modules.arm import Arming
from .modules.gps import Gps
from .modules.modes import Modes
from .modules.takeoff import Takeoff
from .modules.buzzer import Buzzer
from .modules.local import Local
from .modules.rotate import Rotate
from .modules.land import Land
from .modules.param import Params
from .modules.battery import Battery
from .modules.rtl import Rtl
from .modules.heading import Heading
from .modules.move import Move
from .modules.log import Log
import time
import asyncio
import logging
from contextlib import suppress

class XArduCopter(Arming, Modes, Takeoff, Heading, Gps, Buzzer, Local, Rotate, Land, Rtl, Move, Battery, Params, Log, XCopter):
    def __init__(self):
        Log.__init__(self)
        Move.__init__(self, None)
        self.start_time = None
        self._heartbeat_task = None

    async def connect(self, connection_string, baud=115200, source_system=255, source_component=0, gcs=False):
        system = 255 if gcs else source_system

        master = mavutil.mavlink_connection(
            connection_string,
            baud=baud,
            source_system=system,
            source_component=source_component,
        )

        self.master = master
        self.start_time = time.time()

        await asyncio.to_thread(master.wait_heartbeat)

        if gcs:
            self._heartbeat_task = asyncio.create_task(self.heartbeat_loop())


    async def heartbeat_loop(self):
        try:
            while self.master:
                self.master.mav.heartbeat_send(
                    mavlink2.MAV_TYPE_GCS,
                    mavlink2.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                await asyncio.sleep(0.25)
        except asyncio.CancelledError:
            raise

    async def close(self):
        master = self.master
        heartbeat_task = self._heartbeat_task
        self._heartbeat_task = None

        if heartbeat_task:
            self.master = None
            heartbeat_task.cancel()
            with suppress(asyncio.CancelledError):
                await heartbeat_task

        self.master = None

        if master:
            await asyncio.to_thread(master.close)
            logging.info("Connection to the copter closed")
