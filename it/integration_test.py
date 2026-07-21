import unittest
import logging
from xcopter import XArduCopter

takeoff_height = 3
connection_string = "udp:127.0.0.1:14550"
baud = 115200

async def fly(copter):
    try:
        await copter.connect(connection_string, gcs=True, baud=baud)
        await mission(copter)
    except Exception as error:
        logging.exception("Copter mission failed: %s", error)
        await copter.land()
    finally:
        await copter.close()

async def mission(copter):
    await copter.write("BATT_CAPACITY", 5200.0)

    capacity = await copter.read("BATT_CAPACITY")
    copter.info(capacity)

    await copter.guide()

    await copter.arm()

    await copter.takeoff(takeoff_height, speed=0.5)
    copter.info(f"Current altitude: {await copter.height()} meters. Absolute altitude: {await copter.altitude()} meters")

    await copter.rotate(90, 30)

    await copter.shift(forward=5, up=3, speed=1)
    await copter.shift(right=-15, up=1, speed=1)

    copter.info("Actions completed")

    await copter.rtl()

class MainTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        logging.basicConfig(level=logging.INFO)
        self.copter = XArduCopter()

    async def test_fly_success(self):
        await fly(self.copter)


if __name__ == "__main__":
    unittest.main()
