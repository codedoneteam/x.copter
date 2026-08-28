import asyncio
import unittest
import logging
import math
from xcopter import XArduCopter


connection_string = "udp:127.0.0.1:14550"
# connection_string = "uart:/dev/ttyUSB0"
baud = 115200
takeoff_height = 3
earth_radius_m = 6378137.0

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
    logging.info(capacity)

    battery = await copter.battery()
    voltage = await copter.voltage()
    current = await copter.current()
    logging.info("Battery: %s, voltage: %s, current: %s", battery, voltage, current)

    await copter.guide()
    is_guided = await copter.guided()
    logging.info("Is copter guided: %s", is_guided)

    await copter.arm()
    is_armed = await copter.armed()
    logging.info("Is copter armed = %s", is_armed)

    await asyncio.sleep(10)

    await copter.takeoff(takeoff_height, speed=0.5)

    altitude = await copter.altitude()
    height = await copter.height()
    heading = await copter.heading()
    logging.info(f"Current height: {height} meters. Absolute altitude: {altitude} meters. Heading: {heading} degrees")

    location = await copter.location()
    logging.info("Location: %s", location)

    position = await copter.position()
    logging.info("GPS position: %s", position)

    target_lat, target_lon = target(position["lat"], position["lon"], 30)
    await copter.navigate(
        target_lat,
        target_lon,
        position["alt"],
        speed=3,
        epsilon=0.5,
    )

    await copter.stop()

    await copter.rotate(90, 30)

    await copter.shift(forward=5, up=3, speed=1)
    await copter.shift(right=-15, up=1, speed=1)

    copter.info("Actions completed")

    await copter.rtl()

def target(lat, lon, distance_m):
    target_lat = lat + math.degrees(distance_m / earth_radius_m)
    return target_lat, lon

class MainTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        logging.basicConfig(level=logging.INFO)
        self.copter = XArduCopter()

    async def test_fly_success(self):
        await fly(self.copter)


if __name__ == "__main__":
    unittest.main()
