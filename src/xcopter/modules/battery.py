import asyncio
from .log import Log
from .errors.error import CopterError

class Battery(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def voltage(self, timeout=5) -> float:
        try:
            msg = await asyncio.to_thread(self.master.recv_match, type='SYS_STATUS', blocking=True, timeout=timeout)
            if not msg:
                raise CopterError("Failed to get voltage: SYS_STATUS not received")

            voltage: int | None = getattr(msg, "voltage_battery", None)
            if voltage is None:
                raise CopterError("Failed to get voltage: voltage_battery is missing")
            if voltage == 65535:
                raise CopterError("Failed to get voltage: voltage_battery not provided")

            return voltage / 1000.0
        except Exception as e:
            self.error(f"Exception while getting voltage: {e}")
            raise CopterError(f"Exception while getting voltage: {e}") from e

    async def battery(self, timeout=5) -> int:
        try:
            msg = await asyncio.to_thread(self.master.recv_match, type='SYS_STATUS', blocking=True, timeout=timeout)
            if not msg:
                raise CopterError("Failed to get battery: SYS_STATUS not received")

            battery: int | None = getattr(msg, "battery_remaining", None)
            if battery is None:
                raise CopterError("Failed to get battery: battery_remaining is missing")
            if battery == -1:
                raise CopterError("Failed to get battery: battery_remaining not provided")

            return 100 - battery
        except Exception as e:
            self.error(f"Exception while getting battery: {e}")
            raise CopterError(f"Exception while getting battery: {e}") from e

    async def current(self, timeout=5) -> float:
        try:
            msg = await asyncio.to_thread(self.master.recv_match, type='SYS_STATUS', blocking=True, timeout=timeout)
            if not msg:
                raise CopterError("Failed to get current: SYS_STATUS not received")

            current: int | None = getattr(msg, "current_battery", None)
            if current is None:
                raise CopterError("Failed to get current: current_battery is missing")
            if current == -1:
                raise CopterError("Failed to get current: current_battery not provided")

            return current / 100.0
        except Exception as e:
            self.error(f"Exception while getting current: {e}")
            raise CopterError(f"Exception while getting current: {e}") from e
