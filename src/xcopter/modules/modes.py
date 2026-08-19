import asyncio

from ..mavlink import mavlink, mode_string_v10
from .log import Log
from .errors.error import CopterError

class Modes(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def guide(self, timeout=5):
        async def set_mode(mode):
            mode_mapping = self.master.mode_mapping()
            mode_id = mode_mapping.get(mode)
            if mode_id is None:
                raise CopterError(f"Error: mode '{mode}' not found in mode mapping")

            self.master.set_mode(mode_id)
            self.info(f"Mode change command sent: {mode}")

            ack = await asyncio.to_thread(self.master.recv_match, type='COMMAND_ACK', blocking=True, timeout=timeout)
            if ack is None:
                raise CopterError("Mode change failed: no response received")

            if getattr(ack, "command", mavlink.MAV_CMD_DO_SET_MODE) != mavlink.MAV_CMD_DO_SET_MODE:
                raise CopterError(f"Mode change failed: unexpected ACK command {getattr(ack, 'command', None)}")

            if getattr(ack, "result", None) != mavlink.MAV_RESULT_ACCEPTED:
                raise CopterError(f"Mode change failed, code: {getattr(ack, 'result', None)}")

            return True

        try:
            await set_mode("GUIDED")
        except CopterError as e:
            self.error(str(e))
            raise

        self.info("GUIDED mode set")
        return True
    
    async def guided(self, timeout=5):
        current_mode = await self.mode(timeout=timeout)
        if current_mode is None:
            raise CopterError("Failed to determine current mode")
        is_guided = current_mode.lower() == "guided"
        if not is_guided:
            raise CopterError("GUIDED mode is inactive")
        self.info(f"GUIDED mode is {'active' if is_guided else 'inactive'}")
        return is_guided

    async def mode(self, timeout=5):
        try:
            heartbeat = await asyncio.to_thread(self.master.recv_match, type='HEARTBEAT', blocking=True, timeout=timeout)
            if heartbeat is None:
                raise CopterError("Failed to get mode: heartbeat not received")

            base_mode = heartbeat.base_mode
            custom_mode = heartbeat.custom_mode

            try:
                mode_name = mode_string_v10(heartbeat)
            except Exception:
                mode_name = f"base_mode={base_mode}, custom_mode={custom_mode}"

            self.info(f"Current flight mode: {mode_name}")
            return mode_name
        except Exception as e:
            self.error(f"Exception while getting flight mode: {e}")
            raise CopterError(f"Exception while getting flight mode: {e}") from e
