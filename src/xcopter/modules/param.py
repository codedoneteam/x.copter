import asyncio
import time

from ..mavlink import mavlink

from .errors.error import CopterError
from .log import Log

class Params(Log):
    def __init__(self):
        super().__init__()
        self.master = None

    async def read(self, param, timeout=5):
        try:
            param_id = self._encode_param_id(param)
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                param_id,
                -1,
            )

            msg = await self._recv_param_value(param_id, timeout)
            self.info(f"Parameter {self._decode_param_id(param_id)} read: {msg.param_value}")
            return msg.param_value
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"Exception while reading parameter {param}: {e}")
            raise CopterError(f"Exception while reading parameter {param}: {e}") from e

    async def write(self, param, value, param_type=mavlink.MAV_PARAM_TYPE_REAL32, timeout=5):
        try:
            param_id = self._encode_param_id(param)
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param_id,
                value,
                param_type,
            )

            msg = await self._recv_param_value(param_id, timeout)
            self.info(f"Parameter {self._decode_param_id(param_id)} written: {msg.param_value}")
            return msg.param_value
        except CopterError as e:
            self.error(str(e))
            raise
        except Exception as e:
            self.error(f"Exception while writing parameter {param}: {e}")
            raise CopterError(f"Exception while writing parameter {param}: {e}") from e

    def _encode_param_id(self, param):
        if isinstance(param, bytes):
            param_id = param
        else:
            param_id = str(param).encode("ascii")

        if not param_id:
            raise CopterError("Parameter name is empty")
        if len(param_id) > 16:
            raise CopterError(f"Parameter name is too long: {self._decode_param_id(param_id)}")

        return param_id

    def _decode_param_id(self, param_id):
        if isinstance(param_id, bytes):
            return param_id.split(b"\x00", 1)[0].decode("ascii", errors="ignore")
        return str(param_id).split("\x00", 1)[0]

    async def _recv_param_value(self, param_id, timeout):
        deadline = time.monotonic() + timeout

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CopterError(f"Parameter {self._decode_param_id(param_id)} response not received")

            msg = await asyncio.to_thread(
                self.master.recv_match,
                type='PARAM_VALUE',
                blocking=True,
                timeout=remaining,
            )
            if msg is None:
                raise CopterError(f"Parameter {self._decode_param_id(param_id)} response not received")

            if self._param_matches(msg.param_id, param_id):
                return msg

    def _param_matches(self, received, expected):
        return self._decode_param_id(received) == self._decode_param_id(expected)
