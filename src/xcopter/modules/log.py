import logging
from pymavlink import mavutil

class Log:
    def __init__(self):
        self.master = None

    def debug(self, msg):
        logging.debug(msg)
        self._send_statustext(mavutil.mavlink.MAV_SEVERITY_DEBUG, msg)

    def info(self, msg):
        logging.info(msg)
        self._send_statustext(mavutil.mavlink.MAV_SEVERITY_INFO, msg)

    def warn(self, msg):
        logging.warning(msg)
        self._send_statustext(mavutil.mavlink.MAV_SEVERITY_WARNING, msg)

    def error(self, msg):
        logging.error(msg)
        self._send_statustext(mavutil.mavlink.MAV_SEVERITY_ERROR, msg)

    def critical(self, msg):
        logging.critical(msg)
        self._send_statustext(mavutil.mavlink.MAV_SEVERITY_CRITICAL, msg)

    def _send_statustext(self, severity, msg):
        master = getattr(self, "master", None)
        if master is None or not hasattr(master, "mav"):
            return

        try:
            encoded = msg.encode("utf-8")
            master.mav.statustext_send(severity, encoded[:50])
        except Exception:
            logging.debug("Failed to send MAVLink STATUSTEXT", exc_info=True)
