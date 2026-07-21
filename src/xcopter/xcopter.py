from abc import ABCMeta, abstractmethod

class XCopter(metaclass=ABCMeta):

    @abstractmethod
    async def armed(self, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def arm(self, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def disarm(self, timeout=5, force=True):
        raise NotImplementedError

    @abstractmethod
    async def mode(self, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def land(self, descend=2, land=0.5, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def rtl(self, speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def smart_rtl(self, speed=5, climb=2, descend=2, land=0.5, rtl_alt=20, vert_accel=2, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def guide(self, timeout=5):
        raise NotImplementedError
    
    @abstractmethod
    async def guided(self, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def takeoff(self, target_height, epsilon=0.3, speed = 3, hold_time=3, vert_accel=2, takeoff_timeout=300, timeout=1):
        raise NotImplementedError

    @abstractmethod
    async def altitude(self, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def heading(self, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def height(self, timeout=1):
        raise NotImplementedError

    @abstractmethod
    async def beep(self, tone=2000, duration=1, count=1):
        raise NotImplementedError

    @abstractmethod
    async def location(self, timeout=1):
        raise NotImplementedError

    @abstractmethod
    async def shift(self, forward=0.0, right=0.0, up=0.0, speed = 0.1, epsilon=0.05, shift_timeout=300, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def rotate(self, angle, rate, tolerance=3, rotate_timeout=100, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def position(self, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def navigate(self, lat, lon, alt, speed=5, epsilon=0.5, hz=10, navigation_timeout=600, timeout=60):
        raise NotImplementedError

    @abstractmethod
    async def stop(self, timeout=5):
        raise NotImplementedError
    
    @abstractmethod
    async def close(self):
        raise NotImplementedError

    @abstractmethod
    async def voltage(self):
        raise NotImplementedError

    @abstractmethod
    async def current(self):
        raise NotImplementedError

    @abstractmethod
    async def battery(self):
        raise NotImplementedError

    @abstractmethod
    async def read(self, param, timeout=5):
        raise NotImplementedError

    @abstractmethod
    async def write(self, param, value, param_type=None, timeout=5):
        raise NotImplementedError
