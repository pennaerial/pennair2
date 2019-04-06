from threading import Thread
from abc import ABCMeta, abstractmethod
from pennair2.core import Multirotor

class Controller(Thread):
    __metaclass__ = ABCMeta

    def __init__(self, quad):
        # type: (Multirotor) -> None
        self.quad = quad  # type: Multirotor
        self.running = True # Standard param to use in run loop
        Thread.__init__(self)

    @abstractmethod
    def run(self):
        pass
