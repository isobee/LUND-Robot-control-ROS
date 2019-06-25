"""
Example of how to read the Lighthouse base station geometry memory from a
Crazyflie
"""
import logging
import time

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# Only output errors from the logging framework

logging.basicConfig(level=logging.ERROR)


class ReadMem:
    def __init__(self, uri):
        self.got_data = False

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)

            count = len(mems)
            if count != 1:
                raise Exception('Unexpected nr of memories found:', count)

            print('Rquesting data')
            mems[0].update(self._data_updated)

            while not self.got_data:
                time.sleep(1)

    def _data_updated(self, mem):
        mem.dump()
        self.got_data = True


if __name__ == '__main__':
    # URI to the Crazyflie to connect to
    uri = 'radio://0/80/2M'

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

ReadMem(uri)
