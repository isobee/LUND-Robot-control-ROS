import logging
import time

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# Only output errors from the logging framework

logging.basicConfig(level=logging.ERROR)


class WriteMem:
    def __init__(self, uri, bs1, bs2):
        self.data_written = False

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)

            count = len(mems)
            if count != 1:
                raise Exception('Unexpected nr of memories found:', count)

            mems[0].geometry_data = [bs1, bs2]

            print('Writing data')
            mems[0].write_data(self._data_written)

            while not self.data_written:
                time.sleep(1)

    def _data_written(self, mem, addr):
        self.data_written = True
        print('Data written')


if __name__ == '__main__':
    # URI to the Crazyflie to connect to
    uri = 'radio://0/80/2M'

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    bs1 = LighthouseBsGeometry()
    bs1.origin = [1.0, 2.0, 3.0]
    bs1.rotation_matrix = [
        [4.0, 5.0, 6.0],
        [7.0, 8.0, 9.0],
        [10.0, 11.0, 12.0],
    ]

    bs2 = LighthouseBsGeometry()
    bs2.origin = [21.0, 22.0, 23.0]
    bs2.rotation_matrix = [
        [24.0, 25.0, 26.0],
        [27.0, 28.0, 29.0],
        [30.0, 31.0, 32.0],
    ]

WriteMem(uri, bs1, bs2)
