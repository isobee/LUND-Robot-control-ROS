import logging
import time
import threading 
from multiprocessing import Process

#import cflib.crtp
#from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
#from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger



class my_Logger:
    def __init__(self, scf, uri):
        self.scf = scf 
        self.uri = uri 
        self.lighthouse_log_conf = self.setup_lighthouse_logconf()
        self.lighthouse_pos_history = [1000]*10

        self.lighthouse_thread = threading.Thread(target=self.run_logger)
        self.lighthouse_thread.daemon = True
        self.lighthouse_thread.start()

        # Make sure the logger has started running 
        while self.lighthouse_pos_history[-1] == 1000:
            time.sleep(0.1)
    
    def setup_lighthouse_logconf(self):
        log_conf = LogConfig(name='Lighthouse Position', period_in_ms=10) 
        log_conf.add_variable('lighthouse.x', 'float')
        log_conf.add_variable('lighthouse.y', 'float')
        log_conf.add_variable('lighthouse.z', 'float')
        self.scf.cf.log.add_config(log_conf)
        return log_conf

    def run_logger(self):
        with SyncLogger(self.scf, self.lighthouse_log_conf) as logger:
            for log_entry in logger:
                #timestamp = log_entry[0]
                #logconf_name = log_entry[2]
                data = log_entry[1]
                self.lighthouse_pos_history.append(data)
                self.lighthouse_pos_history.pop(0)
                #print('lighthouse.x = %s, lighthouse.y = %s, lighthouse.z = %s' % (data['lighthouse.x'], data['lighthouse.y'], data['lighthouse.z'])) 
    
    def get_lighthouse_pos(self):
        lock = threading.Lock()
        lock.acquire() # will block if lock is already held
        data = self.lighthouse_pos_history[-1]
        lock.release()
        # y and z swap, for some reason 
        position = {'x':(-data['lighthouse.z']), 'y':(-data['lighthouse.x']), 'z':data['lighthouse.y']}
        return position
