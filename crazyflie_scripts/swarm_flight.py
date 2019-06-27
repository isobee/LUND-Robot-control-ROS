#!/usr/bin/env -u python3
# Gets lighthouse position coordinates from a crazyflie. 
import sys
import time
import math 
from threading import Timer

import openvr

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D

#open the VR driver
print('Opening VR driver')
vr = openvr.init(openvr.VRApplication_Other)
print('Opened VR driver')

#Find Drones 
print('Scanning interfaces for Crazyflies...')
available = cflib.crtp.scan_interfaces()
uris = []
    
if len(available) > 0:
    for i in available:
        print ("Interface with URI [%s] found and name/comment [%s]" % (i[0], i[1]))
else:
    print('No Crazyflies found, cannot run')
    sys.exit(1)	

URI0 = available[0][0]
URI1 = available[1][0]

uris = {
    URI0,
    URI1,
}


# sequences are (x,y,z,yaw,time)
default_sequence = [(0.0,0.0,0.0,0.0, 0.0)]

x0 = 0.5
y0 = -0.5
z0 = 0.7 


sequence0 = [
    (x0, y0, z0, 0.0, 3.0),
    (x0, y0, z0, 0.0, 2.0),
    (x0, y0, z0, 0.0, 3.0),
]
sequence1 = [
    (x0, y0, z0, 0.0, 3.0),
    (x0, y0, z0, 0.0, 2.0),
    (x0, y0, z0, 0.0, 3.0),
]


seq_args{URI} = {
    URI0:sequence0,
    URI1:sequence1,
}


    

def wait_for_position_estimator(cf):
    print('Waiting for estimator to find position...')
	# setup a logging configuration 
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')


    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(cf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

        print('estimator has found position')

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)
    


def setup_logconf(scf):
    log_conf = LogConfig(name='Lighthouse Position', period_in_ms=500) 
    log_conf.add_variable('lighthouse.x', 'float')
    log_conf.add_variable('lighthouse.y', 'float')
    log_conf.add_variable('lighthouse.z', 'float')
    scf.cf.log.add_config(log_conf)
    return log_conf


def get_cf_position(scf, log_conf):
    print('getting crazyflie position')
    with SyncLogger(scf, log_conf) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            #print('timestamp:%d \t logconf_name: %s' % (timestamp, logconf_name))
            #print('lighthouse.x = %s, lighthouse.y = %s, lighthouse.z = %s' % (data['lighthouse.x'], data['lighthouse.y'], data['lighthouse.z']))
            break 
    return {'x':data['lighthouse.x'], 'y':data['lighthouse.y'], 'z':data['lighthouse.z']} 
                

# causes the crazyflie to hover 1.0 above the initial position
def take_off(cf, initial_position, hover_height = 0.5, take_off_time = 1.0, sleep_time = 0.1):
    print('taking off')
    steps = int(take_off_time / sleep_time)
    vz = (hover_height) / take_off_time

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

# sends the crazyflie to position, and kills power
def land(cf, final_position, landing_height = 0.3, landing_time = 1.0, sleep_time = 0.1):
    print('landing')
    steps = int(landing_time / sleep_time)
    vz = -landing_height / landing_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def run_sequence(scf, sequence):
    try:
        cf = scf.cf

        take_off(cf, sequence[0])
        for position in sequence:
            print('Setting position {}'.format(position))
            end_time = time.time() + position[3]
            while time.time() < end_time:
                cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2], 0)
                time.sleep(0.1)
        land(cf, sequence[-1])
    except Exception as e:
        print(e)


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        # If the copters are started in their correct positions this is
        # probably not needed. The Kalman filter will have time to converge
        # any way since it takes a while to start them all up and connect. We
        # keep the code here to illustrate how to do it.
        # swarm.parallel(reset_estimator)

        # The current values of all parameters are downloaded as a part of the
        # connections sequence. Since we have 10 copters this is clogging up
        # communication and we have to wait for it to finish before we start
        # flying.
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(wait_for_param_download)

        swarm.parallel(run_sequence, args_dict=seq_args)


print('shutting down openVR')
openvr.shutdown()
print('openVR has shutdown')

