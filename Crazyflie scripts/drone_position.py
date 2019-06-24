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

def find_drones():
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    count = 0
    if len(available) == 1:
        print("One drone found with URI [%s] and name/comment [%s]"%(available[0][0], available[0][1]))
        return available[0][0]
    
    elif len(available) > 1:
        for i in available:
            print ("%s. Interface with URI [%s] found and name/comment [%s]" % ((str(count)), i[0], i[1]))
            count = count + 1 
        num = int(input("Chose your drone "))
        print ("You have chosen: " + available[num][0])
        return available[num][0]
    
    else:
        print('No Crazyflies found, cannot run')
        sys.exit(1)	

def find_controller():
    controllerId = None
    poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if poses[i].bPoseIsValid:
            device_class = vr.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller or device_class == openvr.TrackedDeviceClass_GenericTracker:
                controllerId = i
                break
    if controllerId is None:
        print('Cannot find controller or tracker, exiting')
        sys.exit(1)
    return controllerId


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
            
def get_controller_position(controllerId):
    poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
    controller_pose = poses[controllerId]
    pose = controller_pose.mDeviceToAbsoluteTracking
    print (pose) 
    return pose

def create_flight_path(initial_position, final_position):
    initial_position['yaw'], initial_position['time'] = 0.0, 1.0 
    final_position['yaw'], final_position['time'] = 0.0, 1.0
    
    #set the height the drone will hover at before landing
    landing_height = 0.3
    final_position['z'] = final_position['z'] + landing_height

    flight_path = []

    flight_path.append(initial_position)

    #enter where you would like the drone to fly to {'x':x, 'y':y, 'z':z, 'yaw':yaw, 'time':time}
    manual_sequence = [{'x':0.8, 'y':0.2, 'z':0.7, 'yaw':0.0, 'time':3.0}]

    print("Current flight path: ", flight_path)
    # allow the users to add coordinates
    add = input("Would you like to input more coordinates?")
    if add == 'y':
        add_coordinates(flight_path)

    flight_path.append(final_position)

    return flight_path
        
    
def add_coordinates(flight_path, add)
    while add == 'y':
        print ("This is the current flight path: ", flight_path)
        x = input("x-coordinate ")
        y = float(input("y-coordinate "))
        z = float(input("z-coordinate "))
        yaw = float(input("yaw "))
        time =input("time ")
        new_coordinate = {'x':x, 'y':y, 'z':z, 'yaw':yaw, 'time':time}


        index = int(input("index "))
        add = input.lower("Continue adding? (y/n)")
        

    

# causes the crazyflie to hover 1.0 above the initial position
def take_off(cf, initial_position, take_off_time = 1.0, sleep_time = 0.1):
    print('taking off')
    steps = int(take_off_time / sleep_time)
    vz = (initial_position['z'] + 1.0) / take_off_time

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

# sends the crazyflie to position, and kills power
def land(cf, final_position, landing_time = 1.0, sleep_time = 0.1):
    print('landing')
    steps = int(landing_time / sleep_time)
    vz = -0.3 / landing_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def run_sequence(scf, flight_path):
    try:
        cf = scf.cf

        take_off(cf, flight_path[0])
        
        for coordinate in flight_path: 
            print("Going to (%s,%s,%s), yaw %s for %s seconds" % (coordinate['x'],coordinate['y'],coordinate['z'], coordinate['yaw'], coordinate['time'])
            end_time = time.time() + coordinate['time']
            while time.time() < end_time:
                cf.commander.send_position_setpoint(coordinate['x'],coordinate['y'],coordinate['z'],coordinate['yaw'])
                time.sleep(0.01)

        land(cf, flight_path[-1])
        
        cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
    except Exception as e:
        print(e)


if __name__ == '__main__':	
    #Must intialize the drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)
    
    # find URI of the Crazyflie to connect to
    uri = find_drones()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #Check that the position estimate is calibrated
        reset_estimator(scf)
        #Setup a configuration that will log lighthouse positioning data 
        lighthouse_log_conf = setup_logconf(scf)
        initial_position = get_cf_position(scf, lighthouse_log_conf)
        final_position = initial_position
        flight_path = create_flight_path(initial_position, final_position)
        #print initial_position
        run_sequence(scf,initial_position)


print('shutting down openVR')
openvr.shutdown()
print('openVR has shutdown')

