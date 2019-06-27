#!/usr/bin/env -u python3
# Gets lighthouse position coordinates from a crazyflie. 
import sys
import time
import math 
import openvr

from threading import Timer
from threading import Thread 

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

#This is the logger class
from log_conf import my_Logger

# This was imported for testing swarm functionality
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

#open the VR driver
print('Opening VR driver')
vr = openvr.init(openvr.VRApplication_Other)
print('Opened VR driver')

def find_drones():
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    count = 0
    if len(available) == 1:
        print("One drone found with URI [%s] and name/comment [%s]"%(available[0][0], available[0][1]))
        return available[0][0]
    
    elif len(available) > 1:
        for i in available:
            print ("%s. Interface with URI [%s] found and name/comment [%s]" % ((str(count)), i[0], i[1]))
            count = count + 1 
        num = int(input("Chose your drone "))
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

            #print(max_x-min_x, max_y - min_y, max_z - min_z)

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
    
def create_flight_path(initial_position):
    initial_position['yaw'], initial_position['time'] = 0.0, 4.0 
    
    #set the height the drone will hover at after takeoff. 
    hover_height = 0.5 
    initial_position['z'] = initial_position['z'] + hover_height

    flight_path = [initial_position]


    #enter where you would like the drone to fly to {'x':x, 'y':y, 'z':z, 'yaw':yaw, 'time':time}
    manual_sequence = [{'x':-0.2, 'y':0.4, 'z':0.7, 'yaw':0.0, 'time':3.0},
                       ]
    if (len(manual_sequence) > 0):
        flight_path.extend(manual_sequence)

    
    # allow the users to add coordinates
    add = 'n' #input("Would you like to input more coordinates? (y/n)")
    if add == 'y':
        flight_path = add_coordinates(flight_path)

    return flight_path
        
    
def add_coordinates(flight_path, add):
    print("Current flight path: ", flight_path)
    while add == 'y':
        try:
            x = input("x-coordinate ")
            y = float(input("y-coordinate "))
            z = float(input("z-coordinate "))
            yaw = float(input("yaw "))
            time = input("time ")
            new_coordinate = {'x':x, 'y':y, 'z':z, 'yaw':yaw, 'time':time}
            index = int(input("index "))
            if index > length(flight_path):
                print("Not a valid index, adding to end of list")
                flight_path.append(new_coordinate)
            else:
                flight_path.insert(index,new_coordinate)
            print ("This is the current flight path: ", flight_path)
        except:
            print("You input your coordinate incorrectly")
        add = input("Continue adding coordinates? (y/n)")
    return flight_path
        

    

# causes the crazyflie to hover 1.0 above the initial position
def take_off(cf, hover_height = 0.5, take_off_time = 1.0, sleep_time = 0.1):
    print('taking off')
    steps = int(take_off_time / sleep_time)
    vz = (hover_height) / take_off_time

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)



def land(cf, lawn_logger):
    landing_height = 0.3 
    sleep_time = 0.1
    hover_time = 3.0
    landing_time = 3.0
    offset_x = 0.125
    offset_y = 0.125

    print('landing')

    landing_position = lawn_logger.get_lighthouse_pos()
    landing_position['x'] = landing_position['x']+offset_x
    landing_position['y'] = landing_position['y']+offset_y


    print('retrieved landing position, landing at', landing_position)

    end_time = time.time() + hover_time 
    while time.time() < end_time:
        cf.commander.send_position_setpoint(landing_position['x'],landing_position['y'],landing_position['z']+landing_height, 0.0)
        time.sleep(0.01)

    steps = int(landing_time / sleep_time)
    vz = -landing_height / landing_time
    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def run_sequence(scf, scf2, flight_path, cf_logger, lawn_logger):
    try:
        cf = scf.cf

        take_off(cf)
        
        for coordinate in flight_path: 
            print("Going to (%s,%s,%s), yaw %s for %s seconds" % (coordinate['x'],coordinate['y'],coordinate['z'], coordinate['yaw'], coordinate['time']))
            end_time = time.time() + coordinate['time']
            while time.time() < end_time:
                cf.commander.send_position_setpoint(coordinate['x'],coordinate['y'],coordinate['z'], coordinate['yaw'])
                time.sleep(0.01)   
            print("Crazyflie thinks it is at:", cf_logger.get_lighthouse_pos())

        land(cf, lawn_logger)
        
        cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
    except Exception as e:
        print("Error: ", e)


if __name__ == '__main__':	
    #Must intialize the drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)
    
    # find URI of the Crazyflie to connect to
    uri = 'radio://0/80/2M'
    uri2 = 'radio://0/40/2M'

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf: 
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:
        #Check that the position estimate is calibrated
            reset_estimator(scf)
            reset_estimator(scf2)
            #Setup a configuration that will log lighthouse positioning data 
            cf_logger = my_Logger(scf, uri)
            lawn_logger = my_Logger(scf2, uri2)
            initial_position = cf_logger.get_lighthouse_pos()
            print(initial_position)
            flight_path = create_flight_path(initial_position)
            run_sequence(scf, scf2, flight_path, cf_logger, lawn_logger)


print('shutting down openVR')
openvr.shutdown()
print('openVR has shutdown')

