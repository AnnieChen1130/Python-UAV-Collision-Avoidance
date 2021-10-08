from planeClass import *
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

import time
import math
import numpy as np 
import psutil
import argparse
import copy    


if __name__ == '__main__':
    #parser = argparse.ArgumentParser()
    #parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
    #args = parser.parse_args()
    
    #connection_string = args.connect

    connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
    
    #-- Create the object
    plane = Plane(connection_string)
    altitude = 30

    if not plane.is_armed(): plane.arm_and_takeoff(altitude)

    avoidWP = [33.9875841, -117.9110992, altitude]

    while plane.is_armed():
        plane.run()

        if(plane.current_WP_number() == 4):
            plane.insert_avoidWP(plane.current_WP_number(), avoidWP)
            print("current_WP_number: ", plane.current_WP_number())
            time.sleep(1)
        print("current_WP_number: ", plane.current_WP_number())   
        time.sleep(1)
    

    '''
    #-- Set in fbwb and test the rc override
    plane.set_ap_mode("FBWB")
    time.sleep(3.0)
    print ("Commanding roll 1100")
    plane.set_rc_channel(1, 1300)
    time.sleep(6)
    print ("Commanding roll 1800")
    plane.set_rc_channel(1, 1700)
    time.sleep(6)
    
    #-- Set in Guided and test the ground course
    plane.set_ap_mode("GUIDED")
    
    cruise_altitude = 100
    cruise_angle_deg= 0.0
    delta_angle_deg = 40.0
    
    while True:
        print ("Heading to %.0fdeg"%cruise_angle_deg)
        plane.set_ground_course(cruise_angle_deg, cruise_altitude)
        time.sleep(5)
        cruise_angle_deg    += delta_angle_deg
    '''
