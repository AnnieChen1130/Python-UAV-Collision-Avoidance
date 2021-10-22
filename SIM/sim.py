from planeClass_SIM import *
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

import time
import math
import numpy as np 
import psutil
import argparse
import copy
import datetime
import threading
from  threading import *


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
    args = parser.parse_args()
    
    connection_string = args.connect
    #connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK  
    
    #-- Create the object
    plane = Plane(connection_string)

    
    #-- Arm and takeoff
    if not plane.is_armed(): plane.arm_and_takeoff(altitude=100)

    avoidWP = [34.0793936, -117.6002312, 100]
    plane.run()
    
    while plane.is_armed():
        if(plane.current_WP_number() == 3):
            plane.insert_avoidWP(plane.current_WP_number(), avoidWP)
            #print("current_WP_number: ", plane.current_WP_number())
            time.sleep(1)
        #print("current_WP_number: ", plane.current_WP_number())   
        time.sleep(1)
    
















