from planeClass_CA import *
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

import time
import math
import numpy as np 
import psutil
import argparse
import copy    




if __name__ == '__main__':

    connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
    
    #-- Create the object
    plane = Plane(connection_string)
    altitude = 30

    #plane.run()

    if not plane.is_armed(): plane.arm_and_takeoff(altitude)

    avoidWP = [33.9875841, -117.9110992, altitude]
    
    plane.run()
    
    while plane.is_armed():
    
        if(plane.current_WP_number() == 4):
            plane.insert_avoidWP(plane.current_WP_number(), avoidWP)
            print("current_WP_number: ", plane.current_WP_number())
            time.sleep(1)
        print("current_WP_number: ", plane.current_WP_number())   
        time.sleep(1)
    










