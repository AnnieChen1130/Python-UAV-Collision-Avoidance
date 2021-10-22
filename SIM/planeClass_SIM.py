""" Header
Calling:
plane --connect <connection_string>

connection_string: i.e. tcp:ip_address:port / udp:ip_address:port / comport,baudrate

You can also create the connection on a separate file with vehicle = connect(..) and then
initialize plane = Plane(vehicle), so that you can use the object in your own program


"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude, LocationLocal
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
import serial

#setting up xbee communication

ser = serial.Serial(
    
    port='COM9',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1   
)

class Plane():

    def __init__(self, connection_string=None, vehicle=None):
        """ Initialize the object
        Use either the provided vehicle object or the connections tring to connect to the autopilot
        
        Input:
            connection_string       - the mavproxy style connection string, like tcp:127.0.0.1:5760
                                      default is None
            vehicle                 - dronekit vehicle object, coming from another instance (default is None)
        
        
        """
        
        #---- Connecting with the vehicle, using either the provided vehicle or the connection string
        if not vehicle is None:
            self.vehicle    = vehicle
            print("Using the provided vehicle")
        elif not connection_string is None:
            
            print("Connecting with vehicle...")
            self._connect(connection_string)
        else:
            raise("ERROR: a valid dronekit vehicle or a connection string must be supplied")
            return
            
        self._setup_listeners()

        self.airspeed           = 0.0       #- [m/s]    airspeed
        self.groundspeed        = 0.0       #- [m/s]    ground speed
        
        self.pos_lat            = 0.0       #- [deg]    latitude
        self.pos_lon            = 0.0       #- [deg]    longitude
        self.pos_alt_rel        = 0.0       #- [m]      altitude relative to takeoff
        self.pos_alt_abs        = 0.0       #- [m]      above mean sea level
        
        self.att_roll_deg       = 0.0       #- [deg]    roll
        self.att_pitch_deg      = 0.0       #- [deg]    pitch
        self.att_heading_deg    = 0.0       #- [deg]    magnetic heading
        
        self.wind_dir_to_deg    = 0.0       #- [deg]    wind direction (where it is going)
        self.wind_dir_from_deg  = 0.0       #- [deg]    wind coming from direction
        self.wind_speed         = 0.0       #- [m/s]    wind speed
        
        self.climb_rate         = 0.0       #- [m/s]    climb rate
        self.throttle           = 0.0       #- [ ]      throttle (0-100)
        
        self.ap_mode            = ''        #- []       Autopilot flight mode
        
        self.mission            = self.vehicle.commands #-- mission items
        
        self.location_home      = LocationGlobalRelative(0,0,0) #- LocationRelative type home
        self.location_current   = LocationGlobalRelative(0,0,0) #- LocationRelative type current position
        
    def _connect(self, connection_string):      #-- (private) Connect to Vehicle
        """ (private) connect with the autopilot
        
        Input:
            connection_string   - connection string (mavproxy style)
        """
        self.vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=60)
        self._setup_listeners()
        
    def _setup_listeners(self):                 #-- (private) Set up listeners
        #----------------------------
        #--- CALLBACKS
        #----------------------------
        if True:    
            #---- DEFINE CALLBACKS HERE!!!
            @self.vehicle.on_message('ATTITUDE')   
            def listener(vehicle, name, message):          #--- Attitude
                self.att_roll_deg   = math.degrees(message.roll)
                self.att_pitch_deg  = math.degrees(message.pitch)
                self.att_heading_deg = math.degrees(message.yaw)%360
                
            @self.vehicle.on_message('GLOBAL_POSITION_INT')       
            def listener(vehicle, name, message):          #--- Position / Velocity                                                                                                             
                self.pos_lat        = message.lat*1e-7
                self.pos_lon        = message.lon*1e-7
                self.pos_alt_rel    = message.relative_alt*1e-3
                self.pos_alt_abs    = message.alt*1e-3
                self.location_current = LocationGlobalRelative(self.pos_lat, self.pos_lon, self.pos_alt_rel)
                
                
            @self.vehicle.on_message('VFR_HUD')
            def listener(vehicle, name, message):          #--- HUD
                self.airspeed       = message.airspeed
                self.groundspeed    = message.groundspeed
                self.throttle       = message.throttle
                self.climb_rate     = message.climb 
                
            @self.vehicle.on_message('WIND')
            def listener(vehicle, name, message):          #--- WIND
                self.wind_speed         = message.speed
                self.wind_dir_from_deg  = message.direction % 360
                self.wind_dir_to_deg    = (self.wind_dir_from_deg + 180) % 360
                        
            
        return (self.vehicle)
        print(">> Connection Established")

    def _get_location_metres(self, original_location, dNorth, dEast, is_global=False):
        """
        Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location has the same `alt and `is_relative` values
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        
        if is_global:
            return LocationGlobal(newlat, newlon,original_location.alt)    
        else:
            return LocationGlobalRelative(newlat, newlon,original_location.alt)         
        
    def is_armed(self):                         #-- Check whether uav is armed
        """ Checks whether the UAV is armed
        
        """
        return(self.vehicle.armed)
        
    def arm(self):                              #-- arm the UAV
        """ Arm the UAV
        """
        self.vehicle.armed = True
        
    def disarm(self):                           #-- disarm UAV
        """ Disarm the UAV
        """
        self.vehicle.armed = False

    def set_airspeed(self, speed):              #--- Set target airspeed
        """ Set uav airspeed m/s
        """
        self.vehicle.airspeed = speed
        
    def set_ap_mode(self, mode):                #--- Set Autopilot mode
        """ Set Autopilot mode
        """
        time_0 = time.time()
        try:
            tgt_mode    = VehicleMode(mode)
        except:
            return(False)
            
        while (self.get_ap_mode() != tgt_mode):
            self.vehicle.mode  = tgt_mode
            time.sleep(0.2)
            if time.time() < time_0 + 5:
                return (False)

        return (True)
        
    def get_ap_mode(self):                      #--- Get the autopilot mode
        """ Get the autopilot mode
        """
        self._ap_mode  = self.vehicle.mode
        return(self.vehicle.mode)
        
    def clear_mission(self):                    #--- Clear the onboard mission
        """ Clear the current mission.
        
        """
        cmds = self.vehicle.commands
        self.vehicle.commands.clear()
        self.vehicle.flush()

        # After clearing the mission you MUST re-download the mission from the vehicle
        # before vehicle.commands can be used again
        # (see https://github.com/dronekit/dronekit-python/issues/230)
        self.mission = self.vehicle.commands
        self.mission.download()
        self.mission.wait_ready()

    def download_mission(self):                 #--- download the mission
        """ Download the current mission from the vehicle.
        
        """
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready() # wait until download is complete.  
        self.mission = self.vehicle.commands

    def mission_add_takeoff(self, takeoff_altitude=50, takeoff_pitch=15, heading=None):
        """ Adds a takeoff item to the UAV mission, if it's not defined yet
        
        Input:
            takeoff_altitude    - [m]   altitude at which the takeoff is considered over
            takeoff_pitch       - [deg] pitch angle during takeoff
            heading             - [deg] heading angle during takeoff (default is the current)
        """
        if heading is None: heading = self.att_heading_deg
        
        self.download_mission()
        #-- save the mission: copy in the memory
        tmp_mission = list(self.mission)
        print(type(self.mission))
        
        print ("Mission Size: ", len(tmp_mission))
        is_mission  = False
        if len(tmp_mission) >= 1:
            is_mission = True
            print("Current mission:")
            for item in tmp_mission:
                print(item)
            #-- If takeoff already in the mission, do not do anything
            
        if is_mission and tmp_mission[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            print ("Takeoff already in the mission")
        else:
            print("Takeoff not in the mission: adding")
            self.clear_mission()
            takeoff_item = Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, takeoff_pitch,  0, 0, heading, 0,  0, takeoff_altitude)
            self.mission.add(takeoff_item)
            for item in tmp_mission:
                self.mission.add(item)
            self.vehicle.flush()
            print(">>>>>Done")
        
    def arm_and_takeoff(self, altitude=50, pitch_deg=12):
        """ Arms the UAV and takeoff
        Planes need a takeoff item in the mission and to be set into AUTO mode. The 
        heading is kept constant
        
        Input:
            altitude    - altitude at which the takeoff is concluded
            pitch_deg   - pitch angle during takeoff
        """
        self.mission_add_takeoff(takeoff_altitude=1.5*altitude, takeoff_pitch=pitch_deg)
        print ("Takeoff mission ready")
        
        while not self.vehicle.is_armable:
            print("Wait to be armable...")
            time.sleep(1.0)
            
        
        #-- Save home
        while self.pos_lat == 0.0:
            time.sleep(0.5)
            print ("Waiting for good GPS...")
        self.location_home      = LocationGlobalRelative(self.pos_lat,self.pos_lon,altitude)
        
        print("Home is saved as "), self.location_home
        print ("Vehicle is Armable: try to arm")
        self.set_ap_mode("MANUAL")
        n_tries = 0
        while not self.vehicle.armed:
            print("Try to arm...")
            self.arm()
            n_tries += 1
            time.sleep(2.0) 
            
            if n_tries > 5:
                print("!!! CANNOT ARM")
                break
                
        #--- Set to auto and check the ALTITUDE
        if self.vehicle.armed: 
            print ("ARMED")
            self.set_ap_mode("AUTO")
            
            while self.pos_alt_rel <= altitude:# - 10.0:
                print ("Altitude = %.0f"%self.pos_alt_rel)
                time.sleep(0.5)

            #self.set_airspeed(50)
            
            #print("Altitude reached: set to GUIDED")
            #self.set_ap_mode("GUIDED")

            #time.sleep(5.0)
            
            #print("Set to AUTO")
            #self.set_ap_mode("AUTO")

            
        return True
    
    def current_WP_number(self):
        return self.vehicle.commands.next


    def insert_avoidWP(self,currentWP_index, avoidWP):
        """Annie's Code
            insert avoid wp into mission list and upload to vehicle
       """
        #CMD mission to list
        missionlist=list(self.mission)

        #create cmd wapoint
        newCMD=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, avoidWP[0], avoidWP[1], avoidWP[2])

        #insert avoid wp to list
        missionlist.insert(currentWP_index,newCMD)

        # Clear the current mission (command is sent when we call upload())
        self.mission.clear()

        #Write the modified mission and flush to the vehicle
        for cmd in missionlist:
            self.mission.add(cmd)
        self.mission.upload()

        
    def get_target_from_bearing(self, original_location, ang, dist, altitude=None):
        """ Create a TGT request packet located at a bearing and distance from the original point
        
        Inputs:
            ang     - [rad] Angle respect to North (clockwise) 
            dist    - [m]   Distance from the actual location
            altitude- [m]
        Returns:
            location - Dronekit compatible
        """
        
        if altitude is None: altitude = original_location.alt
        
        # print '---------------------- simulate_target_packet'
        dNorth  = dist*math.cos(ang)
        dEast   = dist*math.sin(ang)
        # print "Based on the actual heading of %.0f, the relative target's coordinates are %.1f m North, %.1f m East" % (math.degrees(ang), dNorth, dEast) 
        
        #-- Get the Lat and Lon
        tgt     = self._get_location_metres(original_location, dNorth, dEast)
        
        tgt.alt = altitude
        # print "Obtained the following target", tgt.lat, tgt.lon, tgt.alt

        return tgt      

    def ground_course_2_location(self, angle_deg, altitude=None):
        """ Creates a target to aim to in order to follow the ground course
        Input:
            angle_deg   - target ground course
            altitude    - target altitude (default the current)
        
        """
        tgt = self.get_target_from_bearing(original_location=self.location_current, 
                                             ang=math.radians(angle_deg), 
                                             dist=5000,
                                             altitude=altitude)
        return(tgt)
        
    def goto(self, location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        self.vehicle.simple_goto(location)
 
    def set_ground_course(self, angle_deg, altitude=None):
        """ Set a ground course
        
        Input:
            angle_deg   - [deg] target heading
            altitude    - [m]   target altitude (default the current)
        
        """
        
        #-- command the angles directly
        self.goto(self.ground_course_2_location(angle_deg, altitude))
        
    def get_rc_channel(self, rc_chan, dz=0, trim=1500):         #--- Read the RC values from the channel
        """
        Gets the RC channel values with a dead zone around trim
        
        Input:
            rc_channel  - input rc channel number
            dz          - dead zone, within which the output is set equal to trim
            trim        - value about which the dead zone is evaluated
            
        Returns:
            rc_value    - [us]
        """
        if (rc_chan > 16 or rc_chan < 1):
            return -1
        
        #- Find the index of the channel
        strInChan = '%1d' % rc_chan
        try:
        
            rcValue = int(self.vehicle.channels.get(strInChan))
            
            if dz > 0:
                if (math.fabs(rcValue - trim) < dz):
                    return trim
            
            return rcValue
        except:
            return 0     
    
    def set_rc_channel(self, rc_chan, value_us=0):      #--- Overrides a rc channel (call with no value to reset)
        """
        Overrides the RC input setting the provided value. Call with no value to reset
        
        Input:
            rc_chan     - rc channel number
            value_us    - pwm value 
        """
        strInChan = '%1d' % rc_chan
        self.vehicle.channels.overrides[strInChan] = int(value_us)
                
    def clear_all_rc_override(self):               #--- clears all the rc channel override
        self.vehicle.channels.overrides = {}

    def prediction(self):
        times = 0        
        distX = 0
        distY = 0
        distZ = 0
        timer = 0

        collisionPredicted = 0
        XAvoidTolerance = 10.0
        YAvoidTolerance = 10.0
        ZAvoidTolerance = 10.0
 
        while not vehicle.armed:
            print("Not Predicting")
            time.sleep(10)
        while vehicle.armed:
            global velX
            global velY
            global velZ
            global posX
            global posY
            global posZ
            velX = float(vehicle.velocity[0])
            velY = float(vehicle.velocity[1])
            velZ = float(vehicle.velocity[2])
            posX = float(vehicle.location.global_relative_frame.lat)*139
            posY = float(vehicle.location.global_relative_frame.lon)*111
            posZ = float(vehicle.location.global_relative_frame.alt)
 
            global v2velX
            global v2velY
            global v2velZ
            global v2posX
            global v2posY
            global v2posZ
            v2velX = float(vehicle2.velocity[0])
            v2velY = float(vehicle2.velocity[1])
            v2velZ = float(vehicle2.velocity[2])
            v2posX = float(vehicle2.location.global_relative_frame.lat)*139 
            v2posY = float(vehicle2.location.global_relative_frame.lon)*111
            v2posZ = float(vehicle2.location.global_relative_frame.alt)
            
            
            print("VELOCITY: %s"%vehicle.velocity)  
            
            print("Vehicle 1 velocity X is: %f m/s"%velX)          
            print("Vehicle 1 velocity Y is: %f m/s"%velY)
            print("Vehicle 1 velocity Z is: %f m/s"%velZ)
            print("Vehicle 1 alt is: %f m" %posZ)
            print("Vehicle 1 position X: %f km" %posX)
            print("Vehicle 1 position Y: %f km" %posY)
 
            
            print("Vehicle 2 velocity X is: %f m/s"%v2velX)          
            print("Vehicle 2 velocity Y is: %f m/s"%v2velY)
            print("Vehicle 2 velocity Z is: %f m/s"%v2velZ)
            print("Vehicle 2 alt is: %f m" %v2posZ)
            print("Vehicle 2 position X: %f km" %v2posX)
            print("Vehicle 2 position Y: %f km" %v2posY)
            
            #vehicle_1.instructerInfo(self)
            #print("Vehicle 1 velocity X is: %f m/s"%velX)
            print("Vehicle 1 position X: %f km" %posX)
            print("Vehicle 1 position Y: %f km" %posY)
            #print("Vehicle 2 velocity X is: %f m/s"%v2velX)
            print("Vehicle 2 position X: %f km" %v2posX)
            print("Vehicle 2 position Y: %f km" %v2posY)
            
            
 
            timestep = 1
 
            for i in range (10):
                distX = vehicle_1.getFutureDistanceY(timestep, posX, velX, v2posX, v2velX)
                print("    X distance is %s m"%distX, " in %s seconds"%timestep)
                distY = vehicle_1.getFutureDistanceX(timestep, posY, velY, v2posY, v2velY)
                print("    Y distance is %s m"%distY, " in %s seconds"%timestep)        
        
                distZ = vehicle_1.getFutureDistanceZ(timestep, posZ, velZ, v2posZ, v2velZ)
                #print("    Z distance is %s m"%distZ, " in %s seconds"%timestep)
                times = times + 0.01
                timestep = timestep + 0.5
                collisionPredicted = vehicle_1.collisionPredictedCompare(collisionPredicted, distX, distY, distZ, XAvoidTolerance, YAvoidTolerance, ZAvoidTolerance)
                if collisionPredicted:
                    print("************************************************************")
                    print("                  Predicted Collision")
                    print(" ")
                    print("predicted collision at (%f,"%vehicle.location.global_relative_frame.lat, " %f)"%vehicle.location.global_relative_frame.lon)
                    print("************************************************************")
                    vehicle_1.avoid(v2posX, posX, v2posY, posY, posZ)
 
            time.sleep(5)


    def save_to_file(self):
        
        shortDate = datetime.datetime.today().strftime('%Y_%m_%d')   
        outputFile = "log_output_" + shortDate + ".txt"
        f = open(outputFile, "a")
        
        lastGPS = [0,0]
        secondTolastGPS = [0,0]
            
        while self.is_armed():
            timeNow = str(datetime.datetime.now())
            f.write(timeNow + " : " + "~~~~~~~~~~New Point~~~~~~~~~~~~" + '\n')
            f.write(timeNow + " : " + "Current Airspped : " + str(self.airspeed) + '\n')
            f.write(timeNow + " : " + "Current X Velocity : " + str(self.vehicle.velocity[0]) + '\n')
            f.write(timeNow + " : " + "Current Y Velocity : " + str(self.vehicle.velocity[1]) + '\n')
            f.write(timeNow + " : " + "Current lattitude : " + str(self.pos_lat) + '\n')
            f.write(timeNow + " : " + "Current longitude : " + str(self.pos_lon) + '\n')
            f.write(timeNow + " : " + "last lattitude : " + str(lastGPS[0]) + '\n')
            f.write(timeNow + " : " + "last longitude : " + str(lastGPS[1]) + '\n')
            f.write(timeNow + " : " + "second to last lattitude : " + str(secondTolastGPS[0]) + '\n')
            f.write(timeNow + " : " + "second to last longitude : " + str(secondTolastGPS[1]) + '\n')

            secondTolastGPS = [lastGPS[0],lastGPS[0]]
            lastGPS = [self.pos_lat,self.pos_lon]

            timestep = 1
            for i in range(10):    
                f.write(timeNow + " : " + "Timestamp" + str(i) + '\n')    
                futurePosX = self.getFuturePosition(self.pos_lat*139, self.vehicle.velocity[0], timestep)
                futurePosY = self.getFuturePosition(self.pos_lon*111, self.vehicle.velocity[1], timestep)
                f.write(timeNow + " : " + "futurePosX : " + str((futurePosX/1000)/139) + '\n')
                f.write(timeNow + " : " + "futurePosY : " + str((futurePosY/1000)/111) + '\n')

                timestep = timestep + 0.5
            
            
            
            time.sleep(1.0)

        f.close()

    def getFuturePosition(self, PosX,VelX,time):
        futurePosX = PosX*1000 + VelX * time
        #futurePosY = PosY*1000 + VelY * time
        #futurePosZ = PosZ + VelZ * time
        return futurePosX

    def send_ADSB_data(self):
        
        print("In send ADSB funtion\n")
        #msg = "In send ADSB funtion\n"
        #ser.write(msg.encode())
        while True: 

            msg = "ICAO: SIM\n"
            msg += "Lattitude: " + str(self.pos_lat) + '\n'
            msg += "Longitude: " + str(self.pos_lon) + '\n'
            msg += "Altitude: " + str(self.pos_alt_rel) + '\n'
            msg += "Velocity: " + str(self.vehicle.velocity) + '\n'
            msg += "Airspeed: " + str(self.airspeed) + '\n'
            msg += "#######################\n"

            #Send out ADSB data
            ser.write(msg.encode())
            time.sleep(5.0)
    
    def receive_ADSB_data(self):
        print("In receive_ADSB_data function")
        while True:    
            msg = ser.readline().decode()
            print(msg)
            time.sleep(0.1)




    def run(self):
        t1 = threading.Thread(target=self.save_to_file, daemon=True)
        t2 = threading.Thread(target=self.send_ADSB_data, daemon=True)
        t3 = threading.Thread(target=self.receive_ADSB_data, daemon=True)

        t1.start()
        t2.start()
        t3.start()

        
  
