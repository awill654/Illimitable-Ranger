
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#--importing Tkhinter; ssudo apt-get install python-tk
import Tkhinter as tk



#--connect to vehicle
print('Connecting...')
vehicle = connect('udp:#')

#--Set the command flying speed
gnd speed = 5 #[m/s]

#--define arm andd take off
def arm_and_takeoff(altitude)

    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)
        
