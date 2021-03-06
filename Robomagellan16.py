#!/usr/bin/python

# Robomagellan 2016
#

# Import all the things
import time
from logger import Logger
from powersteering import *
from compass import *
from gps import *
from camera import *
from robot import *

#Port Definitions
steeringPower_port_name = '/dev/ttyACM0'
compass_port_name = '/dev/ttyACM1'
GPS_port_name = '/dev/ttyUSB0'

def june16course():
  drive_to_cone(1600, 33.77845666666666, 118.41903666666667)
  drive_to_cone(1600, 33.77858166666667, 118.41884166666667)

def july7course():
  drive_to_cone(1600, 33.77854, 118.418916667)
  drive_to_cone(1600, 33.7786883333333335, 118.41892)

def july14course():
  drive_to_cone(1600, 33.77849666666667, 118.41903166666667)
  drive_to_cone(1600, 33.77864833333334, 118.41890666666667)

def aug11course():
  drive_to_cone(1600, 33.77851666666667, 118.41900833333334)
  drive_to_cone(1600, 33.77867166666667, 118.41894333333333)

def nov8course():
  drive_to_cone(1600, 33.77864666666667, 118.41893166666667)
  drive_to_cone(1600, 33.778468333333336, 118.41905)

def dec6course(robot):
  robot.drive_to_cone(1600, 33.77868, 118.41895166666667)
  robot.drive_to_cone(1600, 33.778513333333336, 118.41902)

def dec20course(robot):
  robot.drive_to_waypoint(1600, 33.77862833333333, 118.41897)
  robot.drive_to_waypoint(1600, 33.778395, 118.41895666666667)
  robot.drive_to_waypoint(1600, 33.77862833333333, 118.41897)

def jan10course(robot):
  robot.drive_to_cone(2, 33.778621666666666, 118.41890833333333)
  robot.drive_to_cone(2, 33.778481666666664, 118.41897333333333)

def jan24course(robot):
  robot.drive_to_cone(2, 33.779722, 118.41888888)
  
def feb16course(robot):
  robot.drive_to_cone(2, 33.77863833333333, 118.41891333333334)
  robot.drive_to_cone(2, 33.778286666666666, 118.41896166666666)
  robot.drive_to_cone(2, 33.77840833333333, 118.41915166666666)
  robot.drive_to_cone(2, 33.77863833333333, 118.41891333333334)

def mar6course(robot):
  robot.drive_to_cone(2, 33.77861, 118.41888333333333)
  robot.drive_to_cone(2, 33.778515, 118.41898666666667)

def mar13course(robot):
  robot.drive_to_cone(1.5, 33.77858833333333, 118.41896833333334)
  robot.drive_to_cone(1.5, 33.77849333333333, 118.41899833333333)

#========================================================
#Main program starts here
#========================================================
def main():
    # Start logging
    datalog = Logger()

    # Start the power/steering serial port (causes the Arduido to reset so
    # wait a few seconds for that to happen)
    power_steering = PowerSteering(steeringPower_port_name, 9600, datalog)
    time.sleep(5)

    # Start the compass serial port
    compass = Compass(compass_port_name, 9600, datalog)

    # Start the GPS serial port
    gps = GPS(GPS_port_name, 4800, datalog)

    # Set the Arduino to Drive Mode by sending control string
    power_steering.set_mode_drive()
    time.sleep(5)

    compass_thread = CompassThread(compass, datalog)
    compass_thread.start()

    gps_thread = GPSThread(gps, datalog)
    gps_thread.start()

    camera = Camera(datalog)
    camera_thread = CameraThread(camera, datalog)
    camera_thread.start()

    robot = Robot(power_steering, compass, gps, camera, datalog)

    try:
        gps.get_GPS()
        datalog.write("Armed and Ready - Press Button")
        while compass.get_bump_switch_state() == False:
            pass
        datalog.write("Go!!!")
        mar13course(robot)

    except KeyboardInterrupt:
        pass

    #========================================================
    #Main program stops here
    #========================================================
    robot.stop_driving()
    power_steering.set_mode_stop()

    compass_thread.join()
    gps_thread.join()
    camera_thread.join()
    time.sleep(5)
    datalog.write('Done!!!')
    del datalog


if __name__ == "__main__":
    main()
