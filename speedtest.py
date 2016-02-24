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

datalog = Logger()

power_steering = PowerSteering(steeringPower_port_name, 9600, datalog)
time.sleep(5)

compass = Compass(compass_port_name, 9600, datalog)
compass_thread = CompassThread(compass, datalog)
compass_thread.start()

power_steering.set_mode_drive()
time.sleep(5)

robot = Robot(power_steering, compass, 0, 0, datalog)

for i in range(30):
  robot.powersteering.set_pwr_and_steer(1500, 1, compass.get_speed())
  time.sleep(1)

