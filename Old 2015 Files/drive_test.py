#!/usr/bin/python

# Drive Test
# 
#     This program works in association with the Arduino
#     sketch: mon2_chia_new_Vn
#
#     Usage: drive_test.py
#
import serial
import signal
import time
import sys
import os.path
from threading import Thread

compass_value = 0
compass_run_flag = False

steering_control = 1500
power_control = 1500
steering_gain = 2.5

def servo_convert(mcp_value):
  return mcp_value * 5 + 1500		    

# Start the power/steering serial port (causes the Arduido to reset so
# wait a few seconds for that to happen
port_name = '/dev/ttyACM0'
inputSerial = serial.Serial(port_name, 9600)
time.sleep(5)

# Start the compass serial port
port_name = '/dev/ttyACM1'
compassSerial = serial.Serial(port_name, 9600)


# Compass thread
def compass_thread():
  global compass_value
  global compass_run_flag
  compass_run_flag = True
  while compass_run_flag == True:
    if compassSerial.inWaiting() > 0:	
      compass_data = compassSerial.readline().rstrip()
      #print ">>>", compass_data
      if compass_data[compass_data.find(',')+1:].isdigit():			
         compass_value = int(compass_data[compass_data.find(',')+1:])
	 compass_value = (1.0013*compass_value) - 1.3555
         #print ">>> ", compass_value
  
compass_thread = Thread(target=compass_thread)
compass_thread.start()

def get_compass():
  global compass_value
  return compass_value

# Set the Arduino to Drive Mode by sending control string
inputSerial.write("+++DRIVE+++")
inputSerial.write("\n")
inputSerial.flush()
time.sleep(5)

def set_pwr_and_steer(steer, power):
  inputSerial.write(str(steer) + "," + str(power))
  inputSerial.write('\n')
  inputSerial.flush()
  
def drive(speed, target_heading, time_in_seconds):
  start_time = time.time()
  print speed, target_heading, time_in_seconds

  while ((time.time() - start_time) < time_in_seconds):
    compass_value = get_compass()
    delta_angle = target_heading - compass_value
    if delta_angle > 180:
      delta_angle = delta_angle - 360
    elif delta_angle < -180:
      delta_angle = 360 + delta_angle
     
    steer_value = int((500.0/180.0) * delta_angle * steering_gain)
    
    if steer_value > 500:
      steer_value = 500
    elif steer_value < -500:
      steer_value = -500

    steer_value = 1500-steer_value
      
    print "Target:", target_heading, ",compass:", compass_value, ",delta:", delta_angle, ",speed:", speed, ",steer:", steer_value 
    set_pwr_and_steer(steer_value, speed)
    time.sleep(0.5)
 

# Main program starts here...
print "Go!!!"
drive(1700, 270, 9)
drive(1700,  180, 7)
drive(1700,   90, 7)
drive(1700, 0, 7)
drive(1500,   0, 1) 

compass_run_flag = False
time.sleep(5)
