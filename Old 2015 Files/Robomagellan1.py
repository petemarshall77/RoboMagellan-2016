#!/usr/bin/python

# Robomagellan 2015 Ver 1.0 
# 
#     
#     
#
#    
#
import serial
import signal
import time
import sys
import os.path
from threading import Thread

#Compass Variables
compass_value = 0
compass_run_flag = False

#GPS Variables
GPS_run_flag = False
GPS_Lat = 0
GPS_Long = 0

#Steering Power Variables
steering_gain = 2.5
	
#Port Definitions
steeringPower_port_name = '/dev/ttyACM0'
compass_port_name = '/dev/ttyACM1'
GPS_port_name = '/dev/ttyUSB0'	    

# Start the power/steering serial port (causes the Arduido to reset so
# wait a few seconds for that to happen)
print "Starting Power/Steering Communications"
inputSerial = serial.Serial(steeringPower_port_name, 9600)
time.sleep(5)

# Start the compass serial port
print "Starting Compass Communications"
compassSerial = serial.Serial(compass_port_name, 9600)

# Start the GPS serial port
print "Starting GPS Communications"
GPSSerial = serial.Serial(GPS_port_name, 4800)

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

print "Starting Compass Thread"  
compass_thread = Thread(target=compass_thread)
compass_thread.start()

def get_compass():
  global compass_value
  return compass_value

# GPS thread
def GPS_thread():
  global GPS_Lat
  global GPS_Long
  global GPS_run_flag
  GPS_run_flag = True
  while GPS_run_flag == True:
    if GPSSerial.inWaiting() > 0:	
      GPS_data = GPSSerial.readline().rstrip()
      print ">>>", GPS_data
      
  
print "Starting GPS Thread"
GPS_thread = Thread(target=GPS_thread)
GPS_thread.start()

def get_GPS():
  global GPS_Lat
  global GPS_Long
  return (GPS_Lat,GPS_Long)

# Set the Arduino to Drive Mode by sending control string
print "Set to DRIVE mode"
inputSerial.write("+++DRIVE+++")
inputSerial.write("\n")
inputSerial.flush()
time.sleep(5)

#Send Power and Steering Data
def set_pwr_and_steer(steer, power):
  inputSerial.write(str(steer) + "," + str(power))
  inputSerial.write('\n')
  inputSerial.flush()
  
#Drive At specified Speed and Target Heading for Specified Time
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
 
'''
# Main program starts here...
print "Go!!!"
drive(1700, 270, 9)
drive(1700,  180, 7)
drive(1700,   90, 7)
drive(1700, 0, 7)
drive(1500,   0, 1) 
'''
time.sleep(60)
#Terminate Threads
compass_run_flag = False
GPS_run_flag = False
time.sleep(5)
print 'Done!!!'
