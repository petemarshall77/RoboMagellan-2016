#!/usr/bin/python

# Robomagellan 2015 Ver 1.0 
# 
#     
#     
#
#    
#
import math
import serial
import signal
import time
from socket import socket, AF_INET, SOCK_STREAM
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

#Camera Variables
camera_values = ""
camera_run_flag = False

#Steering Power Variables
steering_gain = 2.5
sum_steering_gain = 0.2
camera_speed = 1575
GPS_speed = 1600
        
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

#Terminate The Program
def terminate():
  global compass_run_flag
  global camera_run_flag
  global GPS_run_flag
  print 'Terminate Called'
  compass_run_flag = False
  GPS_run_flag = False
  camera_run_flag = False
  time.sleep(5)

# Compass thread
def compass_thread():
  global compass_value
  global compass_run_flag
  global bump_switch_state
  compass_run_flag = True
  while compass_run_flag == True:
    if compassSerial.inWaiting() > 0:   
      compass_data = compassSerial.readline().rstrip().split(',')
      #print ">>>", compass_data
      if len(compass_data) == 3:
        if compass_data[1].isdigit():                     
          compass_value = int(compass_data[1])
          compass_value = (1.0013*compass_value) - 1.3555
          #print ">>> ", compass_value
        if compass_data[2] == "1":
          bump_switch_state = True
        else:
          bump_switch_state = False
        #print ">>>", bump_switch_state
  print 'Compass Thread Terminating'

def get_bump_switch_state():
  global bump_switch_state
  return bump_switch_state

def get_compass():
  global compass_value
  return compass_value

# Camera thread
def camera_thread():
    global camera_values
    global camera_run_flag

    print "Camera thread started"
    camera_run_flag = True
    
    s = socket(AF_INET, SOCK_STREAM)
    s.connect(('localhost', 9788))
    while camera_run_flag == True:
        camera_data = s.recv(8192)
        camera_fields = camera_data.split(" ")
        camera_values = (int(camera_fields[0]),int(camera_fields[1]))

    s.close()
    print "Camera thread terminated"

def get_camera_values():
    global camera_values
    return camera_values




# GPS thread
def GPS_thread():
  global GPS_Lat
  global GPS_Long
  global GPS_run_flag
  GPS_run_flag = True
  while GPS_run_flag == True:
    if GPSSerial.inWaiting() > 0:       
      GPS_data = GPSSerial.readline().rstrip()
      if GPS_data[0:6]=="$GPRMC":
        GPS_fields=GPS_data.split(',')
        if GPS_fields[2]=='A':
          GPS_Lat=int(GPS_fields[3][0:2])+(float(GPS_fields[3][2:])/60.0) #ASSUMES TWO DIGIT LATITUDE 
          GPS_Long=int(GPS_fields[5][0:3])+(float(GPS_fields[5][3:])/60.0) #ASSUMES THREE DIGIT LONGITUDE
        else:
          GPS_Lat=0.0
          GPS_Long=0.0 
  print 'GPS Thread Terminating'
  


def get_GPS():
  global GPS_Lat
  global GPS_Long
  while (GPS_Lat==0.0):
    print "Waiting for GPS"
    time.sleep(1)
  return (GPS_Lat,GPS_Long)

# Haversine Thread
def degrees_to_radians(angle):
  return angle*math.pi/180.0

def radians_to_degrees(radians):
  return radians*180/math.pi

def get_distance_and_bearing(from_lat,from_long,to_lat,to_long):
  from_lat=degrees_to_radians(from_lat)
  from_long=degrees_to_radians(from_long)
  to_lat=degrees_to_radians(to_lat)
  to_long=degrees_to_radians(to_long)
  delta_lat=to_lat-from_lat
  delta_long=to_long-from_long
        
  a=math.sin(delta_lat/2)*math.sin(delta_lat/2)+math.cos(from_lat)*math.cos(to_lat)*math.sin(delta_long/2)*math.sin(delta_long/2)
  c=2*(math.atan2(math.sqrt(a),math.sqrt(1-a)))
  distance=6371000*c
  
  y=math.sin(from_long-to_long)*math.cos(to_lat)
  x=math.cos(from_lat)*math.sin(to_lat)-math.sin(from_lat)*math.cos(to_lat)*math.cos(from_long-to_long)
  bearing=(radians_to_degrees(math.atan2(y,x))+360)%360
  
  return (distance,bearing)

# Set the Arduino to Drive Mode by sending control string
print "Set to DRIVE mode"
inputSerial.write("+++DRIVE+++")
inputSerial.write("\n")
#inputSerial.flush()
time.sleep(5)

#Send Power and Steering Data
def set_pwr_and_steer(steer_value, power):
  if steer_value > 500:
    steer_value = 500
  elif steer_value < -500:
    steer_value = -500

  steer_value = 1500-steer_value

  inputSerial.write(str(steer_value) + "," + str(power))
  inputSerial.write('\n')
  inputSerial.flush()
  
#Drive At specified Speed and Target Heading for Specified Time
def drive(speed, target_heading, time_in_seconds):
  start_time = time.time()
  #print speed, target_heading, time_in_seconds

  while ((time.time() - start_time) < time_in_seconds):
    compass_value = get_compass()
    delta_angle = target_heading - compass_value
    if delta_angle > 180:
      delta_angle = delta_angle - 360
    elif delta_angle < -180:
      delta_angle = 360 + delta_angle
     
    steer_value = int((500.0/180.0) * delta_angle * steering_gain)
      
    #print "Target:", target_heading, ",compass:", compass_value, ",delta:", delta_angle, ",speed:", speed, ",steer:", steer_value
    set_pwr_and_steer(steer_value, speed)
    #time.sleep(0.5)

def stop_driving():
  drive(1500, 0, 1)

#Drive to specified GPS way point. 
def drive_to(speed, Latitude, Longitude):
  print "drive_to:",speed, Latitude, Longitude

  (currentLat,currentLong)=get_GPS()
  (currentDistance,currentBearing)=get_distance_and_bearing(currentLat,currentLong,Latitude,Longitude)
  sum_delta_angle = 0
  delta_angle_previous = 0
  while (currentDistance > 3.5):
    compass_value = get_compass()
    delta_angle = currentBearing - compass_value
    sum_delta_angle += delta_angle
    if delta_angle > 180:
      delta_angle = delta_angle - 360
    elif delta_angle < -180:
      delta_angle = 360 + delta_angle
    if delta_angle_previous > 0 and delta_angle < 0:
      sum_delta_angle = 0
    if delta_angle_previous < 0 and delta_angle > 0:
      sum_delta_angle = 0
    delta_angle_previous = delta_angle
     
    steer_value = int((500.0/180.0) * delta_angle * steering_gain + sum_delta_angle * sum_steering_gain)
      
    print "Target:", currentBearing, ", compass:", compass_value, ", delta:", delta_angle, ", distance:", currentDistance, ", speed:", speed, ", steer:", steer_value 
    set_pwr_and_steer(steer_value, speed)
    #time.sleep(0.5)

    (currentLat,currentLong)=get_GPS()
    (currentDistance,currentBearing)=get_distance_and_bearing(currentLat,currentLong,Latitude,Longitude)

def drive_to_cone(speed, Latitude, Longitude):
    found_it = False
    while found_it == False:
     drive_to(speed, Latitude, Longitude)
     (currentLat,currentLong)=get_GPS()
     (currentDistance,currentBearing)=get_distance_and_bearing(currentLat,currentLong,Latitude,Longitude)
     while currentDistance < 5:
        
        print "Camera Mode"
        if get_camera_values()[0] == 0:
          steer_value = 0
        else:
          steer_value = ((get_camera_values()[0] * (500/320))-500) 
        print(str(steer_value))
        set_pwr_and_steer(steer_value,camera_speed)
        if get_bump_switch_state() == True:
	  found_it = True
          stop_driving()
          time.sleep(1)
          print("Sleep!!")
          set_pwr_and_steer(0, 1400)
          time.sleep(3)
          stop_driving()
          break
        (currentLat,currentLong)=get_GPS()
        (currentDistance,currentBearing)=get_distance_and_bearing(currentLat, currentLong, Latitude ,Longitude)
#========================================================
# Driving Instructions
#========================================================
def drive_gps_only():
  counter = 0
  total_lat = 0
  total_long = 0
  while True:
    (Lat, Long) = get_GPS()
    total_lat += Lat
    total_long += Long
    counter += 1
    print counter, total_lat/counter, total_long/counter
    time.sleep(1)
    if counter > 29:
      counter = 0
      total_lat = 0
      total_long = 0
      time.sleep(10)
    
def amateur_square():
  drive(1625, 270, 9)
  drive(1625,  180, 7)
  drive(1625,   90, 7)
  drive(1625, 0, 7)
  stop_driving()

def gps_square_on_grass():
  drive_to(1625, 33.7783936111, 118.419133944)
  drive_to(1625, 33.7781982778, 118.419198722)
  drive_to(1625, 33.778054, 118.418784222)
  drive_to(1625, 33.7782284444, 118.418671889)
  stop_driving()

def gps_zigzag_on_grass():
  #drive_to_cone(1600, 33.778188444, 118.419276833)
  #stop_driving()
  #time.sleep(5)
  drive_to_cone(1590, 33.7783936111, 118.419133944)
  stop_driving()
  time.sleep(5)
  #drive_to_cone(1600, 33.7782412778, 118.4189595)
  #stop_driving()
  #time.sleep(5)
  drive_to_cone(1590, 33.778316666, 118.418795) #point next to door
  stop_driving()

#========================================================
#Main program starts here
#========================================================
print "Starting Compass Thread"  
compass_thread = Thread(target=compass_thread)
compass_thread.start()

print "Starting GPS Thread"
GPS_thread = Thread(target=GPS_thread)
GPS_thread.start()

print "Starting Camera Thread"
camera_thread = Thread(target=camera_thread)
camera_thread.start()

try:
  get_GPS()
  print "Go!!!"
  #drive_gps_only()
  gps_zigzag_on_grass()

except KeyboardInterrupt:
  pass

#========================================================
#Main program stops here
#========================================================
stop_driving()
print "Set to STOPPED mode"
inputSerial.write("+++STOP+++")
inputSerial.write("\n")
inputSerial.flush()

terminate()
print 'Done!!!'

