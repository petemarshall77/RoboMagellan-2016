#!/usr/bin/python

# Robomagellan 2016
#

# Import all the things
import math
import serial
import signal
import time
from socket import socket, AF_INET, SOCK_STREAM
import sys
import os.path
from threading import Thread
from logger import Logger
from robotcontroller import RobotController

run_start_time = time.time()


#Compass Variables
compass_value = 0
compass_run_flag = False

bump_switch_state = 0

#GPS Variables
GPS_run_flag = False
GPS_Lat = 0
GPS_Long = 0

#Camera Variables
camera_values = ""
camera_run_flag = False

#Steering Power Variables
proportional_steering_gain = 2.5
differential_steering_gain = 0.2
camera_speed = 1550
GPS_speed = 1560
        
#Port Definitions
steeringPower_port_name = '/dev/ttyACM0'
compass_port_name = '/dev/ttyACM1'
GPS_port_name = '/dev/ttyUSB0'

#Terminate The Program
def terminate():
  global compass_run_flag
  global camera_run_flag
  global GPS_run_flag
  datalog.write('Terminate Called')
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
      if len(compass_data) == 3:
        if compass_data[1].isdigit():                     
          compass_value = int(compass_data[1])
          compass_value = (1.10905*compass_value) + 1.8176 #(1.0013*compass_value) - 1.3555
        if compass_data[2] == "1":
          bump_switch_state = True
        else:
          bump_switch_state = False
  datalog.write('Compass Thread Terminating')
  compassSerial.close()

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

    datalog.write("Camera thread started")
    camera_run_flag = True
    
    s = socket(AF_INET, SOCK_STREAM)
    s.connect(('localhost', 9788))
    while camera_run_flag == True:
        camera_data = s.recv(8192)
        camera_fields = camera_data.split(" ")
        try:
            camera_values = (int(camera_fields[0]),int(camera_fields[1]))
        except:
            datalog.write("bad camera data", camera_data)

    s.close()
    datalog.write("Camera thread terminated")

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
  datalog.write('GPS Thread Terminating')
  GPSSerial.close()
  


def get_GPS():
  global GPS_Lat
  global GPS_Long
  while (GPS_Lat==0.0):
    datalog.write("Waiting for GPS")
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


#Send Power and Steering Data
def set_pwr_and_steer(steer_value, power):
  # steer_value = steer_value + 100
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

  while ((time.time() - start_time) < time_in_seconds):
    compass_value = get_compass()
    delta_angle = target_heading - compass_value
    if delta_angle > 180:
      delta_angle = delta_angle - 360
    elif delta_angle < -180:
      delta_angle = 360 + delta_angle
     
    steer_value = int((500.0/180.0) * delta_angle * proportional_steering_gain)
      
    set_pwr_and_steer(steer_value, speed)

def stop_driving():
  drive(1500, 0, 1)

#Determine if robot is in camera regime
def in_camera_regime(currentDistance, pixelCount):
    datalog.write("Camera Regime %d %d" %(currentDistance, pixelCount))
    datalog.write(currentDistance < 10 and pixelCount > (currentDistance * -20.0/8.0 + 35)**2)
    return currentDistance < 10 and pixelCount > (currentDistance * -20.0/8.0 + 35)**2

#Drive to specified GPS way point. 
def drive_to(speed, Latitude, Longitude):
  datalog.write("drive_to: %s, %s, %s" % (speed, Latitude, Longitude))

  (currentLat,currentLong)=get_GPS()
  (currentDistance,currentBearing)=get_distance_and_bearing(currentLat,currentLong,Latitude,Longitude)
  delta_angle_previous = 0
  while not in_camera_regime(currentDistance, get_camera_values()[1]):
    compass_value = get_compass()
    delta_angle = currentBearing - compass_value
    if delta_angle > 180:
      delta_angle = delta_angle - 360
    elif delta_angle < -180:
      delta_angle = 360 + delta_angle
    
    delta_delta_angle = delta_angle_previous - delta_angle
    delta_angle_previous = delta_angle
     
    steer_value = int((500.0/180.0) * (delta_angle * proportional_steering_gain - delta_delta_angle * differential_steering_gain)) 
      
    datalog.write("TIME: %s, LAT: %s, LON: %s, BEARING: %s, COMPASS: %s, DELTA: %s, DIST: %s, SPEED %s, STEER %s" % (time.time() - run_start_time, currentLat, currentLong, currentBearing, compass_value, delta_angle, currentDistance, speed, steer_value))
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
     while in_camera_regime(currentDistance, get_camera_values()[1]):

        camera_value = get_camera_values()[0]
        if camera_value == 0:
          datalog.write("Camera Mode - No data")
          # Hunting code: steer from side to side (currently disabled)
          time.sleep(0.2)
          if time.time() % 15 < 15:
            steer_value = 0
          else:
            steer_value = 0
        else:
          steer_value = int((camera_value * (500.0/320.0))-500) 
          datalog.write("Camera Mode - DISTANCE: %s CAMERA-VALUE: %s, STEER: %s" % (currentDistance, camera_value, steer_value))

        set_pwr_and_steer(steer_value,camera_speed)
        if get_bump_switch_state() == True:
          found_it = True
          datalog.write("Found it!")
          stop_driving()
          time.sleep(1)
          set_pwr_and_steer(0, 1380)
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

def dec6course():
  drive_to_cone(1600, 33.77868, 118.41895166666667)
  drive_to_cone(1600, 33.778513333333336, 118.41902)
  

#========================================================
#Main program starts here
#========================================================
def main():
    # Start logging
    datalog = Logger()

    # Start the power/steering serial port (causes the Arduido to reset so
    # wait a few seconds for that to happen)
    robot = RobotController(steeringPower_port_name, 9600, datalog)
    robot.start()
    time.sleep(5)

    # Start the compass serial port
    datalog.write("Starting Compass Communications")
    compassSerial = serial.Serial(compass_port_name, 9600)

    # Start the GPS serial port
    datalog.write("Starting GPS Communications")
    GPSSerial = serial.Serial(GPS_port_name, 4800)


    # Set the Arduino to Drive Mode by sending control string
    robot.drive()
    time.sleep(5)

    datalog.write("Starting Compass Thread")
    compass_thread = Thread(target=compass_thread)
    compass_thread.start()

    datalog.write("Starting GPS Thread")
    GPS_thread = Thread(target=GPS_thread)
    GPS_thread.start()

    datalog.write("Starting Camera Thread")
    camera_thread = Thread(target=camera_thread)
    camera_thread.start()

    try:
        get_GPS()
        datalog.write("Armed and Ready - Press Button")
        while get_bump_switch_state() == False:
          pass
        datalog.write("Go!!!")
        #drive_gps_only()
        dec6course()

    except KeyboardInterrupt:
        pass

    #========================================================
    #Main program stops here
    #========================================================
    stop_driving()
    robot.stop()

    terminate()
    datalog.write('Done!!!')
    del datalog
    

if __name__ == "__main__":
    main()
