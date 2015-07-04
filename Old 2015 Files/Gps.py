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

#GPS Variables
GPS_run_flag = False
GPS_Lat = 0
GPS_Long = 0
	
#Port Definitions
GPS_port_name = '/dev/ttyUSB0'	    

# Start the GPS serial port
print "Starting GPS Communications"
GPSSerial = serial.Serial(GPS_port_name, 4800)

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
	  GPS_Lat=int(GPS_fields[3][0:2])
          print ">>>", GPS_Lat
        else:
	  GPS_Lat=0.0
	  GPS_Long=0.0
      
  
print "Starting GPS Thread"
GPS_thread = Thread(target=GPS_thread)
GPS_thread.start()

def get_GPS():
  global GPS_Lat
  global GPS_Long
  return (GPS_Lat,GPS_Long)


