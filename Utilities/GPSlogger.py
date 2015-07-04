# GPSLogger.py
# 
# Log GPS points for RoboMagellan
#
# Use: 1 - plug in GPS reciever
#      2 - go to desired GPS location
#      3 - hit 'r' to reset data collection
#      4 - wait for samples collected to get to 30
#      5 - hit 'c' to capture point
#      6 - repeat steps 2 thru 5 as necessary
#      7 - hit 'q' to quit
#      8 - cut and paste the output into the RoboMagellan program
#
import serial, time
import Tkinter as tk
from threading import Thread

# GPS Variables
GPS_run_flag = False
GPS_Lat = 0
GPS_Long = 0
GPS_port_name = '/dev/ttyUSB0'      

# Start the GPS serial port
print "Starting GPS Communications"
GPSSerial = serial.Serial(GPS_port_name, 4800)

# Data collection variables
max_sample = 30
sample_reset_flag = False
sample_write_flag = False
GPS_points = []

# GPS Thread
def GPS_thread():
  global GPS_Lat
  global GPS_Long
  global GPS_run_flag
  global sample_reset_flag
  global sample_write_flag
  global GPS_points
  GPS_run_flag = True
  sample_count = 0
  while GPS_run_flag == True:

    if sample_write_flag == True:
      GPS_points.append((GPS_Lat, GPS_Long))
      print 'Wrote (', GPS_Lat, ',', GPS_Long, ')' 
      sample_write_flag = False

    if sample_reset_flag == True:
      sample_count = 0
      sample_reset_flag = False

    if GPSSerial.inWaiting() > 0:       
      GPS_data = GPSSerial.readline().rstrip()
      if GPS_data[0:6]=="$GPRMC":
        GPS_fields=GPS_data.split(',')
        if GPS_fields[2]=='A':
          GPS_Lat=int(GPS_fields[3][0:2])+(float(GPS_fields[3][2:])/60.0) #ASSUMES TWO DIGIT LATITUDE 
          GPS_Long=int(GPS_fields[5][0:3])+(float(GPS_fields[5][3:])/60.0) #ASSUMES THREE DIGIT LONGITUDE
          sample_count += 1
        else:
          GPS_Lat=0.0
          GPS_Long=0.0
      print sample_count, '(', GPS_Lat, ', ', GPS_Long, ')' 
  print 'GPS Thread Terminating'

def get_GPS():
  global GPS_Lat
  global GPS_Long
  while (GPS_Lat==0.0):
    print "Waiting for GPS"
    time.sleep(1)
  return (GPS_Lat,GPS_Long)

def terminate_program():
  print "Terminating"
  global GPS_run_flag
  global GPS_points
  
  # Stop the GPS thread
  GPS_run_flag = False
  time.sleep(3)

  # Print the captured points
  print '>>>>'
  for point in GPS_points:
      print point
  print '<<<<'

# Start GPS Thread
print "Starting GPS Thread"
GPS_thread = Thread(target=GPS_thread)
GPS_thread.start()

# Process Tk keyboard events
def key(event):
    global sample_reset_flag
    global sample_write_flag
    if event.char == 'q':
        terminate_program()
        root.destroy()
    elif event.char == 'r':
        sample_reset_flag = True
    elif event.char == 'w':
        sample_write_flag = True
    else:
        print 'Unknown command', event.char

# Start Tk and Event Loop
root = tk.Tk()
print "Press a key ('q' to quit):"
root.bind_all('<Key>', key)
root.mainloop()

