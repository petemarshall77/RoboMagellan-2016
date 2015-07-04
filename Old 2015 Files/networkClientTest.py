import time
from threading import Thread
from socket import socket, AF_INET, SOCK_STREAM

camera_values = ""
camera_run_flag = False

def camera_thread():
    global camera_values
    global camera_run_flag

    print "Camera thread started"
    camera_run_flag = True
    
    s = socket(AF_INET, SOCK_STREAM)
    s.connect(('192.168.0.200', 9788))
    while camera_run_flag == True:
        camera_data = s.recv(8192)
        camera_fields = camera_data.split(" ")
        camera_values = (int(camera_fields[0]),int(camera_fields[1]))


    s.close()
    print "Camera thread terminated"

def get_camera_values():
    global camera_values
    return camera_values

camera_thread = Thread(target=camera_thread)
camera_thread.start()

try:
  while True:
      print get_camera_values()
      time.sleep(0.5)

except KeyboardInterrupt:
  pass

camera_run_flag = False
time.sleep(3)
