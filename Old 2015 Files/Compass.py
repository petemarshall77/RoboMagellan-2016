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

def get_compass():
  global compass_value
  return compass_value

