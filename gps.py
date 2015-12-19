from serial import Serial
import threading
import time

class GPS:

    def __init__(self, port_name, baud_rate, logger):
        self.logger = logger
        self.logger.write("Starting GPS Communications")
        self.serial = Serial(port_name, baud_rate)

        self.lat = 0
        self.long = 0

    def get_GPS(self):
        while (self.lat == 0.0):
            self.logger.write("Waiting for GPS")
            time.sleep(1)
        return (self.lat, self.long)

    def read_data(self):
        if self.serial.inWaiting() > 0:
            data = self.serial.readline().rstrip()
            if data[0:6] == "$GPRMC":
                fields = data.split(',')
                if fields[2] == 'A':
                  self.lat = int(fields[3][0:2]) + (float(fields[3][2:]) / 60.0) #ASSUMES TWO DIGIT LATITUDE
                  self.long = int(fields[5][0:3]) + (float(fields[5][3:]) / 60.0) #ASSUMES THREE DIGIT LONGITUDE
                else:
                  self.lat = 0.0
                  self.long = 0.0

    def close(self):
        self.serial.close()

class GPSThread(threading.Thread):

    def __init__(self, gps, logger):
        super(GPSThread, self).__init__()
        self.stoprequest = threading.Event()
        self.gps = gps
        self.logger = logger

    def run(self):
        self.logger.write("Starting GPS thread")
        while not self.stoprequest.isSet():
            self.gps.read_data()
        self.logger.write("Terminating GPS thread")
        self.gps.close()

    def join(self, timeout=None):
        self.stoprequest.set()
        super(GPSThread, self).join(timeout)
