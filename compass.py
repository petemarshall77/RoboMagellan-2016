from serial import Serial
import threading

class Compass:

    def __init__(self, port_name, baud_rate, logger):
        self.logger = logger
        self.logger.write("Starting Compass Communications")
        self.serial = Serial(port_name, baud_rate)

        self.value = 0
        self.bump_switch_state = 0

    def get_compass(self):
        return self.value

    def get_bump_switch_state(self):
        return self.bump_switch_state

    def read_data(self):
        if self.serial.inWaiting() > 0:
            compass_data = self.serial.readline().rstrip().split(',')
            if len(compass_data) == 3:
                if compass_data[1].isdigit():
                    self.value = (1.10905 * int(compass_data[1])) + 1.8176
                self.bump_switch_state = True if compass_data[2] == "1" else False

    def close(self):
        self.serial.close()

class CompassThread(threading.Thread):

    def __init__(self, compass, logger):
        super(CompassThread, self).__init__()
        self.stoprequest = threading.Event()
        self.compass = compass
        self.logger = logger

    def run(self):
        self.logger.write("Starting compass thread")
        while not self.stoprequest.isSet():
            compass.read_data()
        self.logger.write("Terminating compass thread")
        self.compass.close()

    def join(self, timeout=None):
        self.stoprequest.set()
        super(CompassThread, self).join(timeout)
