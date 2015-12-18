from serial import Serial

class RobotController:

    def __init__(self, port_name, baud_rate, logger):
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.logger = logger

    def start(self):
        self.log("Starting Power/Steering Communications")
        self.serial = Serial(self.port_name, self.baud_rate)

    def drive(self):
        self.log("Set to DRIVE mode")
        self.serial.write("+++DRIVE+++")
        self.serial.write("\n")

    def stop(self):
        self.log("Set to STOPPED mode")
        self.serial.write("+++STOP+++")
        self.serial.write("\n")
        self.serial.flush()

    def log(self, message):
        self.logger.write(message)
