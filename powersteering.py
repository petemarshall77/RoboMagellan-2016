from serial import Serial

class PowerSteering:

    def __init__(self, port_name, baud_rate, logger):
        self.logger = logger
        self.loggger.write("Starting Power/Steering Communications")
        self.serial = Serial(port_name, baud_rate)

    def set_mode_drive(self):
        self.logger.write("Set to DRIVE mode")
        self.serial.write("+++DRIVE+++")
        self.serial.write("\n")

    def set_mode_stop(self):
        self.logger.write("Set to STOPPED mode")
        self.serial.write("+++STOP+++")
        self.serial.write("\n")
        self.serial.flush()

    def set_pwr_and_steer(steer_value, power):
      if steer_value > 500:
        steer_value = 500
      elif steer_value < -500:
        steer_value = -500

      steer_value = 1500-steer_value

      self.serial.write(str(steer_value) + "," + str(power))
      self.serial.write('\n')
      self.serial.flush()
