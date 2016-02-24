\from serial import Serial

class PowerSteering:

    def __init__(self, port_name, baud_rate, logger):
        self.logger = logger
        self.logger.write("Starting Power/Steering Communications")
        self.serial = Serial(port_name, baud_rate)
        self.current_power = 1500
        self.current_speed = 0

    def set_mode_drive(self):
        self.logger.write("Set to DRIVE mode")
        self.serial.write("+++DRIVE+++")
        self.serial.write("\n")

    def set_mode_stop(self):
        self.logger.write("Set to STOPPED mode")
        self.serial.write("+++STOP+++")
        self.serial.write("\n")
        self.serial.flush()

    def set_pwr_and_steer(self, steer_value, target_speed, new_speed):
      if steer_value > 500:
        steer_value = 500
      elif steer_value < -500:
        steer_value = -500

      steer_value = 1500-steer_value

      # Calculate the power value
      delta_speed = target_speed - new_speed
      acceleration = new_speed - self.current_speed
      delta_power = delta_speed * 0.1 - acceleration * 0.0
      self.current_power = self.current_power + delta_power
      self.logger.write("Power steer- target: %.2f, current: %.2f, power: %d" % (target_speed, new_speed, self.current_power))
      self.current_speed = new_speed
      if self.current_power > 1650:
          self.current_power = 1500

      self.serial.write(str(steer_value) + "," + str(int(self.current_power)))
      self.serial.write('\n')
      self.serial.flush()
