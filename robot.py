from utils import *
import time

class Robot:

    def __init__(self, powersteering, compass, gps, camera, logger):
        self.proportional_steering_gain = 2.5
        self.differential_steering_gain = 0.2
        self.camera_speed = 1550

        self.logger = logger
        self.powersteering = powersteering
        self.compass = compass
        self.gps = gps
        self.camera = camera
        self.run_start_time = time.time()

    def drive_to_waypoint(self, speed, latitude, longitude):
        self.logger.write("drive_to: %s, %s, %s" % (speed, latitude, longitude))

        (currentLat, currentLong) = self.gps.get_GPS()
        (currentDistance, currentBearing) = Utils.get_distance_and_bearing(currentLat, currentLong, latitude, longitude)
        delta_angle_previous = 0

        while not self.in_camera_regime(currentDistance, self.camera.get_camera_values()[1]): #TODO: add camera class + this method
            compass_value = self.compass.get_compass()
            delta_angle = currentBearing - compass_value
            delta_angle = self.reduce_delta_angle(delta_angle)

            steer_value = self.calculate_steer_value(delta_angle, delta_angle_previous)

            delta_angle_previous = delta_angle

            self.logger.write("TIME: %s, LAT: %s, LON: %s, BEARING: %s, COMPASS: %s, DELTA: %s, DIST: %s, SPEED %s, STEER %s" \
                % (time.time() - self.run_start_time, currentLat, currentLong, currentBearing, compass_value, delta_angle, \
                currentDistance, speed, steer_value))
            self.powersteering.set_pwr_and_steer(steer_value, speed)

            (currentLat,currentLong) = self.gps.get_GPS()
            (currentDistance,currentBearing) = Utils.get_distance_and_bearing(currentLat, currentLong, latitude, longitude)

    def drive_to_cone(self, speed, latitude, longitude):
        found_it = False
        while found_it == False:
            self.drive_to_waypoint(speed, latitude, longitude)
            (currentLat, currentLong) = self.gps.get_GPS()
            (currentDistance,currentBearing) = Utils.get_distance_and_bearing(currentLat, currentLong, latitude, longitude)

            while self.in_camera_regime(currentDistance, self.camera.get_camera_values()[1]):
                camera_value = self.camera.get_camera_values()[0]
                if camera_value == 0:
                    self.logger.write("Camera Mode - No data")
                    time.sleep(0.2)
                    steer_value = 0
                else:
                    steer_value = int((camera_value * (500.0/320.0))-500)
                    self.logger.write("Camera Mode - DISTANCE: %s CAMERA-VALUE: %s, STEER: %s" % \
                        (currentDistance, camera_value, steer_value))

                self.powersteering.set_pwr_and_steer(steer_value, self.camera_speed)
                if self.compass.get_bump_switch_state() == True:
                    found_it = True
                    self.logger.write("Found it!")
                    self.stop_driving()
                    time.sleep(1)
                    self.powersteering.set_pwr_and_steer(0, 1380)
                    time.sleep(3)
                    self.stop_driving()
                    break

                (currentLat, currentLong) = self.gps.get_GPS()
                (currentDistance, currentBearing) = Utils.get_distance_and_bearing(currentLat, currentLong, latitude, longitude)

    def drive_gps_only():
        counter = 0
        total_lat = 0
        total_long = 0
        while True:
            (latitude, longitude) = self.gps.get_GPS()
            total_lat += latitude
            total_long += longitude
            counter += 1
            print counter, total_lat/counter, total_long/counter
            time.sleep(1)
            if counter > 29:
                counter = 0
                total_lat = 0
                total_long = 0
                time.sleep(10)

    def drive(self, speed, target_heading, time_in_seconds):
        start_time = time.time()
        while ((time.time() - start_time) < time_in_seconds):
            compass_value = self.compass.get_compass()
            delta_angle = target_heading - compass_value
            delta_angle = self.reduce_delta_angle(delta_angle)
            steer_value = int((500.0/180.0) * delta_angle * self.proportional_steering_gain)
            self.powersteering.set_pwr_and_steer(steer_value, speed)

    def stop_driving(self):
      self.drive(1500, 0, 1)

    def in_camera_regime(self, currentDistance, pixelCount):
        self.logger.write("Camera Regime %d %d" % (currentDistance, pixelCount))
        self.logger.write(currentDistance < 10 and pixelCount > (currentDistance * -20.0/8.0 + 35)**2)
        return currentDistance < 10 and pixelCount > (currentDistance * -20.0/8.0 + 35)**2

    def calculate_steer_value(self, delta_angle, delta_angle_previous):
        delta_delta_angle = delta_angle_previous - delta_angle
        return int((500.0/180.0) * (delta_angle * self.proportional_steering_gain - delta_delta_angle * self.differential_steering_gain))

    def reduce_delta_angle(self, delta_angle):
        if delta_angle > 180:
            return delta_angle - 360
        elif delta_angle < -180:
            return 360 + delta_angle
        return delta_angle
