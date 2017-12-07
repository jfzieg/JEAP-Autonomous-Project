import RPi.GPIO as GPIO
import time


################
# SENSOR CLASS #
################


class Sensor(object):
    # Sensor object for use with Car class
    # GPIO code structure for getDistance() function taken from:
    # https://www.modmypi.com/download/range_sensor.py

    def __init__(self, echo, trigger, min_distance=.3):
        self.MIN_DISTANCE = min_distance * 100
        # set GPIO Pins
        self.GPIO_TRIGGER = trigger
        self.GPIO_ECHO = echo

        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

        GPIO.output(self.GPIO_TRIGGER, False)
        time.sleep(2)

    def collisonWarning(self):
        if self.getDistance() <= self.MIN_DISTANCE:
            return True
        return False

    def getDistance(self):
        # Returns the distance in cm given by the sensor

        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        while GPIO.input(self.GPIO_ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(self.GPIO_ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)
        return distance

    def distanceTest(self):
        # Tests the sensor input, outputs to console for 10s
        for i in range(30):
            dist = self.getDistance()
            print ("Measured Distance = %.1f cm" % dist)
            if self.collisonWarning():
                print("!!Min Distance Reached!!")
            time.sleep(.1)
