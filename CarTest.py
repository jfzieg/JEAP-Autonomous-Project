# Authors:
# PJ Solomon November 27 2017
# Joseph Zieg November 29 2017
#
# Assumes Steering = Motor1, Drive = Motor4
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import RPi.GPIO as GPIO
import time

# Globals for direction
RIGHT = 1
LEFT = 2
STRAIGHT = 3


class Car(object):
    # Controls for the rover. Uses input from sensors to determine optimal course

    def __init__(self, addr=0x60, steering_id=1, drive_id=4):
        # Initialize HAT and motors
        self.mh = Adafruit_MotorHAT(addr)
        self.steering = self.mh.getMotor(steering_id)
        self.motor = self.mh.getMotor(drive_id)
        self.MAX_SPEED = 255

        # Initialize sensors
        GPIO.setmode(GPIO.BCM)
        self.us1 = Sensor(6, 12)  # Defaults to decision made at 30 cm
        #self.us2 = Sensor(19, 20, .3)  # Re-enable when more sensors needed

    def drive(self):
        for i in range(100):  # Runs for 10 seconds
            if self.us1.collisonWarning():
                self.turn(self.MAX_SPEED, RIGHT)
            else:
                self.steering.setSpeed(0)
                self.forward(self.MAX_SPEED)
            time.sleep(.1)

    def forward(self, speed):
        self.motor.setSpeed(speed)
        self.motor.run(Adafruit_MotorHAT.FORWARD)
        time.sleep(.01)

    def backward(self, speed):
        self.motor.reverse()
        self.motor.setSpeed(speed)
        self.motor.run(Adafruit_MotorHAT.BACKWARD)

    def turn(self, speed, direction):
        if direction is RIGHT:
            self.steering.setSpeed(speed)
            self.motor.setSpeed(speed)
            self.steering.run(Adafruit_MotorHAT.FORWARD)
            self.motor.run(Adafruit_MotorHAT.FORWARD)
        elif direction is LEFT:
            self.steering.reverse()
            self.steering.setSpeed(speed)
            self.motor.setSpeed(speed)
            self.steering.run(Adafruit_MotorHAT.FORWARD)
            self.motor.run(Adafruit_MotorHAT.FORWARD)

    def directionDecision(self):
        # Update once turning is figured out
        return None

    def turnOff(self):
        self.steering.run(Adafruit_MotorHAT.RELEASE)
        self.motor.run(Adafruit_MotorHAT.RELEASE)
        GPIO.cleanup()

    def test(self):
        # Test sensor output
        self.us1.distanceTest()

        # set the speed to start, from 0 (off) to 255 (max speed)
        self.forward(self.MAX_SPEED)

        # Adjust these to use functions eventually
        self.steering.setSpeed(0)
        self.steering.run(Adafruit_MotorHAT.FORWARD)

        time.sleep(10)

        # turn off motor
        self.turnOff()


class Sensor(object):
    # Sensor object for use with Car class
    # GPIO code structure for getDistance() function taken from:
    # https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/

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
        for i in range(100):
            dist = self.getDistance()
            print ("Measured Distance = %.1f cm" % dist)
            if self.collisonWarning():
                print "!!Min Distance Reached!!"
            time.sleep(.1)


####################
# CAR RUNNING CODE #
####################
car = Car()

car.us1.getDistance()