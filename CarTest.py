# Authors:
# PJ Solomon November 27 2017
# Joseph Zieg November 29 2017
#
from Adafruit_MotorHAT import Adafruit_MotorHAT as HAT
import RPi.GPIO as GPIO
import time

# Globals for direction control
RIGHT = 0
STRAIGHT = 1
LEFT = 2

#############
# CAR CLASS #
#############
class Car(object):
    # Controls for the rover. Uses input from sensors to determine optimal course
    # Assumes Steering = Motor1, Drive = Motor4
    # Uses 3 HR-S04 ultrasonic sensors for obstacle detection

    def __init__(self, addr=0x60, steering_id=1, drive_id=4):
        # Initialize hat and motors
        self.mh = HAT(addr)
        self.steering = self.mh.getMotor(steering_id)
        self.motor = self.mh.getMotor(drive_id)
        self.MAX_SPEED = 255

        # Initialize sensors
        # Defaults to decision made at 30 cm
        GPIO.setmode(GPIO.BCM)
        self.usm = Sensor(13, 12) # Middle sensor
        self.usr = Sensor(6, 5)   # Right side sensor
        self.usl = Sensor(7, 8)   # Left side sensor
        self.usTriggered = None

        self.sensors = [self.usl, self.usm, self.usr]

    def drive(self):
        # Decision should update frequently, currently 1/100th of a sec
        # Testing should see if this is enough
        for i in range(1000):  # Runs for 10 seconds
            if self.usm.collisonWarning():
                self.turn(self.MAX_SPEED / 2)
            else:

                self.forward(self.MAX_SPEED)
            time.sleep(.01)

    def forward(self, speed):
        self.steering.setSpeed(0)
        self.steering.run(HAT.FORWARD)

        self.motor.setSpeed(speed)
        self.motor.run(HAT.FORWARD)

    def backward(self, speed):
        self.steering.setSpeed(0)
        self.steering.run(HAT.FORWARD)

        self.motor.reverse()
        self.motor.setSpeed(speed)
        self.motor.run(HAT.BACKWARD)

    def right(self, speed):
        # idk if these are the correct directions
        self.steering.reverse()
        self.steering.setSpeed(self.MAX_SPEED)
        self.steering.run(HAT.FORWARD)

        self.motor.setSpeed(speed)
        self.motor.run(HAT.FORWARD)

    def left(self, speed):
        # idk if these are the correct directions
        self.steering.setSpeed(self.MAX_SPEED)
        self.steering.run(HAT.FORWARD)

        self.motor.setSpeed(speed)
        self.motor.run(HAT.FORWARD)

    def turn(self, speed):
        if self.usTriggered == STRAIGHT:
            direction = self.directionDecision()
            if direction is RIGHT:
                self.left(speed)
            elif direction is LEFT:
                self.right(speed)

        elif self.usTriggered == LEFT:
            self.right(speed)

        elif self.usTriggered == RIGHT:
            self.left(speed)

    def directionDecision(self):
        if self.sensors[0].getDistance() < self.sensors[2].getDistance():
            return RIGHT
        else:
            return LEFT

    def CollisionWarning(self):
        # Iterates over the sensor array, determines if one sensor is triggered
        # returns True if an object is within minimum safe distance for any sensor
        for us in self.sensors:
            if us.collisonWarning():
                self.usTriggered = self.sensors.index(us)
                return True
        return False

    def turnOff(self):
        # Stop motors and clean up GPIO pins
        self.steering.run(HAT.RELEASE)
        self.motor.run(HAT.RELEASE)
        GPIO.cleanup()

    def test(self):
        # Test sensor output
        print("Testing Middle")
        self.usm.distanceTest()

        print("Testing Left")
        self.usl.distanceTest()

        print("Testing Right")
        self.usr.distanceTest()

        # Test obstacle detection and motor response
        self.drive()

        # turn off motor
        self.turnOff()

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
                print "!!Min Distance Reached!!"
            time.sleep(.1)

    def __iter__(self):
        return self.getDistance()


####################
# CAR RUNNING CODE #
####################
try:
    car = Car()

    car.test()
except KeyboardInterrupt:
    car.turnOff()
    print("Program closed")
