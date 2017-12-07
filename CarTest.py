# Authors:
# PJ Solomon November 27 2017
# Joseph Zieg November 29 2017
#
from Adafruit_MotorHAT import Adafruit_MotorHAT as HAT
import SensorTest as s
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
        self.usm = s.Sensor(13, 12)  # Middle sensor
        self.usr = s.Sensor(6, 5)  # Right side sensor
        self.usl = s.Sensor(7, 8)  # Left side sensor
        self.usTriggered = None

        self.sensors = [self.usl, self.usm, self.usr]

    def drive(self, sec):
        # Decision should update frequently, currently 1/100th of a sec
        # Testing should see if this is enough
        # May want to add functionality to back up if dist to wall is ~0
        for i in range(sec * 1000):  # Runs for sec amount of seconds
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
        self.drive(30)

        # turn off motor
        self.turnOff()


####################
# CAR RUNNING CODE #
####################
try:
    car = Car()

    car.test()
except KeyboardInterrupt:
    car.turnOff()
    print("Program closed")
