
#Authors:
#PJ Solomon November 27 2017
#Joseph Zieg November 29 2017

#Assumes Steering = Motor1, Drive = Motor4


from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from gpiozero import DistanceSensor
import RPi.GPIO as GPIO
import time
import atexit

GPIO.setmode(GPIO.BCM)

class car(object):
# Controls for the rover. Uses input from sensors to determine optimal course

    def __init__(self, addr=0x60, steering_id=1, drive_id=4):
        # Initialize HAT and motors
        self.mh = Adafruit_MotorHAT(addr)
        self.steering = self.mh.getMotor(steering_id)
        self.motor = self.mh.getMotor(drive_id)

        self.MAX_SPEED = 255

        # Initialize sensors
        self.us1 = sensor(19, 16, .3) # Makes decision at 30 cm
        #self.us2 = sensor(19, 20, .3) #Re-enable when more sensors needed

    def turnOffMotors(self):
        self.steering.run(Adafruit_MotorHAT.RELEASE)
        self.motor.run(Adafruit_MotorHAT.RELEASE)

    def drive(self):
        for i in range(100):  # Runs for 10 seconds
            if self.us1.collisonWarning():
                self.right(self.MAX_SPEED);
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

    def test(self):
        # set the speed to start, from 0 (off) to 255 (max speed)
        self.forward(self.MAX_SPEED)

        # Adjust these to use functions eventually
        self.steering.setSpeed(0)
        self.steering.run(Adafruit_MotorHAT.FORWARD)

        time.sleep(10)

        # turn off motor
        self.motor.run(Adafruit_MotorHAT.RELEASE)
        self.steering.run(Adafruit_MotorHAT.RELEASE)

    def right(self, speed):
        self.steering.setSpeed(speed);
        self.motor.setSpeed(speed);
        self.steering.run(Adafruit_MotorHAT.FORWARD)
        self.motor.run(Adafruit_MotorHAT.FORWARD)


class sensor(object):

    def __init__(self, echo, trigger, min_distance=.3):
        self.MIN_DISTANCE = min_distance * 100
        # set GPIO Pins
        self.GPIO_TRIGGER = trigger
        self.GPIO_ECHO = echo

        # set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def collisonWarning(self):
        if self.getDistance() <= self.MIN_DISTANCE:
            return True
        return False

    def getDistance(self):
        # Returns the distance in cm given by the sensor

        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        startTime = time.time()
        stopTime = time.time()

        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            startTime = time.time()

        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            stopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = stopTime - startTime

        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        return distance

    def distanceTest(self):
        # Tests the sensor input, outputs to console
        try:
            while True:
                dist = self.getDistance()
                print ("Measured Distance = %.1f cm" % dist)
                time.sleep(1)

                # Reset by pressing CTRL + C
        except KeyboardInterrupt:
            print("Measurement stopped by User")
            GPIO.cleanup()

car = car()

car.test()
