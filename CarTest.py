
#Authors:
#PJ Solomon November 27 2017
#Joseph Zieg November 29 2017

#Assumes Steering = Motor1, Drive = Motor4


from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from gpiozero import DistanceSensor
import time
import atexit


class car(object):

 #   Controls for the rover. Uses input from sensors to

    def __init__(self, addr=0x60, steering_id=1, drive_id=4):
        # Initialize HAT and motors
        self.mh = Adafruit_MotorHAT(addr)
        self.steering = self.mh.getMotor(steering_id)
        self.motor = self.mh.getMotor(drive_id)

        self.MAX_SPEED = 255

        # Initialize sensors
        self.us1 = sensor(19, 16, 1, 0.3)
        #self.us2 = sensor(19, 20) #Re-enable when more sensors needed

    def turnOffMotors(self):
        self.steering.run(Adafruit_MotorHAT.RELEASE)
        self.motor.run(Adafruit_MotorHAT.RELEASE)

    def drive(self):
        for i in range(100):  # Runs for 10 seconds
            if self.us1.senseObject():
                self.right(self.MAX_SPEED);
            else
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

    def __init__(self, echo, trigger, max_d, trig_d):
        self.ultrasonic = DistanceSensor(echo, trigger, max_distance=max_d, threshold_distance=trig_d)

    def senseObject(self):

 #       Senses if object is within the minimum safe distance
 #       :return: True if an object is within range, false if not

        if self.ultrasonic.when_in_range():
            return True
        else:
            return False

car = car()

car.test()
