# PJ Solomon November 27 2017
#Modified Adafruit Code for RC Car
#Assumes Steering = Motor1, Drive = Motor4

#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

################################# DC motor test!
mySteering = mh.getMotor(1)
myMotor = mh.getMotor(4)

# set the speed to start, from 0 (off) to 255 (max speed)
myMotor.setSpeed(250)
myMotor.run(Adafruit_MotorHAT.FORWARD);
mySteering.setSpeed(0)
mySteering.run(Adafruit_MotorHAT.FORWARD);
# turn on motor
myMotor.run(Adafruit_MotorHAT.RELEASE);
mySteering.run(Adafruit_MotorHAT.RELEASE);


while (True):
    print("\tForward and Right! ")
    myMotor.run(Adafruit_MotorHAT.BACKWARD);
    mySteering.run(Adafruit_MotorHAT.FORWARD)

    mySteering.setSpeed(250);

    print("\tSpeed up...")
    for i in range(250):
        myMotor.setSpeed(i)
        time.sleep(0.01)

    time.sleep(3)

    print("\tSlow down...")
    for i in reversed(range(250)):
        myMotor.setSpeed(i)
        time.sleep(0.01)

    print("\tForward and Left!")
    mySteering.run(Adafruit_MotorHAT.BACKWARD)

    mySteering.setSpeed(250);

    print("\tSpeed up...")
    for i in range(250):
        myMotor.setSpeed(i)
        time.sleep(0.01)

    time.sleep(3)

    print("\tSlow down...")
    for i in reversed(range(250)):
        myMotor.setSpeed(i)
        time.sleep(0.01)

    print("\tRelease")
    myMotor.run(Adafruit_MotorHAT.RELEASE)
    #time.sleep(1.0)
