# JEAP-Autonomous-Project
The purpose of this project is to explore simple autonomous systems. The project consists of designing and building
a motorized rover that is able to move autonomously. The design consists of a remote controlled car coupled with a
Raspberry Pi 3 Model B+, implementing ultrasonic range detection for obstacle avoidance. We will be overhauling
the current RC control scheme of the car to allow it to be controlled directly from the Raspberry Pi.

# Obstacle Avoidance
Obstacle avoidance will be tested using an array of HR-S04 ultrasonic sensors, with the goal of minimizing error in
sensor readings that could lead to the rover crashing. The current implemetation uses the DistanceSensor class from
the `gpiozero` library.

![Short Test of Autonomous Control](https://i.imgur.com/FqeefRO.gifv)

# Rover Control
The rover control is tailored to integrate with an RC car powered by 2 DC motors. The motors are driven by an
Adafruit Stepper Motor HAT and a Raspberry Pi, implemented using the `Adafruit_MotorHAT` library.

### Contributors
*Everest Witman: Project Manager, Junior Developer
*Joseph Zieg: Senior Developer
*Aaron Seibring: Head Hardware Engineer
*PJ Solomon: Hardware Engineer, Junior Developer

