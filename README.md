# Robotics
Hardware, firmware, software and documentation for low cost robotics experimentation and development using 
Particle Photon and MIT App Inventor 2.

## Story
We (Team Practical Projects) wanted to explore the capabilities and limitations of very inexpensive robot 
technology.  We obtained a basic robot kit for about $12.  This kit contains a plastic chassis, two drive
motors and drive wheels, a front caster wheel, a 4 AA cell battery holder, an on/off switch, and mounting
and assembly hardware.  

In order to run, test and explore the robot, we designed and built a Particle Photon-based printed circuit
board.  We chose the Particle Photon for our microcontroller because of Arduino
code compatibility, the ability to flash new firmware over WiFi, and Particle's IoT cloud functionality.
However, we used an inexpensive bluetooth module for basic communication with the robot (vs. the Particle Cloud)
because we wanted to experiment with our robot both indoors and outdoors, the latter where there is no
WiFi capability.

The printed circuit board that we designed for this project contains all of the electronics needed to control
and monitor the robot.  This board includes:
1. The Particle Photon.
2. A buck power converter (so that the electronics can be powered from the robot's batteries).
3. Interface circuitry for three HC-SR04 ultrasonic distance sensors.
4. An HC-05 bluetooth module.
5. A dual motor controller capable of driving the two motors on the robot.
6. Expansion capability for future development.

We wrote an Android app (in MIT App Inventor 2) to communicate with the robot and control its motion
manually and in an automatic, self-navigating, obstacle-avoiding mode.  

This repository contains complete, open source documention for building and operating this robot.

## Findings
1. The cheap caster wheel that came with the robot kit had a tendency to bind up and send the robot off
at odd angles.  Our solution was to find a suitable ball caster that replaces the robot kit front wheel.

2. In order for the robot to self-navigate, it needs to take distance measurements to obstacles both in
front of the robot and on both sides of the robot.  We tried using a single ultrasonic sensor mounted on a servo but
we found that the overall measurment cycle was too slow for our purposes.  We opted for using three, fixed
mounted ultrasonic distance sensors: one in front and two facing to each side of the robot.  These sensors
are cheap enough for us to use them this way. The side sensors
are needed in the event that the robot approaches a side wall at a heavily oblique angle.  In particular,
we have found that these sensors should face the side somewhere between 45 degrees and 60 degrees from the
robot's front. 

3. The overall ability of the robot to self-navigate and avoid obstacles is fairly impressive, given the
limitations of the sensing capability.  The robot has the ability to find a way out from most obstacles,
including corners, chair and table legs, etc. Self navigation is not perfect at this point.  In particular,
the robot pivots on an axis between the rear drive wheels and not the center of the chassis, which results
in the robot moving a little bit forward as it tries to pivot around to find it's way out of a trap.  A 4 wheel
robot or a robot based on a tank chassis might obviate this problem.  Alternatively, adding a rear sensor and
firmware to back the robot out of a jam (in lieu of pivoting the robot to move out of a jam forward) might
inprove this behavior.

4. The strength of the motors and the size of the drive and caster wheels limit the types of surfaces where the robot
can operate successfully.  A flat hard surface or a very low pile office carpet work well.  Plush carpet and 
height transitions do not work well with this robot. A tank chassis might allow the robot to operate over
small height transitions and on plush surfaces.


We plan to continue testing and trying to improve the robot's ability to self-navigate and avoid obsticles.
The current robot firmware performs very well, given the limitations above, and is fun to watch and to experiment
with.


