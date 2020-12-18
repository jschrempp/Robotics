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
because we wanted to experiment with our robot both indoors and outdoors; the latter where there is no
WiFi available.

The printed circuit board that we designed for this project contains all of the electronics needed to control
and monitor the robot.  This board includes:
1. The Particle Photon.
2. A buck power converter (so that the electronics can be powered from the robot's batteries).
3. Interface circuitry for three HC-SR04 ultrasonic distance sensors.
4. An HC-05 bluetooth module.
5. A dual H-bridge motor controller capable of driving the two motors on the robot.
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
are cheap enough for us to use them this way. The side sensors are needed in the event that the robot approaches 
a side wall at a heavily oblique angle.  In particular, we have found that these sensors should face the 
side somewhere between 60 degrees and 90 degrees from the robot's front. We were unable to determine a single 
optimal angle for the side avoidance sensors.  We believe that there is too much dependancy on wall reflectivity 
and corner angles to establish a single, best sensor configuration (owning, we believe, to multi-path effects -- see below).

3. We found that there are dead spots between the three ultrasonic sensors. If the robot approaches an obstacle in 
the dead spot (e.g. a table leg), it will get hung up. We also found that the ultrasonic sensors are subject to multi-path effects.  The
ultrasonic signals can bounce off of multiple walls before returning to the sensor, resulting in incorrect distance measurements.
We also found that it is possible for a multi-path signal reflection from one sensor to be detected by another
sensor.  We created a delay of 1 millisecond between sensor readings in order to reduce this effect.  

4. We found that different surfaces reflect different amounts of ultrasonic energy.  Some surfaces (generally
soft surfaces) are poor reflectors of ultrasonic signals while other surfaces (generally hard surfaces) perform much better.
Unsurprisingly, the robot works best when tested against hard vertical walls set at right angles to each other.

5. Given the limitations of ultrasonic distance sensors, we investigated alternatives but we were unable to find
any suitable alternative sensors in the low cost range.  We tried infrared distance sensors that work by measuring
the strength of reflected infrared energy, but we found that the performance was highly sensitive to the color
of a surface.  White walls worked well, whereas black walls did not work at all (absorbed almost all of the
infrared energy).  We concluded that the best sensor technology would likely be lidar, but the cost of lidar
technology far exceeds the cost constriants of this project.

6. We found that weight distribution on the robot body impacts robot traction. The heaviest item on the 
robot platform are the batteries.  It would be best to place the battery holder directly over axis between
the two rear drive wheels. However, the parts don't fit well in this configuration.  The parts fit best
in the conguration shown on the photo "Robot.png" in this repository; however, traction is not
optimal with this configuration of parts.  Traction is improved by placing the printed circuit board
up front and the battery holder in the back.

7. The overall ability of the robot to self-navigate and avoid obstacles is fairly impressive, given the
limitations of the sensing capability.  The robot has the ability to find a way out from most obstacles,
including corners, chair and table legs, etc. Self navigation is not perfect, however.
The robot pivots on an axis between the rear drive wheels and not the center of the chassis, which results
in the pivot arc being longer than we had expected.  A 4 wheel
robot or a robot based on a tank chassis might obviate this problem.  Alternatively, adding a rear sensor and
firmware to back the robot out of a jam (in lieu of pivoting the robot to move out of a jam forward) might
improve this behavior.

8. The strength of the motors and the size of the drive and caster wheels limit the types of surfaces where the robot
can operate successfully.  A flat hard surface or a very low pile office carpet work well.  Plush carpet and 
height transitions do not work well with this robot. A tank chassis might allow the robot to operate over
small height transitions and on plush surfaces.


We have documented the robot's behavior in autonomous navigation mode (AUTO) in a series of six videos that 
you can view, beginning here:

https://www.youtube.com/watch?v=S9v4ddf_cKs&list=PLz4TQp3CqyALdgTUBRFCJkOn_9tXRT2St

In addition to autonomous mode, the Robot can be manually steered via the app supplied with this project.

## Conclusions
Our conclusion is that the robot can successfully navigate autonomously over a fairly wide range of challenging
conditions, given the limitations of the low cost hardware used.
