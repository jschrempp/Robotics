# Phone controlled robot.  
This project uses a cell phone app to control the robot via Bluetooth commands.  The project uses a combination of the Bluetooth hardware and the motor control hardware that were developed and tested independently in earlier projects.  The Arduino firmware has a set of basic robot control functions (forward, backward, turn right, turn left, move right, move left, stop) which, in turn, use the individual motor control functions developed in one of the earlier projects.
## Hardware.
The hardware is simply a combination of the motor control hardware and the Bluetooth test hardware.  The schematics for wiring these parts are copied in this folder.  Although the schematics are in separate files, they are compatible with each other and both sets of hardware wiring must be present for this project to work. The following files are used for this project:
### Motor_drive_schematic.
This is the schematic for wiring the Arduino Uno to the two motors via the TB6612 motor control module, in PDF form (created using Powerpoint.
### Bluetooth_Test_Hardware_schem.
This is the schematic for wiring the HC-05/06 Bluetooth module to the Arduino Uno and also for wiring in an LED.
## Software.
### RobotMovementTest.ino.
This folder contains the Arduino code (.ino file) to run the robot based upon single character commands from the cell phone app.  The firmware includes functions for controlling the Robot movement which, in turn, use the low level motor control functions developed in an earlier project. 
### BasicRobotControl app.
This is an Android app created in MIT App Inventor 2.  Both source code (.aia) and installable software (.apk) are included.  The “Choose BT device” button displays a list of paired Bluetooth devices and the user must choose the HC-05 (or HC-06) device used for this project.  If the device does not show in the list, then use the phone/tablet’s Bluetooth control to pair the HC-05/06 with the Android phone/tablet. [Note: pairing PIN code is 1234, if needed].
Once the phone/tablet is connected to the project, the buttons on the app may be used to control the robot in real-time.
## Instructions.
1. Build the hardware according to the schematics.  
2. Use the Arduino IDE (version 1.8.7 was used for development and testing) to compile and upload the firmware to the Uno.  Power the Uno from a USB supply or a barrel jack 7-12 volt supply.  Power the motor separately from a 6 volt supply.  Make sure that the 6 volt supply ground and the Uno’s 5 volt ground are connected together.  
3.  Install the app (.apk file) on an Android phone or tablet.
4.  When using the app to control the robot, make sure that the project has a Bluetooth connection to the app.
## What this test project demonstrates.
This test project demonstrates how to create an MIT App Inventor app to control a robot using Bluetooth communication.  It demonstrates how to connect up a Bluetooth module and motor controller (and motors) to an Arduino (Uno) and the firmware necessary to (a) communicate with the phone/tablet for robot commands and status, and (b) controlling 2 motors using a TB6612 motor controller to make the robot move and turn in various ways.
