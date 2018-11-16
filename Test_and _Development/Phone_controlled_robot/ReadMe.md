# Phone controlled robot.  
This project uses a cell phone app to control the robot via Bluetooth commands.  The project uses a combination of the Bluetooth hardware and the motor control hardware that were developed and tested independently in earlier projects.  The Arduino firmware has a set of basic robot control functions (forward, backward, turn right, turn left, stop) which, in turn, use the individual motor control functions developed in one of the earlier projects.
## Hardware.
The hardware is simply a combination of the motor control hardware and the Bluetooth test hardware.  The schematics for wiring these parts are copied in this folder.  Although the schematics are in separate files, they are compatible with each other and both sets of hardware wiring must be present for this project to work. The following files are used for this project:
### Motor_drive_schematic.
This is the schematic for wiring the Arduino Uno to the two motors via the TB6612 motor control module, in PDF form (created using Powerpoint.
### Bluetooth_Test_Hardware_schem.
This is the schematic for wiring the HC-05/06 bluetooth module to the Arduino Uno and also for wiring in an LED.
## Software.
### RobotMovementTest.ino.
This folder contains the Arduino code (.ino file) to run the robot based upon single character commands from the cell phone app.  The firmware includes functions for controlling the Robot movement which, in turn, use the low level motor control functions developed in an earlier project. 
### APP TO BE PROVIDED
## Instructions.
1. Build the hardware according to the schematics.  
2. Use the Arduino IDE (verion 1.8.7 was used for development and testing) to compile and upload the firmware to the Uno.  Power the Uno from a USB supply or a barrel jack 7-12 volt supply.  Power the motor separately from a 6 volt supply.  Make sure that the 6 volt supply ground and the Uno’s 5 volt ground are connected together.  
3.  TO BE SUPPLIED
## What this test project demonstrates.
TO BE SUPPLIED
