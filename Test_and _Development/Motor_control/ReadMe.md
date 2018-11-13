# Arduino Motor Control using a TB6612 motor controller IC module.
This folder contains a test of an Arduino Uno running two DC motors through high speed, low speed, forward, reverse, stop and brake operations.  A set of Arduino functions are provided for running these motor operations and setup() and loop() test out all of these functions.
## Hardware.
The hardware consists of an Arduino Uno with  TB6612 based dual motor control module. The following files are used for this project:
### Motor_drive_schematic.
This is the project schematic in PDF form (created using Powerpoint.
### motor_driver_module_info.
This is the manufacturer’s data sheet (PDF format)for the motor driver module.  It includes the module pinout as viewed from the top of the module.
### TB6612_datasheet.
This is the manufacturer’s datasheet for the TB6612 motor driver IC (PDF format).  The datasheet provides functional and electrical specifications for the motor driver.
## Software.
### MotorControlTest.ino.
This folder contains the Arduino code (.ino file) to run two motors through their paces.  The firmware includes functions for moving a motor forward, backward, stop and brake.  Setup() and loop() test out all of these functions on two DC motors.
## Instructions.
1. Build the hardware according to the schematic.  
2. Use the Arduino IDE (verion 1.8.7 was used for development and testing) to compile and upload the code to the Uno.  Power the Uno from a USB supply or a barrel jack 7-12 volt supply.  Power the motor separately from a 6 volt supply.  Make sure that the 6 volt supply ground and the Uno’s 5 volt ground are connected together.  
3.  After uploading the compiled firmward the the Uno, the D13 LED should flash twice and then the motors should move as follows:
- Motor A forward at high speed for one second, then stop.
- Motor A forward at low speed for one second, then stop. 
- Motor A backward at low speed for one second, then stop.
- Motor A backward at high speed for one second, then stop.
- Motor A hard brake.
- Motor B forward at high speed for one second, then stop.
- Motor B forward at low speed for one second, then stop. 
- Motor B backward at low speed for one second, then stop.
- Motor B backward at high speed for one second, then stop.
- Motor B hard brake. 
The code will then stop (infinite loop).  A retest can be achieved by pressing the Uno’s reset button.
## What this test project demonstrates.
Running two motors in all of the operations supported by the TB6612 module.
