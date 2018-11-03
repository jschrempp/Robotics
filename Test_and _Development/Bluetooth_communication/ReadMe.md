# Arduino Bluetooth Communication using HC-05 or HC-06 Bluetooth Module.
This folder contains a test of MIT App Inventor 2 software communicating with an Arduino Uno
over bluetooth to turn an LED on and off.
## Hardware.
The hardware consists of an Arduino Uno with an HC-05 (or HC-06) bluetooth module.  The bluetooth
module communicates with the Arduino using serial asynchronous communication at the default of
9600 baud.  In order to avoid interfering with the USB serial connection to a host using the UNO's
hardware serial port, this project uses the Arduino SoftwareSerial library to communicate with
the bluetooth module.  The following files are used for this project:
### Bluetooth Test Hardware_schem.
This is the project schematic in PDF form (created using fritzing).  Uno pin 4 receives serial data
from the bluetooth module.  Uno pin 6 sends serial data from the Uno to the bluetooth module, using 
a voltage divider to reduce the voltage to approximately 3.3 volts (for compatibility with the
bluetooth module).  Uno pin 8 drives and LED.
## Software.
### BTserial.ino.
This folder contains the Arduino code (.ino file) to receive data from the bluetooth module and
turn the pin 8 LED on and off.  This code was copied from an online source.
### MIT App Inventor Software.
Bluetooth_HC06.aia is the source code (AI2) and Bluetooth_HC06.apk is the build file ready to install
on an Android device that supports bluetooth.
## Instructions.
1. Build the hardware according to the schemation.  
2. Use the Arduino IDE (verion 1.8.7 was used for development and testing) to compile and upload the 
	code to the UNO.  Power the project from the Uno 5 volts supply.  The bluetooth module should blink
	its LED rapidly.
3. Use the Android device's bluetooth settings to find the bluetooth module and pair it with the phone/
	tablet.  The Pin (if required) is 1234.
4. Install the app (.apk) file on the Android device.
5. Open the app and tap the "Choose BT device" button if the associated labet shows "not connected".  The
	HC05 or HC06 device should show in the list picker.  Tap on it to connect.  the label shoild show
	"connected".
6.  The ON button should turn the pin 8 LED on and the OFF button should turn it off.
## What this test project demonstrates.
1. Pairing an HC05/06 module with a phone/tablet.
2. Connecting and HC05/06 module to an Arduino using the SoftwareSerial library.
3. Arduino receiving a single text character from the bluetooth module using it to perform an action,
4. Using the Bluetooth Client component in AI2 to connect to an HC05/06 module and send text characters to it.


