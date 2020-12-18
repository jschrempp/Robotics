/* RobotSensorTest (Using Particle Photon): 
 *  This project is used to continuously report the sensors over BT to an Android app. 
 *  The intent is to test for cross talk and spurious distance reporting by the sensors.
 *
 *  The test robot can be controlled manually via an Android app (RobotSensorTest). The app allows the user to
 *  select which sensors are active and what delay will be used between each sensor activation.
 *  
 *	This robot has three ultrasonic rangefinders.  The main one is pointed dead forward.  There is also a rangefinder
 *	looking to the left (about 45 degrees) and to the right (about 45 degrees).  
 *  
 *	The micorcontroller used for this project is a Particle Photon. A 74HC595 shift register is used to expand the Photon's I/O's.
 *	Pin assignements are as follows:
 *  
 * 	Shift Register (SR1) controls: SHIFT_CLK = D5; REG_CLK = A0; SHIFT_DATA = D6
 *
 *	Forward ultrasounic rangefinder:  Trigger pin = SR1 QF (decimal value = 32); echo pin = D3
 *	Left ultrasounic rangefinder:  Trigger pin = SR1 QE (decimal value = 16); echo pin = D4
 *	Right ultrasounic rangefinder:  Trigger pin = SR1 QG (decimal value = 64); echo pin = D2
 *  
 *	Left motor (motor A):  in1 = SR1 QC (decimal value = 4); in2 = SR1 QD (decimal value = 8); PWM pin = D0
 *	Right motor (motor B):  in1 = SR1 QB (decimal value = 2); in 2 = SR1 QA (decimal value = 1); PWM pin = D1
 *  
 *	Bluetooth module:  serial1 Photon Rx connected to BT module Tx; Photon Tx connected to BT module Rx
 *  
 *	LED: Photon: D7.  PCB LED: SR1 QH (decimal value = 128)
 *  
 *  Based on version 3.13 of the AutonomousRobotParticle firmware.
 *  
 *	by: Bob Glicksman, Jim Schrempp, Team Practical Projects
 *		Version 1.0 11/6/2020
*/

// Set the system mode to semi-automatic so that the robot will ruyn even if there is no Wi-Fi
SYSTEM_MODE(SEMI_AUTOMATIC);

#define version 1.0

// Global constants

	// robot command modes from app
typedef enum eAppCommands: int {
	NO_COMMAND = 0,
	DELAY_PLUS_10,  		// plus 10 milliseconds to inter sensor delay
    DELAY_MINUS_10, 		// minus 10 milliseconds from inter sensor delay
 	REPORTING_TOGGLE, 		// start/stop sending distance reports
	RIGHT_SENSOR_TOGGLE,	// turn this sensor on/off
	LEFT_SENSOR_TOGGLE,
	FORWARD_SENSOR_TOGGLE
} eAppCommands ;


	// robot sensor positions
const int LEFT = 0;
const int FORWARD = 1;
const int RIGHT = 2;

// Pins
	// shift register control pins
const int SHIFT_CLK = D5;
const int REG_CLK = A0;
const int SHIFT_DATA = D6;

	// ultrasonic rangefinder pins
const int F_TRIG_PIN = 32;	// SR1 QF
const int F_ECHO_PIN = D3;
const int L_TRIG_PIN = 16;	// SR1 QE
const int L_ECHO_PIN = D4;
const int R_TRIG_PIN = 64;	// SR1 QG
const int R_ECHO_PIN = D2;
	
	// LED pin definitions
const int LEDpin = D7;	// Photon onboard LED
const int extLED = 128;	// SR1 QH

	// Other Pins
const int WIFI_CONNECT = A2;  // if low photon will connect to WiFi in setup()

static float g_frontDistance; // the measured distance ahead
static float g_leftDistance;  // the measured clearance to the left
static float g_rightDistance; // the measured clearance to the right

struct globalvars {
	unsigned int sensorTimeout = 5;  // max measurement time is 20 ms or about 11 feet.
	int sensorDelay = 0; // delay between sensor reads
} g; 





// Cloud Functions

/**************************************************************************
 *  setup() 
***************************************************************************/
void setup() {
	// shift register control pins
	pinMode(SHIFT_CLK, OUTPUT);
	pinMode(REG_CLK, OUTPUT);
	pinMode(SHIFT_DATA, OUTPUT);
	
	// ultrasonic sensor pins
	pinMode(F_ECHO_PIN, INPUT);
	pinMode(L_ECHO_PIN, INPUT);
	pinMode(R_ECHO_PIN, INPUT);

	// Photon LED output pin
	pinMode(LEDpin, OUTPUT);

	// Set the baud rate of the bluetooth module
	Serial1.begin(9600);

	reportAction("Reset");

	// see if we should connect to the cloud
	pinMode(WIFI_CONNECT, INPUT_PULLUP);
	if(digitalRead(WIFI_CONNECT) == LOW) {
		Particle.connect();
	}	

	// flash the LED twice to indicate setup is complete
	for(int i = 0; i < 2; i++) {
		digitalWrite(LEDpin, HIGH);
		delay(500); 
    		digitalWrite(LEDpin, LOW);
		delay(500);
	}

}  // end of setup()

/**************************************************************************
 *  loop() 
***************************************************************************/
void loop() {
  
	static bool enableReporting = false;
	static bool enableForward = true;
	static bool enableLeft = true;
	static bool enableRight = true;

	eAppCommands commandFromBT = NO_COMMAND;

	commandFromBT = (eAppCommands) commandBT(); // look for bluetooth command and process command accordingly
	
	switch (commandFromBT) {
		case (DELAY_PLUS_10):    // change the current mode to manual
			g.sensorDelay += 1;
			reportAction("delay between sensors now: " + String(g.sensorDelay));
			break;
		case (DELAY_MINUS_10):    // change the current mode to auto
			g.sensorDelay -= 1;	
			if (g.sensorDelay < 0) {
				g.sensorDelay = 0;
			}
			reportAction("delay between sensors now: " + String(g.sensorDelay));
			break;
		case (REPORTING_TOGGLE):
			enableReporting = !enableReporting;
			reportAction("reporting start/stop");
			break;
		case (LEFT_SENSOR_TOGGLE):
			enableLeft = !enableLeft;
			reportAction("left toggle on/off");
			break;

		case (RIGHT_SENSOR_TOGGLE):
			enableRight =  !enableRight;
			reportAction("right toggle on/off");
			break;
			
		case (FORWARD_SENSOR_TOGGLE):
			enableForward = !enableForward;
			reportAction("forward toggle on/off");
			break;
			
		case (NO_COMMAND):  // leave current mode as it was
			break;
		default:    // leave current mode as it was
		break;
	}
  
	if (enableReporting) {
		// Take measurements
		if(enableLeft){
			g_leftDistance = measureDistance(LEFT); // take ultrasonic range measurement left
			delay(g.sensorDelay);
		} else {
			g_leftDistance = -1;
		}
		if (enableRight){
			g_rightDistance = measureDistance(RIGHT); // take ultrasonic range measurement right
			delay(g.sensorDelay);
		} else {
			g_rightDistance = -1;
		}
		if (enableForward){
			g_frontDistance = measureDistance(FORWARD);  // take ultrasonic range measurement forward
			delay(g.sensorDelay);
		} else {
			g_frontDistance = -1;
		}

		// why not measure each distance three times and take the lowest number for each?

		reportDistance(g_frontDistance, g_leftDistance, g_rightDistance); 

		// toggle the on board LED to show that we've completed a distance measure cycle
		static bool LEDStatus = false;
		digitalWrite(LEDpin, LEDStatus);
		LEDStatus = ! LEDStatus;
	}

} // end of loop()


/**************************************************************************
 *  Function to process bluetooth commands
***************************************************************************/
int commandBT() { 
	static eAppCommands answer = NO_COMMAND; 
  
	// Test if command character received by bluetooth
	if (Serial1.available() > 0) {

		// Get the char
		char data = (char)Serial1.read();

		// Control the robot according to the command
		switch(data) {

			case '+': // increase delay between sensor reads by 10ms
				answer = DELAY_PLUS_10;
				reportAction("Increase sensor delay by 10ms");
				break;
      
			case '-': // decrease delay between sensor reads by 10ms
				answer = DELAY_MINUS_10;
				reportAction("Decrease sensor delay by 10ms");
				break;

			case 's': // toggle reporting
				answer = REPORTING_TOGGLE;
				break;

			case 'f': // toggle reporting
				answer = FORWARD_SENSOR_TOGGLE;
				break;	

			case 'l': // toggle reporting
				answer = LEFT_SENSOR_TOGGLE;
				break;	

			case 'r': // toggle reporting
				answer = RIGHT_SENSOR_TOGGLE;
				break;
			default:
				answer = NO_COMMAND;
				reportAction("cmd: Error!");
				break;

		}  // end of switch
	} 
	else {  // no character received
		answer = NO_COMMAND;
	}
	
	return answer;
	
} // end of command processor


/**************************************************************************
 *	Action Report to Client Application  
 *		function to format action string and send to client. 
 *		(This should really be JSON at some point).
***************************************************************************/

void reportAction(String theAction) {
	static String lastMsg = "";

	if (lastMsg.indexOf(theAction) >= 0) {
		// we have already sent that message
		return;
	}
	lastMsg = theAction;

	reportDistance(g_frontDistance,g_leftDistance,g_rightDistance);

	String output = theAction;
	output += "|";

	Serial1.print(output);  
	Serial1.print(".|");  
  
}	// end of reportAction()

/**************************************************************************
 *	Distance Report to Client Application  
 *		function to format Dist string and send to client. 
 *		(This should really be JSON at some point).
***************************************************************************/

void reportDistance(float front, float left, float right) {

	String output = "Dist ";
	output += String(left,2);
	output += " ";
	output += String(front,2);
	output += " ";
	output += String(right,2);
	output += "|";

	Serial1.print(output);

}	// end of reportDistance()


/**************************************************************************
 *  Distance measurement and scanning functions  
***************************************************************************/

// function to measure distance in inches using the ultrasonic rangefinder
float measureDistance(int direction){

	long duration; // variable to hold the distance measurement in microseconds
	int pinToSense; // the sensor on this pin will be our ping
	int pinToEcho;  // then sensor on this pin will be our echo

	switch (direction) {
		case FORWARD:
			pinToSense = F_TRIG_PIN;
			pinToEcho = F_ECHO_PIN;
			break;
		case LEFT:
			pinToSense = L_TRIG_PIN;
			pinToEcho = L_ECHO_PIN;
			break;
		case RIGHT:
			pinToSense = R_TRIG_PIN;
			pinToEcho = R_ECHO_PIN;
			break;
		default:
			pinToSense = -1;
	}

	if (pinToSense > -1) {

		// Clear the trigger pin
		srWrite(pinToSense, 0);
		delayMicroseconds(1000);
    
		// Set the trigger pin HIGH for 10 micro seconds
		srWrite(pinToSense, 1);
		delayMicroseconds(10);
		srWrite(pinToSense, 0);
    
		// Read the echoPin, return the sound wave travel time in microseconds
		// duration = pulseIn(pinToEcho, HIGH);
		
		duration = rdPulseIn(pinToEcho, HIGH, g.sensorTimeout);
		
		if(duration == 0) { // timeout response
			return (g.sensorTimeout*1000)/74.0/2.0; // convert timeout to microsecs and convert to inches
		}
		else {
			return duration/74.0/2.0;  // conversion of microseconds to inches  		
		}
    
	} 
	else { 
		duration = -1;  // INVALID SENSOR CALLED FOR
		return duration;
	}  
  
} // end of measureDistance()




/**************************************************************************
 *	Function to set shift register control bits
 *		replace digitalWrite() with srWrite()
***************************************************************************/
void srWrite(byte pattern, byte value) {
	static byte currentPattern = 0;
	
	if(value == 0) {	// we must reset the bit in SR1
		currentPattern = currentPattern & (~pattern);
	}
	else {			// set the bit in SR1
		currentPattern = currentPattern | pattern;
	}
	
	// shift the pattern into SR1
	digitalWrite(SHIFT_CLK, LOW);	// make sure clk is low to begin with
	shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, currentPattern);	// place pattern into SR1
	
	// copy SR1 data into SR1 output register
	digitalWrite(REG_CLK, LOW);
	digitalWrite(REG_CLK, HIGH);
	digitalWrite(REG_CLK, LOW);
	
	return;
}	// end of srWrite()

/**************************************************************************
 *	Function to clear out sift register SR1
***************************************************************************/
void srClear() {
	digitalWrite(SHIFT_CLK, LOW);	// make sure clk is low to begin with
	shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, 0);	// shift zeros into SR1
	
	// copy SR1 data into SR1 output register
	digitalWrite(REG_CLK, LOW);
	digitalWrite(REG_CLK, HIGH);
	digitalWrite(REG_CLK, LOW);
	
	return;
}	// end of srClear()


/************************************************************************
 * pulseIn() function with timeout
 *  Note: the standard Particle pulseIn() does not have a timeout
 *      parameter and the default is 3 seconds - way too long for us.
 *      This function was written by Ric and posted to the Particle
 *      community on Nov 16, 2016.
*************************************************************************/
unsigned long rdPulseIn(int pin, int value, unsigned int timeout) { // note "timeout" is in milliseocnds
    
    unsigned long now = micros();
    while(pinReadFast(pin) == value) { // wait if pin is already "value" when the function is called, but timeout if it never changes
        if (micros() - now > (timeout*1000)) {
            return 0;
        }
    }
    
    now = micros(); // could delete this line if you want only one timeout period from the start until the actual pulse width timing starts
    while (pinReadFast(pin) != value) { // pin is "!value", wait for it to change before we start timing, but timeout if it never changes
        if (micros() - now > (timeout*1000)) { 
            return 0;
        }
    }
    
    now = micros();
    while (pinReadFast(pin) == value) { // start timing the "value" pulse width, but time out if over timeout milliseconds
        if (micros() - now > (timeout*1000)) {
            return 0;
        }
    }
    return micros() - now;
}   // end of rePulseIn()

