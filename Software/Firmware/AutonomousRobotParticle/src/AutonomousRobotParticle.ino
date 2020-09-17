/* Basic Autonomous Robot (Using Particle Photon): The robot can be controlled manually via an app (Basic Robot Movement) and can also be placed
 *	into autonomous mode.  When in autonomous mode, the robot uses the HC-SR04 ultrasonic range finder to detect an
 *	obsticle in front. If forward movement is obstructed, the robot pivots and measures to find an open path ahead and then
 *	continues forward on that path until again obstructed or commanded out of the autonomous mode.  The robot delares that it is
 *	stuck if it cannot find an open path within a predetermined number of pivots/distance measurements.  If the robot is stuck 
 *	in this manner, it automatically exits autonomous mode and must be commanded manually again via the app.
 *  
 *	This version of the robot has three ultrasonic rangefinders.  The main one is pointed dead forward.  There is also a rangefinder
 *	looking to the left (about 45 degrees) and to the right (about 45 degrees).  The left and right rangefinders detect if the robot has
 *	moved too close to one side.  If so, the robot pivots the other way until the side sensor is cleared.  If the sides are clear,
 *	the robot tries to move forward and avoids obstacles ahead, as described above.
 *  
 *	The micorcontroller used for this project is a Particle Photon.  A 74HC595 shift register is used to expand the Photon's I/O's.
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
 *	This version is compatible with AI2 app version 2.0.  This version adds reporting of the measured front and side distances
 *	back to the app when the robot is in AUTO mode.
 *  
 *	by: Bob Glicksman, Jim Schrempp, Team Practical Projects
 *		Version 3.5x2 9/17/2020
 *			Moved all action reports to reportAction() to allow removal of duplicate seconds
 *				and standard formatting
 *      Version 3.5x 9/11/2020
 * 			This version will spin in place to avoid a close forward obstacle instead of pivot.
 * 			It will spin for 10 seconds before giving up.
 * 			It handles the case of a narrowing corridor pretty well; it finds its way out.
 * 			There is a behavior in a really tight space where the robot will backup but I'm uncertain
 * 				how to force the robot into that case.
 *		Version 3.4 9/3/2020
 *			If pin A2 is grounded during boot, then the photon will connect to WiFi and the Particle cloud
 *			If connected to cloud, there is a function Autonomous Mode that can be called from the console or Particle app on
 *             smart phone to assert autonomous mode. Doing so also disconnects WiFi to conserve power.
 *		Version 3.3 8/30/2020
 *			Robot will now alter course to avoid a side obstacle that gets NEAR_SIDE distance away. This
 *			causes the robot to glide around obstacles before deciding to stop and pivot.
 *  	Version 3.2 8/25/20
 *      	Added version of pulseIn() with timeout; copied from Particle Community 11/16/16, written by "Ric" 
 *  	Version 3.1 8/23/20
 *      Added semi-automatic system mode declaration so that robot can operate without WiFi
 *	Version 3.0 8/22/20
 *		Converted Arduino robot code to the Photon based PCB, according to pin definitions above.
 *	Version 2.5 01/23/19
 *		Removed 3 ms delay between readings due to additional testing.  Set numPivots to zero when ahead is clear but a side
 *		reading is obstructed.  This way, the Robot can pivot away from the obstruction but if forward path is now blocked, a
 *		random pivot can take place.
 *	Version 2.4 01/21/2019
 *		Added a 3ms delay at the end of each ultrasonic sense. Found empiracally that this eliminated ghost "too close"
 *		measurements in an acoustically challenging environment (my kitchen). No delay would have robot go into avoidance
 *		on occasion in the middle of the floor with plenty of clear space. With a 3ms delay I never observed this.
 *	version 2.3 01/19/2019
 * 		Now do ahead check first and only do side check if ahead is clear.
 *		Now always sense distance after movement, before making any decisions. Avoids some movement loops.
 *	version 2.2 01/18/2019
 *		Added random direction when front is too close
 *		Moved pivot counting into main loop
 *		checkAhead considers both obstruction close and clear ahead distances
 *	version 2.1 01/18/2019
 *		Restructured distancemeasure routine
*/

// Set the system mode to semi-automatic so that the robot will ruyn even if there is no Wi-Fi
SYSTEM_MODE(SEMI_AUTOMATIC);

#define version 3.5x2

// Global constants
	// motor speeds
const int HIGH_SPEED = 200;
const int LOW_SPEED = 150;
const int SLOW_SPEED = 100; // for tight turns while seeking open space
const int CREEP_SPEED = 25; // for use when we are pretty much stuck
const int FORWARD = 2;
const int LEFT = 0;
const int RIGHT = 1;

	// pivot time for searching
const int PIVOT_TIME = 400; // time in milliseconds to pivot robot while searching

	// ultrasonic scan and measurement times
const float OBSTRUCTION_CLOSE_DISTANCE = 8.0; // distance (inches) that is too close; must stop and turn
const float TOO_CLOSE_SIDE = 4.0; // distance (inches) that is too close to a side (left/right) sensor; must stop and turn
const float CLEAR_AHEAD = 12.0; // minimum distance (inches) for robot to be OK to move ahead
const float NEAR_SIDE = 6.0; // distance (inches) that is so close to a side we will turn while moving.
const unsigned int TIMEOUT = 20;  // max measurement time is 20 ms or about 11 feet.

	// robot command modes from app
const int NO_COMMAND = -1;
const int MANUAL_MODE = 0;
const int AUTO = 1; //robot drives ahead. May enter SCAN mode.

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

	// Motor A control pins
const int PWMA = D0;
const int AIN1 = 4;	// SR1 QC
const int AIN2 = 8;	// SR1 QD
const int MOTORA = 0;

	// Motor B control pins
const int PWMB = D1;
const int BIN1 = 2;	// SR1 QB
const int BIN2 = 1;	// SR1 QA
const int MOTORB = 1;

	// LED pin definitions
const int LEDpin = D7;	// Photon onboard LED
const int extLED = 128;	// SR1 QH

	// Other Pins
const int WIFI_CONNECT = A2;  // if low photon will connect to WiFi in setup()

// Global Variables
bool g_ForceAutonomousMode = false;

// Cloud Functions

/* cloudGoAutonomous
*  Call this routine (data parameter is ignored) and the robot will enter
*  autonomous navigation mode. It will also disable WiFi to conserve power.
*/
int cloudGoAutonomous (String data) {

	g_ForceAutonomousMode = true;
	Particle.disconnect();
	WiFi.off();

	return 0;

}

/**************************************************************************
 *  setup() 
***************************************************************************/
void setup() {
	// shift register control pins
	pinMode(SHIFT_CLK, OUTPUT);
	pinMode(REG_CLK, OUTPUT);
	pinMode(SHIFT_DATA, OUTPUT);
	
	// motor control pins
	pinMode(PWMA, OUTPUT);
	pinMode(PWMB, OUTPUT);
	
	// ultrasonic sensor pins
	pinMode(F_ECHO_PIN, INPUT);
	pinMode(L_ECHO_PIN, INPUT);
	pinMode(R_ECHO_PIN, INPUT);

	// Photon LED output pin
	pinMode(LEDpin, OUTPUT);

	// Set the baud rate of the bluetooth module
	Serial1.begin(9600);

	// make sure the robot is stopped
	robotStop();
	reportAction("Reset");

	// see if we should connect to the cloud
	pinMode(WIFI_CONNECT, INPUT_PULLUP);
	if(digitalRead(WIFI_CONNECT) == LOW) {
		Particle.connect();
		Particle.function("Autonomous Mode", cloudGoAutonomous);
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
  
	static int currentMode = MANUAL_MODE;  // place in manual mode until commanded otherwise
	//static int notClear = 0; // number of times we have been unable to move forward
	static float frontDistance = 0; // the measured distance ahead
	static float leftDistance = 0;  // the measured clearance to the left
	static float rightDistance = 0; // the measured clearance to the right

	int commandMode = 0;

	commandMode = command(); // look for bluetooth command and process command accordingly

	if (g_ForceAutonomousMode) {
		commandMode = AUTO;
	}
	
	switch (commandMode) {
		case (MANUAL_MODE):    // change the current mode to manual
			currentMode = MANUAL_MODE;
			break;
		case (AUTO):    // change the current mode to auto
			currentMode = AUTO;
			break;
		case (NO_COMMAND):  // leave current mode as it was
			break;
		default:    // leave current mode as it was
		break;
	}
  
	// process automonomous mode
	if(currentMode == AUTO){

		static bool avoidFrontMode = false;
		static int scanDirection = -1;
		static unsigned int avoidStartMillis = 0;

		// XXX Why not move these measures and the reportDistance to above this auto
		//     block so that the application gets to see distance sensing even when not
		//     in auto mode?
		leftDistance = measureDistance(LEFT); // take ultrasonic range measurement left
		rightDistance = measureDistance(RIGHT); // take ultrasonic range measurement right
		frontDistance = measureDistance(FORWARD);  // take ultrasonic range measurement forward

		reportDistance(frontDistance,leftDistance,rightDistance);

		// toggle the on board LED to show that we've completed a distance measure cycle
		static bool LEDStatus = false;
		digitalWrite(LEDpin, LEDStatus);
		LEDStatus = ! LEDStatus;

		if (avoidFrontMode > 0) {
			frontDistance = frontDistance - 4; // we need to find a way out, so let's look for a good long path
		}

		int checkResult;
		checkResult = checkAhead(leftDistance, rightDistance, frontDistance);
		switch (checkResult) {
			case 0: // clear ahead               
				checkResult = checkSides(leftDistance, rightDistance, frontDistance); // check for side obstructions
				if (checkResult != -1) {
					// too close on the side
					String msg = "Avoid S:" + String(checkResult);
					reportAction(msg); 
					switch (checkResult){
						case 0:
							robotFwdTightLeft(LOW_SPEED);
							break;
						case 1:
							robotFwdTightRight(LOW_SPEED);
							break;
						case 3:
							robotPivotLeft(LOW_SPEED);
							break;
						case 4:
							robotPivotRight(LOW_SPEED);
							break;
						default:
							// near or way near both sides	
							robotBack(SLOW_SPEED);
							delay(500);
							robotLeft(CREEP_SPEED);
							delay(250);
							// commandMode = MANUAL_MODE;
							break;
					}
				} else {
					// side and ahead are clear
					reportAction("Go Forward"); 
					robotForward(LOW_SPEED);
					avoidFrontMode = false; // reset avoidance becasue we're running now
				}
				
				break;

			case 1: // close ahead, might go slowly 
			case 2: // obstruction head
			default:

				if (!avoidFrontMode) { 

					reportAction("Avoid F"); 
					avoidFrontMode = true ;  // will be set to 0 if the robot moves ahead

					avoidStartMillis = millis();

					// first time in avoidance, pick a direction
					if (leftDistance > rightDistance) {
						scanDirection = 0;
					} else {
						scanDirection =1;
					}
				
				}

				if (millis() - avoidStartMillis > 10000) {
					// we have been in avoidance mode for 10 seconds
					reportAction("avoid now stops");
					robotStop();
					commandMode = MANUAL_MODE;
				} else {
					scanAway(scanDirection);
				}

				break;

			}
    
  } // end of auto mode processing

} // end of loop()


/**************************************************************************
 *  Function to process bluetooth commands
***************************************************************************/
int command() { 
	static int mode = NO_COMMAND; 
  
	// Test if command character received by bluetooth
	if (Serial1.available() > 0) {

		// Get the char
		char data = (char)Serial1.read();

		// Control the robot according to the command
		switch(data) {

			case 'a': // place the robot in automatic mode. Any other button puts it back in manual mode
				digitalWrite(LEDpin, HIGH);
				srWrite(extLED, 1);
				robotStop();
				reportAction("Robot is autonomous");
				mode = AUTO;
				break;
      
			case 'f': // robot forward
				digitalWrite(LEDpin, HIGH);
				srWrite(extLED, 1);
				robotForward(HIGH_SPEED);
				reportAction("Robot moves forward");
				mode = MANUAL_MODE;
				break;

			case 'l':  // robot tight turn left at slow speed
				digitalWrite(LEDpin, HIGH);
				srWrite(extLED, 1);
				robotLeft(SLOW_SPEED);
				reportAction("Robot pivots left");
				mode = MANUAL_MODE;
				break;
      
			case 'r':  // robot tight turn right at slow speed
				digitalWrite(LEDpin, HIGH);
				srWrite(extLED, 1);
				robotRight(SLOW_SPEED);
				reportAction("Robot pivots right");
				mode = MANUAL_MODE;
				break;

			case 'b':  // robot moves backward at low speed
				digitalWrite(LEDpin, HIGH);
				srWrite(extLED, 1);
				robotBack(LOW_SPEED);;
				reportAction("Robot moves backward");
				mode = MANUAL_MODE;
				break;
      
			case 'w':  // robot turns leftward at low speed
				digitalWrite(LEDpin, HIGH);
				srWrite(extLED, 1);
				robotFwdLft(LOW_SPEED);
				reportAction("Robot turns leftward");
				mode = MANUAL_MODE;
				break;

			case 'e':  // robot turns rightward at low speed
				digitalWrite(LEDpin, HIGH);
				srWrite(extLED, 1);
				robotFwdRgt(LOW_SPEED);
				reportAction("Robot turns rightward");
				mode = MANUAL_MODE;
				break;     

			case 's':  // robot stops by briefly braking and then romoving motor power
				digitalWrite(LEDpin, LOW);
				srWrite(extLED, 0);
				robotStop(); 
				reportAction("Robot brakes then stops");
				mode = MANUAL_MODE;
				break;

			default:
				digitalWrite(LEDpin, LOW);
				srWrite(extLED, 0);
				mode = NO_COMMAND;
				reportAction("Error!");
		}  // end of switch
	} 
	else {  // no character received
		mode = NO_COMMAND;
	}
	
	return mode;
	
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

	String output = theAction;
	output += "|";

	Serial1.print(output);  
  
}	// end of reportAction()

/**************************************************************************
 *	Distance Report to Client Application  
 *		function to format Dist string and send to client. 
 *		(This should really be JSON at some point).
***************************************************************************/

void reportDistance(float front, float left, float right) {
	static unsigned long lastSend = millis();
	if ((millis() - lastSend) > 500) {
		// send a Dist string to client
		String output = "Dist ";
		output += String(left,2);
		output += " ";
		output += String(front,2);
		output += " ";
		output += String(right,2);
		output += "|";

		Serial1.print(output);
    
		lastSend = millis();
	}
  
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
		delayMicroseconds(2);
    
		// Set the trigger pin HIGH for 10 micro seconds
		srWrite(pinToSense, 1);
		delayMicroseconds(10);
		srWrite(pinToSense, 0);
    
		// Read the echoPin, return the sound wave travel time in microseconds
		// duration = pulseIn(pinToEcho, HIGH);
		
		duration = rdPulseIn(pinToEcho, HIGH, TIMEOUT);
		
		if(duration == 0) { // timeout response
			return (TIMEOUT*1000)/74.0/2.0; // convert timeout to microsecs and convert to inches
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


// function to check side distance
// returns:
//    -1:  both sides clear
//     0:  right side near
//     1:  left side near
//     2:  both sides near
//     3:  right side too close
//     4:  left side too close
//     5:  both sides too close
int checkSides(float leftDistance, float rightDistance, float frontDistance) {

	int obstacleDetection = -1;

	// avoid left or right
	if (leftDistance < NEAR_SIDE) {
		obstacleDetection = 1;
	} else if (leftDistance < TOO_CLOSE_SIDE) {
		obstacleDetection = 4;
	} else if (rightDistance < NEAR_SIDE) {
		obstacleDetection = 0;
	} else if (rightDistance < TOO_CLOSE_SIDE) {
		obstacleDetection = 3;
	} else if  (  (leftDistance < NEAR_SIDE)  &&  (rightDistance < NEAR_SIDE) ) {
		obstacleDetection = 2;
	} else if (  (leftDistance < TOO_CLOSE_SIDE)  &&  (rightDistance < TOO_CLOSE_SIDE) ) {
		obstacleDetection = 5;
	}

	return obstacleDetection;
}  // end of checkSides

// function to pivot robot to avoid an obstacle
//	direction 0:pivotleft   1:pivotright

void pivotAway(int direction) {
	switch (direction) {
		case 0:
			robotPivotLeft(PIVOT_TIME);
			break;
		case 1:
			robotPivotRight(PIVOT_TIME);
			break;
  		default:
			break;
  	}
} //end of pivotAway()

// function to scan robot to avoid an obstacle
//	direction 0:scan left   1: scan right

void scanAway(int direction) {
	switch (direction) {
		case 0:
			robotLeft(SLOW_SPEED);
			break;
		case 1:
			robotRight(SLOW_SPEED);
			break;
  		default:
			break;
  	}
} //end of pivotAway()


// function to check ahead for obstacle
//	returns:
//		0:  clear far ahead
//		1:  clear for a little bit
//		2:  obstacle ahead

int checkAhead(float leftDistance, float rightDistance, float frontDistance) {

	if (frontDistance > CLEAR_AHEAD) {
		return 0;
	}
	
	// if we're too close front
	if (frontDistance < OBSTRUCTION_CLOSE_DISTANCE) { // we are too close, sweep the sides to find where clear
		return 2;
	} 
  
	return 1;
  
}  // end of checkAhead()


/**************************************************************************
 *  Robot Movement Functions 
***************************************************************************/

// function to pivot robot right
void robotPivotRight(int howLong) {
	robotRight(SLOW_SPEED);
	delay(howLong);
	robotStop();
}

// function to pivot robot left
void robotPivotLeft(int howLong) {
	robotLeft(SLOW_SPEED);
	delay(howLong);
	robotStop();
}

// function to move robot forward at commanded speed
void robotForward(int speed){
	forward(MOTORA, speed);
	forward(MOTORB, speed);
	return;
}

// function to turn robot left at commanded speed
void robotLeft(int speed){
	forward(MOTORA, speed);
	backward(MOTORB, speed);
	return;
}

// function to turn robot right at commanded speed
void robotRight(int speed){
	backward(MOTORA, speed);
	forward(MOTORB, speed);
	return;
}

// function to move the robot backward at commanded speed
void robotBack(int speed){
	backward(MOTORA, speed);
	backward(MOTORB, speed);
	return;
}

// function to turn the robot rightward at commanded speed
void robotFwdRgt(int speed){
	forward(MOTORA, speed*0.75);
	forward(MOTORB, speed);
	return;
}

// function to turn the robot leftward at commanded speed
void robotFwdLft(int speed){
	forward(MOTORA, speed);
	forward(MOTORB, speed*0.75);
	return;
}
  
// function to turn the robot tight rightward at commanded speed
void robotFwdTightRight(int speed){
  forward(MOTORA, speed*0.25);
  forward(MOTORB, speed);
  return;
}

// function to turn the robot tight leftward at commanded speed
void robotFwdTightLeft(int speed){
  forward(MOTORA, speed);
  forward(MOTORB, speed*0.25);
  return;
}

// function to stop the robot
void robotStop(){
	//first apply the brakes
	brake(MOTORA);
	brake(MOTORB);
	delay(100);

	// now release the breaks so the motor leads are floating
	stop(MOTORA);
	stop(MOTORB);
}

/**************************************************************************
 *  Low Level Motor Movement Functions 
***************************************************************************/

// function to spin a motor forward
void forward(int motor, int speed){
	// clamp the speed between 0 and 255
	if(speed < 0){
		speed = 0;
	} else if(speed > 255){
		speed = 255;
	}
	// set up a motor to go forward at the indicated speed
	switch(motor){
		case MOTORA:
			srWrite(AIN1, 0);
			srWrite(AIN2, 1);
			analogWrite(PWMA, speed);
			break;     
		case MOTORB:
			srWrite(BIN1, 0);
			srWrite(BIN2, 1);
			analogWrite(PWMB, speed);
			break;    
		default:
			break;
	}
	return;
} // end of forward()

// function to spin a motor backward
void backward(int motor, int speed){
	// clamp the speed between 0 and 255
	if(speed < 0){
		speed = 0;
	} else if(speed > 255){
		speed = 255;
	}
	// set up a motor to go backward at the indicated speed
	switch(motor){
		case MOTORA:
			srWrite(AIN1, 1);
			srWrite(AIN2, 0);
			analogWrite(PWMA, speed);
			break;     
		case MOTORB:
			srWrite(BIN1, 1);
			srWrite(BIN2, 0);
			analogWrite(PWMB, speed);
			break;    
		default:
			break;
	}
	return;
} // end of backward()

// function to stop a motor
void stop(int motor){
	switch(motor){
		case MOTORA:
			srWrite(AIN1, 0);
			srWrite(AIN2, 0);
			digitalWrite(PWMA, HIGH);
			break;     
		case MOTORB:
			srWrite(BIN1, 0);
			srWrite(BIN2, 0);
			digitalWrite(PWMB, HIGH);
			break;    
		default:
			break;
	}
	return;
} // end of stop()

// function to brake a motor
void brake(int motor){
	switch(motor){
		case MOTORA:
			srWrite(AIN1, 1);
			srWrite(AIN2, 1);
			digitalWrite(PWMA, LOW);
			break;     
		case MOTORB:
			srWrite(BIN1, 1);
			srWrite(BIN2, 1);
			digitalWrite(PWMB, LOW);
			break;    
		default:
			break;
	}
	return;
} // end of brake()

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

