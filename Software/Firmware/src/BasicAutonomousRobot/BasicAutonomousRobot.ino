/* Basic Autonomous Robot: The robot can be controlled manually via an app (Basic Robot Movement) and can also be placed
 *  into autonomous mode.  When in autonomous mode, the robot uses the HC-SR04 ultrasonic range finder to detect an
 *  obsticle in front. If forward movement is obstructed, the robot pivots and measures to find an open path ahead and then
 *  continues forward on that path until again obstructed or commanded out of the autonomous mode.  The robot delares that it is
 *  stuck if it cannot find an open path within a predetermined number of pivots/distance measurements.  If the robot is stuck 
 *  in this manner, it automatically exits autonomous mode and must be commanded manually again via the app.
 *  
 *  This version of the robot has three ultrasonic rangefinders.  The main one is pointed dead forward.  There is also a rangefinder
 *  looking to the left (about 45 degrees) and to the right (about 45 degrees).  The left and right rangefinders detect if the robot has
 *  moved too close to one side.  If so, the robot pivots the other way until the side sensor is cleared.  If the sides are clear,
 *  the robot tries to move forward and avoids obstacles ahead, as described above.
 *  
 *  The micorcontroller used for this project is an Arduino Uno, albeit any Arduino with a standard pinout should be usable.  Pin
 *  assignements are as follows:
 *  
 *  Forward ultrasounic rangefinder:  Trigger pin = A1; echo pin = A2
 *  Left ultrasounic rangefinder:  Trigger pin = A0; echo pin = A3
 *  Right ultrasounic rangefinder:  Trigger pin = A4; echo pin = A5
 *  
 *  Left motor (motor A):  in1 = pin 12; in 2 = pin 7; PWM = pin 11
 *  Right motor (motor B):  in1 = pin 2; in 2 = pin 3; PWM = pin 5
 *  
 *  Bluetooth module:  Rx = pin 10 (connected to BT module Tx); Tx = pin 6 (connected to BT module Rx)
 *  
 *  LED connected to pin 8
 *  
 *  This version is compatible with AI2 app version 2.0.  This version adds reporting of the measured front and side distances
 *  back to the app when the robot is in AUTO mode.
 *  
 *  by: Bob Glicksman, Jim Schrempp, Team Practical Projects
 *  Version 2.4 01/21/2019
 *     Added a 3ms delay at the end of each ultrasonic sense. Found empiracally that this eliminated ghost "too close"
 *     measurements in an acoustically challenging environment (my kitchen). No delay would have robot go into avoidance
 *     on occasion in the middle of the floor with plenty of clear space. With a 3ms delay I never observed this.
 *  version 2.3 01/19/2019
 *     Now do ahead check first and only do side check if ahead is clear.
 *     Now always sense distance after movement, before making any decisions. Avoids some movement loops.
 *  version 2.2 01/18/2019
 *     Added random direction when front is too close
 *     Moved pivot counting into main loop
 *     checkAhead considers both obstruction close and clear ahead distances
 *  version 2.1 01/18/2019
 *     Restructured distancemeasure routine
*/

//#define DEBUG   // uncomment this line to use serial monitor for debugging

// libraries to include
#include "Arduino.h"
#include <SoftwareSerial.h>

// Global constants
  // motor speeds
const int HIGH_SPEED = 200;
const int LOW_SPEED = 150;
const int SLOW_SPEED = 100; // for tight turns while seeking open space
const int FORWARD = 2;
const int LEFT = 0;
const int RIGHT = 1;

  // pivot time for searching
const int PIVOT_TIME = 400; // time in milliseconds to pivot robot while searching

  // ultrasonic scan and measurement times
const float OBSTRUCTION_CLOSE_DISTANCE = 8.0; // distance (inches) that is too close; must stop and turn
const float TOO_CLOSE_SIDE = 4.0; // distance (inches) that is too close to a side (left/right) sensor; must stop and turn
const float CLEAR_AHEAD = 12.0; // minimum distance (inches) for robot to be OK to move ahead
const unsigned long TIMEOUT = 20000;  // max measurement time is 22 ms (22000 us) or about 11 feet.

  // robot command modes from app
const int NO_COMMAND = -1;
const int MANUAL = 0;
const int AUTO = 1; //robot drives ahead. May enter SCAN mode.

// Pins
  // ultrasonic rangefined pins
const int F_TRIG_PIN = A1;
const int F_ECHO_PIN = A2;
const int L_TRIG_PIN = A0;
const int L_ECHO_PIN = A3;
const int R_TRIG_PIN = A4;
const int R_ECHO_PIN = A5;

  // Motor A control pins
const int PWMA = 11;
const int AIN1 = 12;
const int AIN2 = 7;
const int MOTORA = 0;

  // Motor B control pins
const int PWMB = 5;
const int BIN1 = 2;
const int BIN2 = 3;
const int MOTORB = 1;

  // bluetooth serial pins
const int BT_RX = 10; // received serial data from the bluetooth module Tx pin
const int BT_TX = 6;  // transmitted data to the bluetooth mosule Rx pint (through a voltage divider)

  // LED pin
const int LEDpin = 8;

// Global variables
SoftwareSerial BTserial(BT_RX, BT_TX);  // instance of SoftwareSerial to communicate with the bluetooth module




/**************************************************************************
 *  setup() 
***************************************************************************/
void setup() {
    // motor control pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

    // LED output pin
  pinMode(LEDpin, OUTPUT);

  // Set the baud rate of the bluetooth module
  BTserial.begin(9600);
 
  // set up ultrasound module pins
  pinMode(F_TRIG_PIN, OUTPUT); 
  pinMode(F_ECHO_PIN, INPUT);
  pinMode(L_TRIG_PIN, OUTPUT); 
  pinMode(L_ECHO_PIN, INPUT);
  pinMode(R_TRIG_PIN, OUTPUT); 
  pinMode(R_ECHO_PIN, INPUT);

  // make sure the robot is stopped
  robotStop();
  BTserial.print("Reset");

  // flash the LED twice to indicate setup is complete
  for(int i = 0; i < 2; i++) {
    digitalWrite(LEDpin, HIGH);
    delay(500); 
    digitalWrite(LEDpin, LOW);
    delay(500);
  }

#ifdef DEBUG
  // Setup serial monitor communication for use in debugging
  Serial.begin(9600);
  Serial.println("Set up complete");
#endif  

}  // end of setup()

/**************************************************************************
 *  loop() 
***************************************************************************/
void loop() {
  
  static int currentMode = MANUAL;  // place in manual mode until commanded otherwise
  static int notClear = 0; // number of times we have been unable to move forward
  static float frontDistance; // the measured distance ahead
  static float leftDistance;  // the measured clearance to the left
  static float rightDistance; // the measured clearance to the right
  int commandMode;

  commandMode = command(); // look for bluetooth command and process command accordingly
  switch (commandMode) {
    case (MANUAL):    // change the current mode to manual
      currentMode = MANUAL;
      break;
    case (AUTO):    // change the current mode to auto
      currentMode = AUTO;
      break;
    case (NO_COMMAND):  // leave current mode as it was
      break;
    default:    // leave current mode as it was
      break;
  }
  
#ifdef DEBUG
  Serial.print("mode: ");
  if(currentMode == MANUAL) Serial.println("manual"); else if (currentMode == AUTO) Serial.println("auto"); else Serial.println("undefined");
#endif

  // process automonomous mode
  if(currentMode == AUTO){
    leftDistance = measureDistance(LEFT); // take ultrasonic range measurement left
    rightDistance = measureDistance(RIGHT); // take ultrasonic range measurement right
    frontDistance = measureDistance(FORWARD);  // take ultrasonic range measurement forward

    reportDistance(frontDistance,leftDistance,rightDistance);

#ifdef DEBUG
  Serial.print("measured forward distance: ");
  Serial.println(frontDistance);
  Serial.print("measured left distance: ");
  Serial.println(leftDistance);
  Serial.print("measured right distance: ");
  Serial.println(rightDistance);
  Serial.println();
#endif

    static int numPivots = 0;
    static int pivotDirection = -1;
    int checkResult;

    checkResult = checkAhead(leftDistance, rightDistance, frontDistance);
    switch (checkResult) {
      case 0: // all clear front, could go fast               
        checkResult = checkSides(leftDistance, rightDistance, frontDistance); // check for obstructions
        if (checkResult != -1) {
           BTserial.print("Side avoidance|"); // This can be removed eventually
           pivotAway(checkResult);
           numPivots = 0;
        } else {
           // side and ahead are clear
           robotForward(LOW_SPEED);
           numPivots = 0; // reset avoidance pivots becasue we're running now
        }
        break;
      case 1: // close, might go slowly 
      case 2: // obstruction near by
      default:
        // it would be great to get this avoidance moved into a routine, as this is just one avoidance plan.
        // but if we put it in a routine, how would it know that the avoidance worked and it should
        // reset the avoidance counter?
      
        // we need to pivot
        if (numPivots > 5) {
          // we are stuck
          numPivots = 0;
          robotStop();
          currentMode = MANUAL;
          BTserial.print("Stuck, now manual mode|"); // I don't like printing this here. Would be better to have a state transition place to do this.
        } else {
          if (numPivots == 0) {
            // this is our first pivot away attempt, pick a random direction
            BTserial.print("forward obstacle, picking random pivot|"); 
            pivotDirection = random(2);
            numPivots = 1;
          } else {
            // we're going to pivot in the same direction again
            BTserial.print("forward obstacle, same pivot again|"); 
            numPivots++;
          }
        }
        pivotAway(pivotDirection);
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
  if (BTserial.available() > 0) {

    // Get the char
    char data = (char) BTserial.read();

    // Control the robot according to the command
    switch (data) {

    case 'a': // place the robot in automatic mode. Any other button puts it back in manual mode
      digitalWrite(LEDpin, HIGH);
      robotStop();
#ifdef DEBUG
      Serial.println("robot is autonomous");
#endif      
      BTserial.print("Robot is autonomous|");
      mode = AUTO;
      break;
      
    case 'f': // robot forward
      digitalWrite(LEDpin, HIGH);
      robotForward(HIGH_SPEED);
#ifdef DEBUG
      Serial.println("robot moves forward");
#endif
      BTserial.print("Robot moves forward|");
      mode = MANUAL;
      break;

    case 'l':  // robot tight turn left at slow speed
      digitalWrite(LEDpin, HIGH);
      robotLeft(SLOW_SPEED);
#ifdef DEBUG
      Serial.println("robot pivots left");
#endif
      BTserial.print("Robot pivots left|");
      mode = MANUAL;
      break;
      
    case 'r':  // robot tight turn right at slow speed
      digitalWrite(LEDpin, HIGH);
      robotRight(SLOW_SPEED);
#ifdef DEBUG
      Serial.println("robot piviots right");
#endif
      BTserial.print("Robot pivots right|");
      mode = MANUAL;
      break;

    case 'b':  // robot moves backward at low speed
      digitalWrite(LEDpin, HIGH);
      robotBack(LOW_SPEED);;
#ifdef DEBUG
      Serial.println("robot moves backward");
#endif
      BTserial.print("Robot moves backward|");
      mode = MANUAL;
      break;
      
    case 'w':  // robot turns leftward at low speed
      digitalWrite(LEDpin, HIGH);
      robotFwdLft(LOW_SPEED);
#ifdef DEBUG
      Serial.println("robot turns leftward");
#endif
      BTserial.print("Robot turns leftward|");
      mode = MANUAL;
      break;

    case 'e':  // robot turns rightward at low speed
      digitalWrite(LEDpin, HIGH);
      robotFwdRgt(LOW_SPEED);
#ifdef DEBUG
      Serial.println("robot turns rightward");
#endif
      BTserial.print("Robot turns rightward|");
      mode = MANUAL;
      break;     

    case 's':  // robot stops by briefly braking and then romoving motor power
      digitalWrite(LEDpin, LOW);
      robotStop(); 
#ifdef DEBUG
      Serial.println("robot brakes then stop|");
#endif
      BTserial.print("Robot brakes then stops|");
      mode = MANUAL;
      break;

    default:
      digitalWrite(LEDpin, LOW);
      mode = NO_COMMAND;
#ifdef DEBUG
      Serial.print("NOT RECOGNISED: ");
      Serial.println(data);
#endif
      BTserial.print("Error!|");
    }  // end of switch
  } else {  // no character received
    mode = NO_COMMAND;
  }
  return mode;
} // end of command processor


/**************************************************************************
 *  Distance Report to Client Application  
***************************************************************************/
// function to format Dist string and send to client. This should really be JSON at some point
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

    BTserial.print(output);
    
    lastSend = millis();
  }
  
}




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
    digitalWrite(pinToSense, LOW);
    delayMicroseconds(2);
    
    // Set the trigger pin HIGH for 10 micro seconds
    digitalWrite(pinToSense, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinToSense, LOW);
    
    // Read the echoPin, return the sound wave travel time in microseconds
    duration = pulseIn(pinToEcho, HIGH, TIMEOUT);
    if(duration == 0) {
      duration = TIMEOUT; // if it timed out the pulseIn(), set the value to the timeout
    }
    
  } else { 
    duration = -1;  // INVALID SENSOR CALLED FOR
  }

  //delay(3);
   
  // Calculate the distance
  return duration/74.0/2.0;  // conversion of microseconds to inches  
  
} // end of measureDistance()


// function to check side distance
// returns:
//    -1:  both sides clear
//     0:  right side too close
//     1:  left side too close
//     2:  both sides too close
int checkSides(float leftDistance, float rightDistance, float frontDistance) {

  int direction = -1;
  
  // avoid left or right
  if (leftDistance < TOO_CLOSE_SIDE) {
    direction = 1;
  } else if (rightDistance < TOO_CLOSE_SIDE) {
    direction = 0;
  } else if (  (leftDistance < TOO_CLOSE_SIDE)  &&  (rightDistance < TOO_CLOSE_SIDE) ) {
    direction = 2;
  }

  return direction;
}  // end of checkSides

// function to pivot robot to avoid an obstacle
//   direction 0:pivotleft   1:pivotright
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
} //end of pivotAway


// function to check ahead for obstacle
// returns:
//     0:  clear far ahead
//     1:  clear for a little bit
//     2:  obstacle ahead
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
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      analogWrite(PWMA, speed);
      break;     
    case MOTORB:
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
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
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      analogWrite(PWMA, speed);
      break;     
    case MOTORB:
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
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
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(PWMA, HIGH);
      break;     
    case MOTORB:
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
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
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      digitalWrite(PWMA, LOW);
      break;     
    case MOTORB:
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, HIGH);
      digitalWrite(PWMB, LOW);
      break;    
    default:
      break;
  }
  return;
} // end of brake()
