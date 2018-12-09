/* Basic Autonomous Robot: The robot can be controlled manually via an app (Basic Robot Movement) and can also be placed
 *  into autonomous mode.  When in autonomous mode, the robot uses the HC-SR04 ultrasonic range finder to detect an
 *  obsticle in front. If forward movement is obstructed, the robot pivots and measures to find an open path ahead and then
 *  continues forward on that path until again obstructed or commanded out of the autonomous mode.  The robot delares that it is
 *  stuck if it cannot find an open path within a predetermined number of pivots/distance measurements.  If the robot is stuck 
 *  in this manner, it automatically exits autonomous mode and must be commanded manually again via the app.
 *  
 *  This version of the robot has one ultrasonic rangefinder pointed dead forward.  It has been decided that two more ultrasonic 
 *  rangefinders will be added to the sides of the robot.  These sensors will be used (in a future version of this firmware) 
 *  to detect if the robot is too close to a wall along the side and will stear the robot away accordingly.  In this version, 
 *  side wall hit detection is NOT implemented and may cause the robot to stall or try to climb the wall!
 *  
 *  by: Bob Glicksman, Team Practical Projects
 *  version 1.2
 *  12/08/2018
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

  // pivot time for searching
const int PIVOT_TIME = 400; // time in milliseconds to pivot robot while searching

  // ultrasonic scan and measurement times
const int SERVO_FRONT_OFFSET = -5;  // offset to achieve mechanical position of front for angle = 90; your servo may vary.
const int FRONT_MEASURE_ANGLE = 90; // front
const float OBSTRUCTION_CLOSE_DISTANCE = 8.0; // distance (inches) that is too close; must stop and turn
const float CLEAR_AHEAD = 18; // minimum distance (inches) for robot to be OK to move ahead

  // robot command modes from app
const int NO_COMMAND = -1;
const int MANUAL = 0;
const int AUTO = 1;

// Pins
  // ultrasonic rangefined pins
const int TRIG_PIN = A1;
const int ECHO_PIN = A2;

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
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);

  // make sure the robot is stopped
  robotStop();

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
  
  static float frontDistance; // the measured distance ahead
  static int currentMode = MANUAL;  // place in manual mode until commanded otherwise
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
    frontDistance = measureDistance();  // take ultrasonic range measurement forward

#ifdef DEBUG
  Serial.print("measured forward distance: ");
  Serial.println(frontDistance);
#endif
  
    if(frontDistance < OBSTRUCTION_CLOSE_DISTANCE){ // we are too close, sweep the sides to find where clear
      int dir = random(2);  // random number between 0 (left) and 1(right)    
      if (scan(dir)) {   // scan; return true if OK to move forward, false if stuck
        robotForward(LOW_SPEED);  // it is clear ahead
      } 
      else {    // we are stuck
        digitalWrite(LEDpin, LOW);
        currentMode = MANUAL;
      }     
      
    } 
    else {    // clear ahead so move forward
      robotForward(LOW_SPEED);  // it is clear ahead
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
      BTserial.print("Robot is autonomous");
      mode = AUTO;
      break;
      
    case 'f': // robot forward
      digitalWrite(LEDpin, HIGH);
      robotForward(HIGH_SPEED);
#ifdef DEBUG
      Serial.println("robot moves forward");
#endif
      BTserial.print("Robot moves forward");
      mode = MANUAL;
      break;

    case 'l':  // robot tight turn left at slow speed
      digitalWrite(LEDpin, HIGH);
      robotLeft(SLOW_SPEED);
#ifdef DEBUG
      Serial.println("robot pivots left");
#endif
      BTserial.print("Robot pivots left");
      mode = MANUAL;
      break;
      
    case 'r':  // robot tight turn right at slow speed
      digitalWrite(LEDpin, HIGH);
      robotRight(SLOW_SPEED);
#ifdef DEBUG
      Serial.println("robot piviots right");
#endif
      BTserial.print("Robot piviots right");
      mode = MANUAL;
      break;

    case 'b':  // robot moves backward at low speed
      digitalWrite(LEDpin, HIGH);
      robotBack(LOW_SPEED);;
#ifdef DEBUG
      Serial.println("robot moves backward");
#endif
      BTserial.print("Robot moves backward");
      mode = MANUAL;
      break;
      
    case 'w':  // robot turns leftward at low speed
      digitalWrite(LEDpin, HIGH);
      robotFwdLft(LOW_SPEED);
#ifdef DEBUG
      Serial.println("robot turns leftward");
#endif
      BTserial.print("Robot turns leftward");
      mode = MANUAL;
      break;

    case 'e':  // robot turns rightward at low speed
      digitalWrite(LEDpin, HIGH);
      robotFwdRgt(LOW_SPEED);
#ifdef DEBUG
      Serial.println("robot turns rightward");
#endif
      BTserial.print("Robot turns rightward");
      mode = MANUAL;
      break;     

    case 's':  // robot stops by briefly braking and then romoving motor power
      digitalWrite(LEDpin, LOW);
      robotStop(); 
#ifdef DEBUG
      Serial.println("robot brakes then stop");
#endif
      BTserial.print("Robot brakes then stops");
      mode = MANUAL;
      break;

    default:
      digitalWrite(LEDpin, LOW);
      mode = NO_COMMAND;
#ifdef DEBUG
      Serial.print("NOT RECOGNISED: ");
      Serial.println(data);
#endif
      BTserial.print("Error!");
    }  // end of switch
  } else {  // no character received
    mode = NO_COMMAND;
  }
  return mode;
} // end of command processor


/**************************************************************************
 *  Distance measurement and scanning functions  
***************************************************************************/

// function to measure distance in inches using the ultrasonic rangefinder
float measureDistance(){

  long duration; // variable to hold the distance measurement in microseconds
  
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin HIGH for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echoPin, return the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  return duration/74.0/2.0;  // conversion of microseconds to inches
  
}  // end of measureDistance()


// function to scan for open space ahead by pivoting the robot.  Returns true of OK ahead, false if stuck
bool scan(int direction) {
  
  const int LEFT = 0;   // direction constant; left is 0, right is other 
  const int MAX_PIVOTS = 5; // declare stuck if exceed this number of pivots
  int pivotCount = 0;
  
  while (pivotCount++ < MAX_PIVOTS) {
    if (direction == LEFT) {
      robotLeft(SLOW_SPEED);
    } else {
      robotRight(SLOW_SPEED);
    }
    delay(PIVOT_TIME);
    robotStop();

    // is it clear ahead?
    if(measureDistance() >= CLEAR_AHEAD) {
      return true;  // break out of loop and return that OK to move ahead
    }    
  }

  // hit the max number of pivots without finding open space ahead  
  robotStop();
  return false;
  
}  // end of scan()

/**************************************************************************
 *  Robot Movement Functions 
***************************************************************************/

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
  delay(500);

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
