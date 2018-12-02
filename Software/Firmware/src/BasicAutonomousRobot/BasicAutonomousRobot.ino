/* Basic Autonomous Robot: The robot can be controlled manually via an app (Basic Robot Movement) and can also be placed
 *  into autonomous mode.  When in autonomous mode, the robot uses the HC-SR04 ultrasonic range finder to detect an
 *  obsticale in front and then use a servo to ultrasonically scan the surrounding area for an open position.  The robot
 *  then pivots to the open position, reverifies clearance ahead, and moves forward in that direction.
 *  
 *  by: Bob Glicksman, Team Practical Projects
 *  version 1.0
 *  12/02/2018
*/

#define DEBUG

// libraries to include
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Servo.h>

// Global definitions
  // motor speeds
const int HIGH_SPEED = 200;
const int LOW_SPEED = 150;

  // ultrasonic scan and measurement times
const int MEASURE_TIME = 400;  // time for position servo to take an ultrasonic measurement
const int SERVO_FRONT_OFFSET = -10;  // offset to achieve mechanical position of front for angle = 90; your servo may vary.
const int LEFT_MEASURE_ANGLE = 150; // 60 degrees off to the left of front
const int RIGHT_MEASURE_ANGLE = 30; // 60 degrees off to the right of front
const int FRONT_MEASURE_ANGLE = 90; // front
const float OBSTRUCTION_CLOSE_DISTANCE = 4.0; // 4" away is too close; must stop and turn

// Pins
  // sero and ultrasonic rangefined pins
const int SERVO_PIN = A0;  // servo wiring: attach red wire to Vcc; brown wire to ground; yellow wire to Arduino pin A0
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
Servo sweepServo;  // instance of Servo object to control the untrasonic sweep sero
bool autoMode;  // false = manual mode; true = autonomous mode
float leftDistance, frontDistance, rightDistance; // variables to hold the last distances measured

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

#ifdef DEBUG
  // Setup serial monitor communication for use in debugging
  Serial.begin(9600);
  Serial.println("Set up complete");
#endif  

  // Set the baud rate of the bluetooth module
  BTserial.begin(9600);
 
  // instantiate the sweep servo
  sweepServo.attach(SERVO_PIN);

  // set up ultrasound module pins
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);

  // make sure the robot is stopped
  robotStop();

  // center the servo
  sweepServo.write(90 + SERVO_FRONT_OFFSET);

  // set the mode to manual
  autoMode = false;

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
  
  // Wait for a character to be received by the bluetooth module
  if (BTserial.available() > 0) {

    // Get the char
    char data = (char) BTserial.read();

    // Control the robot according to the command
    switch (data) {

    case 'a': // place the robot in automatic mode. Any other button puts it back in manual mode
      digitalWrite(LEDpin, HIGH);
      robotForward(LOW_SPEED);  // low speed for safety!
#ifdef DEBUG
      Serial.println("robot is autonomous");
#endif      
      BTserial.print("Robot is autonomous");
      autoMode = true;
      break;
      
    case 'f': // robot forward
      digitalWrite(LEDpin, HIGH);
      robotForward(HIGH_SPEED);
#ifdef DEBUG
      Serial.println("robot moves forward");
#endif
      BTserial.print("Robot moves forward");
      autoMode = false;
      break;

    case 'l':  // robot tight turn left at low speed
      digitalWrite(LEDpin, HIGH);
      robotLeft(LOW_SPEED);
#ifdef DEBUG
      Serial.println("robot pivots left");
#endif
      BTserial.print("Robot pivots left");
      autoMode = false;
      break;
      
    case 'r':  // robot tight turn right at low speed
      digitalWrite(LEDpin, HIGH);
      robotRight(LOW_SPEED);
#ifdef DEBUG
      Serial.println("robot piviots right");
#endif
      BTserial.print("Robot piviots right");
      autoMode = false;
      break;

    case 'b':  // robot moves backward at low speed
      digitalWrite(LEDpin, HIGH);
      robotBack(LOW_SPEED);;
#ifdef DEBUG
      Serial.println("robot moves backward");
#endif
      BTserial.print("Robot moves backward");
      autoMode = false;
      break;
      
    case 'w':  // robot turns leftward at high speed
      digitalWrite(LEDpin, HIGH);
      robotFwdLft(HIGH_SPEED);
#ifdef DEBUG
      Serial.println("robot turns leftward");
#endif
      BTserial.print("Robot turns leftward");
      autoMode = false;
      break;

    case 'e':  // robot turns rightward at high speed
      digitalWrite(LEDpin, HIGH);
      robotFwdRgt(HIGH_SPEED);
#ifdef DEBUG
      Serial.println("robot turns rightward");
#endif
      BTserial.print("Robot turns rightward");
      autoMode = false;
      break;     

    case 's':  // robot stops by briefly braking and then romoving motor power
      digitalWrite(LEDpin, LOW);
      robotStop(); 
#ifdef DEBUG
      Serial.println("robot brakes then stop");
#endif
      BTserial.print("Robot brakes then stops");
      autoMode = false;
      break;

    default:
      digitalWrite(LEDpin, LOW);
      autoMode = false;
#ifdef DEBUG
      Serial.print("NOT RECOGNISED: ");
      Serial.println(data);
#endif
      BTserial.print("Error!");
    }
  }

  // process automonomous mode
  if(autoMode){
    // measure distance ahead
    sweepServo.write(FRONT_MEASURE_ANGLE + SERVO_FRONT_OFFSET);
    delay(MEASURE_TIME);  // wait for sero to move
    frontDistance = measureDistance();  // take ultrasonic range measurement forward
    if(frontDistance < OBSTRUCTION_CLOSE_DISTANCE){ // we are too close, sweep the sides to find where clear
      /**** temporarily stop until figure out what to do next ***/
      robotStop();
      digitalWrite(LEDpin, LOW);
      autoMode = false;
    }     
  }
  
} // end of loop()

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

// function to sweep the servo with the ultrasonic range finder to get range front and sides
void sweep(){
  // measure distance to the right
    sweepServo.write(RIGHT_MEASURE_ANGLE + SERVO_FRONT_OFFSET);
    delay(MEASURE_TIME);  // wait for sero to move
    rightDistance = measureDistance();  // take ultrasonic range measurement for the right

    // measure distance to the left
    sweepServo.write(LEFT_MEASURE_ANGLE + SERVO_FRONT_OFFSET);
    delay(MEASURE_TIME);  // wait for sero to move
    leftDistance = measureDistance();  // take ultrasonic range measurement for the left

    // measure distance ahead
    sweepServo.write(FRONT_MEASURE_ANGLE + SERVO_FRONT_OFFSET);
    delay(MEASURE_TIME);  // wait for sero to move
    frontDistance = measureDistance();  // take ultrasonic range measurement for the left

#ifdef DEBUG
    // print to serial monitor for debugging
    Serial.print("distance left = ");
    Serial.print(leftDistance);
    Serial.print("; distance front = ");
    Serial.print(frontDistance);
    Serial.print("; distance rignt = "); 
    Serial.println(rightDistance);   
#endif    
  
}  // end of sweep()


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
