/* Robot Movement Test: Moves the Robot using the TB6612 motor control module to control the wheel motors.
 *  Robot commands are received via an HC05/06 bluetooth module running at 9600 baud.
 *  
 *  by: Bob Glicksman, Team Practical Projects
 *  version 1.0
 *  11/15/2018
*/

// libraries to include
#include "Arduino.h"
#include <SoftwareSerial.h>

// Global definitions
  // Motor A
const int PWMA = 11;
const int AIN1 = 12;
const int AIN2 = 7;
const int MOTORA = 0;

  // Motor B
const int PWMB = 5;
const int BIN1 = 2;
const int BIN2 = 3;
const int MOTORB = 1;

  // motor speeds
const int HIGH_SPEED = 200;
const int LOW_SPEED = 100;

  // LED
const int LEDpin = 8;

  //create an instance of SoftwareSerial to communicate with the bluetooth module
SoftwareSerial BTserial(4, 6); // RX , TX

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

  // Setup serial monitor communication for use in debugging
  Serial.begin(9600);
  Serial.println("Set up complete");

  // Set the baud rate of the bluetooth module
  BTserial.begin(9600);

  // make sure the robot is stopped
  robotStop();

  // flash the LED twice to indicate setup is complete
  pinMode(LEDpin, OUTPUT);

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

    case 'f': // robot forward
      Serial.println("ON");
      digitalWrite(LEDpin, HIGH);
      robotForward(HIGH_SPEED);
      BTserial.print("Robot moves forward");
      break;

    case 'l':  // robot tight turn left at low speed
      Serial.println("ON");
      digitalWrite(LEDpin, HIGH);
      robotLeft(LOW_SPEED);
      BTserial.print("Robot turns left");
      break;
      
    case 'r':  // robot tight turn right at low speed
      Serial.println("ON");
      digitalWrite(LEDpin, HIGH);
      robotRight(LOW_SPEED);
      BTserial.print("Robot turns right");
      break;

    case 'b':  // robot moves backward at low speed
      Serial.println("ON");
      digitalWrite(LEDpin, HIGH);
      robotBack(LOW_SPEED);
      BTserial.print("Robot turns right");
      break;
      
    case 'w':  // robot turns leftward at high speed
      Serial.println("ON");
      digitalWrite(LEDpin, HIGH);
      robotFwdLft(HIGH_SPEED);
      BTserial.print("Robot turns rightward");
      break;

    case 'e':  // robot turns rightward at high speed
      Serial.println("ON");
      digitalWrite(LEDpin, HIGH);
      robotFwdRgt(HIGH_SPEED);
      BTserial.print("Robot turns rightward");
      break;     

    case 's':  // robot stops by briefly braking and then romoving motor power
      Serial.println("OFF");
      digitalWrite(LEDpin, LOW);
      robotStop();
      BTserial.print("Robot brakes then stops");
      break;

    default:
      Serial.print("NOT RECOGNISED: ");
      Serial.println(data);
      BTserial.print("Error!");
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
  forward(MOTORA, speed/2);
  forward(MOTORB, speed);
  return;
}

// function to turn the robot leftward at commanded speed
void robotFwdLft(int speed){
  forward(MOTORA, speed);
  forward(MOTORB, speed/2);
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
