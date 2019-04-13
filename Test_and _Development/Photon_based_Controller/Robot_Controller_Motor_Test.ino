/* Motor Control Test: Exercizes two motors using TB6612 motor control module
 * and shift register for direction control signals.
 *  by: Bob Glicksman, Team Practical Projects
 *  version 1.0
 *  4/13/2019
 * based upon motor test code for the Arduino
 * Motor controls are attached to shift register 1 pins:
 *  BIN2 = QA (pattern = B00000001 = 1)
 *  BIN1 = QB (pattern = B00000010 = 2)
 *  AIN1 = QC (pattern = B00000100 = 4)
 *  AIN2 = QD (pattern = B00001000 = 8)
 * Motor speed is via PWM (analogWrite()) on the following Photon pins
 *  D0 = PWM for motor A
 *  D1 = PWM for motor B
 * The Shift rgister is controlled as follows:
 *  Data In = Photon pin D6
 *  SR clock (shift) = Photon pin D5
 *  Register clock (transfer) = Photon pin A0
*/


// Global definitions

const int delayTime = 1000;
  // Motor A
const int PWMA = D0;
byte AIN1 = 4;  // SR1 QC
byte AIN2 = 8;  // SR1 QD
const int MOTORA = 0;

  // Motor B
const int PWMB = D1;
byte BIN1 = 2;  // SR1 QB
byte BIN2 = 1;  // SR1 QA
const int MOTORB = 1;

  // motor speeds
const int HIGH_SPEED = 255;
const int LOW_SPEED = 127;

  // LED
const int LED_PIN = D7;

  // SR control pins
const int SHIFT_CLK = D5;
const int REG_CLK = A0;
const int SHIFT_DATA = D6;


void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(REG_CLK, OUTPUT);
  pinMode(SHIFT_DATA, OUTPUT);
  
  // clear out SR1
  srClear();

  // flash the D7 LED twice to indicate setup is complete
  pinMode(LED_PIN, OUTPUT);

  for(int i = 0; i < 2; i++) {
   digitalWrite(LED_PIN, HIGH);
    delay(500); 
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}  // end of setup()

void loop() {
  // run motor A forward at high speed for 1 second
  forward(MOTORA, HIGH_SPEED);
  delay(delayTime);
  stop(MOTORA);
  delay(delayTime);
 

  // run motor A forward at low speed for 1 second
  forward(MOTORA, LOW_SPEED);
  delay(delayTime);
  stop(MOTORA);
  delay(delayTime);

  // run motor A backward at low speed for 1 second
  backward(MOTORA, LOW_SPEED);
  delay(delayTime);
  stop(MOTORA);
  delay(delayTime);
  
   // run motor A backward at high speed for 1 second
  backward(MOTORA, HIGH_SPEED);
  delay(delayTime);
  brake(MOTORA); // hard stop the motor
  delay(delayTime);


   // run motor B forward at high speed for 1 second
  forward(MOTORB, HIGH_SPEED);
  delay(delayTime);
  stop(MOTORB);
  delay(delayTime);

   // run motor B forward at low speed for 1 second
  forward(MOTORB, LOW_SPEED);
  delay(delayTime);
  stop(MOTORB);
  delay(delayTime);

  // run motor B backward at low speed for 1 second
  backward(MOTORB, LOW_SPEED);
  delay(delayTime);
  stop(MOTORB);
  delay(delayTime);
  
   // run motor B backward at high speed for 1 second
  backward(MOTORB, HIGH_SPEED);
  delay(delayTime);
  brake(MOTORB); // hard stop the motor
  delay(delayTime);

  // remove power from both motors by stopping them
  stop(MOTORA);
  stop(MOTORB);
  delay(delayTime);
  
} // end of loop()

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

// function to set control bits via SR1
void srWrite(byte pattern, byte value) {
    static byte currentPattern = 0;
    
    if(value == 0) {    // we must reset the bit in the SR
        currentPattern = currentPattern & (~pattern);
    } else {            // set the bit in the SR
        currentPattern = currentPattern | pattern;
    }
    
    digitalWrite(SHIFT_CLK, LOW);       // make sure that the clock line is low to begin
    shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, currentPattern); // place the pattern into the sr
    
    // copy SR data into the output register
    digitalWrite(REG_CLK, LOW);
    digitalWrite(REG_CLK, HIGH);
    digitalWrite(REG_CLK, LOW);
    
    return;
    
} // end of srWrite()

// function to clear our SR1
void srClear() {
   
    digitalWrite(SHIFT_CLK, LOW);       // make sure that the clock line is low to begin
    shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, 0); // shift zeros into the sr
    
    // copy SR data into the output register
    digitalWrite(REG_CLK, LOW);
    digitalWrite(REG_CLK, HIGH);
    digitalWrite(REG_CLK, LOW);
    
    return;
    
} // end of srClear()