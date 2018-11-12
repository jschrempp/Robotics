/* Motor Control Test: Exercizes two motors using TB6612 motor control module.
 *  by: Bob Glicksman, Team Practical projects
 *  version 1.0
 *  11/12/2018
*/

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
const int HIGH_SPEED = 255;
const int LOW_SPEED = 127;

  // LED
  const int LEDPIN = 13;

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // flash the D13 LED twice to indicate setup is complete
  pinMode(LEDPIN, OUTPUT);

  for(int i = 0; i < 1; i++) {
   digitalWrite(LEDPIN, HIGH);
    delay(500); 
    digitalWrite(LEDPIN, LOW);
    delay(500);
  }
}

void loop() {
  // run motor A forward at high speed for 1 second
  forward(MOTORA, HIGH_SPEED);
  delay(1000);
  stop(MOTORA);
  delay(1000);
  stop(MOTORA);

   // run motor B forward at high speed for 1 second
  forward(MOTORB, HIGH_SPEED);
  delay(1000);
  stop(MOTORB);
  delay(1000);
  stop(MOTORB); 
  
  // stop the loop -- reset to get it going again
  while(true){
  }
}


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
}

// stop a motor
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
}
