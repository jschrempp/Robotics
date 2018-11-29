/* ServoDistanceTest: tests our offsets and constants to sweep a servo for ultrasonic measurement of robot environment.
    Also runs the SR-HC04 ultrasound module and displays the measured distaces at various angles set by the servo.
 
    by: Bob Glicksman; Team Practical Projects
    date: 11/29/18
*/

// Globals
const int MEASURE_TIME = 400;  // time for position servo to take an ultrasonic measurement
const int SERVO_FRONT_OFFSET = -10;  // offset to achieve mechanical position of front for angle = 90; your servo may vary.
const int MEASURE_ANGLE[] = {30,150,90};  // the angles to take ultrasonic measurements at

// Pins
const int SERVO_PIN = A0;  // servo wiring: attach red wire to Vcc; brown wire to ground; yellow wire to Arduino pin A0
const int TRIG_PIN = A1;
const int ECHO_PIN = A2;

// servo library
#include <Servo.h>

Servo sweepServo;  // create servo object to control a servo

void setup() {
  // instantiate the sweep servo
  sweepServo.attach(SERVO_PIN);

  // set up ultrasound module pins
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);

  // serial port for testing
  Serial.begin(9600); // Starts the serial communication

    // center the servo
  sweepServo.write(90 + SERVO_FRONT_OFFSET);

  // wait 3 seconds to get serial monitor up and running
  delay(3000);
}

void loop() {
  for(int i = 0; i < 5; i++){ // take 5 sweep measurements 
    sweep();
    delay(3000);  // wait 3 seconds between measurements
  }
  while(true);
}

// function to sweep the servo with the ultrasonic range finder to get range front and sides
void sweep(){
  for(int pos = 0; pos < sizeof(MEASURE_ANGLE)/sizeof(int); pos++){
    
    // move the servo with the ultrasound range finder
    sweepServo.write(MEASURE_ANGLE[pos] + SERVO_FRONT_OFFSET);
    delay(MEASURE_TIME);  // wait for sero to move
    
    // measure the distance and print out results to the serial monitor
    Serial.print("distance at angle ");
    Serial.print(MEASURE_ANGLE[pos]);
    Serial.print(" = ");
    Serial.print(measureDistance());
    Serial.println(" inches");
  }

  Serial.println();  // print a blank line between measurement sets
  
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
