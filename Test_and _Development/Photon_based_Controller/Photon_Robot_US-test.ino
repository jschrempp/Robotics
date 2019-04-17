/* Ultrasound Photon Test: test routine to exercise HC-SR04 ultrasound modules
 *  using a Photon and a shift register for the US trigger pulses.  The US
 *  echo signals go directly to (5v tolerant) pins on the Photon.
 *
 * Left US module is connected to SR1 QE pin (decimal value = 16)
 * Front US module is connected to SR1 QF pin (decimal value = 32)
 * Right US module is connected to SR1 QG pin (decimal value = 64)
 * external LED is connected to SR1 QH pin (decimal value = 128)
 * 
 * The test takes US distance readings at 1 second intervals ans
 *  reports the results via publishing them to the Particle cloud.
 *  The results can be viewed on the Particle console.
 *
 * (c) 2019 by Bob Glicksman, Team Practical Projects
 * version 1.0; 4/15/2019
 *
*/

// Global definitions
const int delayTime = 1000;
byte extLED = 128;  // SR1 QH

  // Ultrasound sensor definitions
byte US_LEFT_TRIG = 16;  // SR1 QE
byte US_FRONT_TRIG = 32; // SR QF
byte US_RIGHT_TRIG = 64; // SR1 QG

const int US_LEFT_ECHO = D4;
const int US_FRONT_ECHO = D3;
const int US_RIGHT_ECHO = D2;

  // on-board LED
const int LED_PIN = D7;

  // SR control pins
const int SHIFT_CLK = D5;
const int REG_CLK = A0;
const int SHIFT_DATA = D6;


void setup() {
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(REG_CLK, OUTPUT);
  pinMode(SHIFT_DATA, OUTPUT);
  
  pinMode(US_LEFT_ECHO, INPUT);
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_RIGHT_ECHO, INPUT);
  
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
  static boolean extLEDflag = false;
  
 // take a distance reading on the left ultrasound module
  float leftDistance = measureDistance(US_LEFT_TRIG, US_LEFT_ECHO);
  delay(50); // wait 50 ms between readings
  float frontDistance = measureDistance(US_FRONT_TRIG, US_FRONT_ECHO);
  delay(50); // wait 50 ms between readings
  float rightDistance = measureDistance(US_RIGHT_TRIG, US_RIGHT_ECHO);
  delay(50); // wait 50 ms between readings
  
  // publish the measured distance to the cloud and wait one second
  Particle.publish("Left distance: ", String(leftDistance), PRIVATE);
  delay(delayTime); // wait 1 second so as not to overload publications
  Particle.publish("Front distance: ", String(frontDistance), PRIVATE);
  delay(delayTime); // wait 1 second so as not to overload publications
  Particle.publish("Right distance: ", String(rightDistance), PRIVATE);
  delay(delayTime); // wait 1 second so as not to overload publications
  
  // toggle the external LED once per loop
  if(extLEDflag == false) {
      srWrite(extLED, 1);   // turn on the LED
      digitalWrite(LED_PIN, HIGH);
      extLEDflag = true;    // toggle the flag
  } else {
      srWrite(extLED, 0);   // turn off the LED
      digitalWrite(LED_PIN, LOW);
      extLEDflag = false;    // toggle the flag
  }
  
} // end of loop()

// function to measure distances (in cm) using HC-SR04 ultrasonic sensor
float measureDistance(byte triggerPattern, unsigned int echoPin) {
    // Clear the trigPin
  srWrite(triggerPattern, 0);
  delayMicroseconds(2);
  
  // Set the trigPin on HIGH state for 10 micro seconds
  srWrite(triggerPattern, 1);
  delayMicroseconds(10);
  srWrite(triggerPattern, 0);
  
  // Read the echoPin, return the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  float distance= duration*0.034/2.0;
  
  return distance;

} // end of measureDistance()


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
