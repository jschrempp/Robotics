# Use servo to sweep the ultrasonic range finder and measure distances front and sides.
Mount the SR-HC04 ultrasonic distance sensor on a hobby servo and hot glue the hobby servo to the front of the robot.  Wire the servo control pin to Arduino pin A0.  Wire the ultrasonic rangefinder trigger pin to Arduino pin A1 and the echo pin to Arduino pin A2.

Upload the software to the Arduino.  The servo should scan the distance at angles 30 degrees (right of center), 150 degrees (left of center) and 90 degrees (dead center) and print out the results to the serial monitor.

Adjust the defined constant SERVO_FRONT_OFFSET to center (front) the ultrasonic rangefinder as per your assembly.

