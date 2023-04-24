#include <Arduino.h>
#include <pins_arduino.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
VL53L0X sensor;


Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(12);  // attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop()
{
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      Serial.print(sensor.readRangeContinuousMillimeters());
      if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
            Serial.println();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      Serial.print(sensor.readRangeContinuousMillimeters());
      if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
            Serial.println();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }

}
