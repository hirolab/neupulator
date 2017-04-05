#include <Servo.h>

Servo elbow, wrist;
float min0 = 1024, max0 = 0, min1 = 1024, max1 = 0;

void setup() {

  Serial.begin( 9600 );
  
  elbow.attach( 2 );
  wrist.attach( 3 );
}

void loop() {

  float volt0 = 0, volt1 = 0;
  for(int n = 0; n < 10; n++) {
    volt0 += analogRead( A0 ) / 10.0;
    volt1 += analogRead( A1 ) / 10.0;
    delay(5);
  }

  min0 = min( min0, volt0 );
  max0 = max( max0, volt0 );
  min1 = min( min1, volt1 );
  max1 = max( max1, volt1 );
  
  // Joint angle mapping
  int angElbow = map( volt0, min0, max0, 10, 150 );
  int angWrist = map( volt1, min1, max1, 10, 170 );
  
  // Actuate
  elbow.write( angElbow );
  wrist.write( angWrist );
}
