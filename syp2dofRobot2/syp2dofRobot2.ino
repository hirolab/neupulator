#include <Servo.h>

Servo elbow, wrist;
float min0 = 1024, max0 = 0, min1 = 1024, max1 = 0;

void setup() {

  Serial.begin( 9600 );
  
  elbow.attach( 9 );
  wrist.attach( 8 );
  
  // Calibration
  elbow.write( 0 );
  wrist.write( 90 );

  while( millis() < 5000 ) {

    float val0 = analogRead( A0 );
    float val1 = analogRead( A1 );
    
    min0 = min( min0, val0 );
    max0 = max( max0, val0 );
    min1 = min( min1, val1 );
    max1 = max( max1, val1 );
    
    delay( 50 );
  }
}

void loop() {
  
  // Joint angle mapping
  int angElbow = map( analogRead( A0 ), min0, max0, 0, 150 );
  int angWrist = map( analogRead( A1 ), min1, max1, 0, 170 );
  
  // Actuate
  elbow.write( angElbow );
  wrist.write( angWrist );
  
  delay(50);
}
