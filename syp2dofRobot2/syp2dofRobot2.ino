#include <Servo.h>

Servo elbow, wrist;
float min0 = 1024, max0 = 0, min1 = 1024, max1 = 0;

void setup() {

  Serial.begin( 9600 );
  
  elbow.attach( 4 );
  wrist.attach( 5 );
  
  // Calibration
  elbow.write( 90 );
  wrist.write( 90 );

  while( millis() < 5000 ) {
    // Filtering
    float average0 = 0, average1 = 0;
    
    for( int k = 0; k < 10; k++ ) {
      average0 += analogRead( A0 ) / 10.0;
      average1 += analogRead( A1 ) / 10.0;
    }
    
    min0 = min( min0, average0 );
    max0 = max( max0, average0 );
    min1 = min( min1, average1 );
    max1 = max( max1, average1 );
    
    delay( 50 );
  }
}

void loop() {
  
  // Filtering
  float average0 = 0, average1 = 0;
  
  for( int k = 0; k < 10; k++ ) {
    average0 += analogRead( A0 ) / 10.0;
    average1 += analogRead( A1 ) / 10.0;
  }
  
  // Joint angle mapping
  int angElbow = map( average0, min0, max0, 30, 150 );
  int angWrist = map( average1, min1, max1, 150, 30 );
  
  // Actuate
  elbow.write( angElbow );
  wrist.write( angWrist );
  
  delay(50);
}
