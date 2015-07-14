#include <Servo.h>

Servo elbow, wrist;
float min0 = 1024, max0 = 0, min1 = 1024, max1 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  
  elbow.attach( 2 );
  wrist.attach( 3 );
  
  elbow.write( 90 );
  wrist.write( 90 );
  
  //while(1);
  
  while( millis() < 5000 ) {
    // Filtering
    float average0 = 0, average1 = 0;
    
    for( int k = 0; k < 10; k++ ) {
      average0 += analogRead( A0 ) / 10.0;
      average1 += analogRead( A1 ) / 10.0;
    }
    
    // Online self tuning
    min0 = min( min0, average0 );
    max0 = max( max0, average0 );
    min1 = min( min1, average1 );
    max1 = max( max1, average1 );
    
    delay( 50 );
  }
  
  Serial.print("Calibration data: ");
  Serial.print( min0 ); Serial.print('\t');
  Serial.print( max0 ); Serial.print('\t');
  Serial.print( min1 ); Serial.print('\t');
  Serial.print( max1 ); Serial.print('\n');
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Filtering
  float average0 = 0, average1 = 0;
  
  for( int k = 0; k < 10; k++ ) {
    average0 += analogRead( A0 ) / 10.0;
    average1 += analogRead( A1 ) / 10.0;
  }
  
//  // Online self tuning
//  min0 = min( min0, average0 );
//  max0 = max( max0, average0 );
//  min1 = min( min1, average1 );
//  max1 = max( max1, average1 );
  
  // Joint angle estimation
  int angElbow = map( average0, min0, max0, 0, 180 );
  int angWrist = map( average1, min1, max1, 30, 180 );
  
  // Motion constrain
  //angElbow = constrain( angElbow, 10, 180 );
  //angWrist = constrain( angWrist, max( 0, angElbow + 10), 180 );
  
  // Actuate
  elbow.write( angElbow );
  wrist.write( angWrist );
//  elbow.write( 90 );
//  wrist.write( 90 );
  
  
  delay(50);
  
  // Print for callibration
  Serial.print( analogRead(A0) );
  Serial.print('\t');
  Serial.println( analogRead(A1) );

  // Print angles
//  Serial.print( angElbow );
//  Serial.print('\t');
//  Serial.println( angWrist );
}
