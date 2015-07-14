#include <Servo.h>
#include "numericalTools.h"

float Q[4];
Servo servo[5];

void getLinePath( float x0, float x1, int n, float *x ) {
  for( int k = 0; k < n; k++ )
    x[k] = (x1-x0)*(-cos(k*PI/n)/2.0 + 0.5) + x0;
}

void DKFunc( float* X, float* Y ) {
  float q1 = 80*PI/180.0 - X[0], q2 = 97*PI/180.0 - X[1], q3 = - 110*PI/180.0 + X[2], q4 = PI/2 - X[3];
  const float l1 = 18.25e-3, l2 = 147.5e-3, l3 = 187.5e-3, l4 = 85e-3, h1 = 70e-3;
  
  float r = l1 + l2*sin(q2) + l3*cos(q3-q2) + l4*cos(q4+q3-q2);
  float th = q1;
  float z = h1 + l2*cos(q2) + l3*sin(q3-q2) + l4*sin(q4+q3-q2);
  float qy = q2 - q3 - q4;
  
  float x = r*cos(th);
  float y = r*sin(th);
  
  Y[0] = x;
  Y[1] = y;
  Y[2] = z;
  Y[3] = qy;
}
// 
//void DKFunc( float* X, float* Y ) {
//  
//  float (*A0)[1] = (float(*)[1]) Y;
//  float q1 = X[0], q2 = X[1], q3 = X[2], q4 = X[3];
//  
//  A0[0][0] = sin(3.141592653589793*(1.0/6.0E1)+q1-q2+q3)*(-3.0/3.2E1)-sin(\
//3.141592653589793*(1.7E1/1.8E2)+q1+q2-q3)*(3.0/3.2E1)-sin(3.14159265358979\
//3*(7.0/9.0E1)+q1+q2)*(5.9E1/8.0E2)+cos(3.141592653589793*(1.0/6.0E1)+q1-q2\
//+q3+q4)*(1.7E1/4.0E2)-cos(3.141592653589793*(-7.0/1.5E1)+q1-q2)*(5.9E1/8.0\
//E2)-sin(3.141592653589793*(1.0/1.8E1)+q1)*1.825E-2-cos(3.141592653589793*(\
//1.7E1/1.8E2)+q1+q2-q3-q4)*(1.7E1/4.0E2)+2.9E1/1.0E2;
//  A0[1][0] = cos(3.141592653589793*(1.0/6.0E1)+q1-q2+q3)*(-3.0/3.2E1)-cos(\
//3.141592653589793*(1.7E1/1.8E2)+q1+q2-q3)*(3.0/3.2E1)-cos(3.14159265358979\
//3*(7.0/9.0E1)+q1+q2)*(5.9E1/8.0E2)-sin(3.141592653589793*(1.0/6.0E1)+q1-q2\
//+q3+q4)*(1.7E1/4.0E2)-cos(3.141592653589793*(1.0/3.0E1)+q1-q2)*(5.9E1/8.0E\
//2)-cos(3.141592653589793*(1.0/1.8E1)+q1)*1.825E-2+cos(3.141592653589793*(-\
//7.3E1/1.8E2)+q1+q2-q3-q4)*(1.7E1/4.0E2);
//  A0[2][0] = cos(3.141592653589793*(7.0/1.8E2)+q2-q3-q4)*(1.7E1/2.0E2)+sin\
//(3.141592653589793*(7.0/1.8E2)+q2-q3)*(3.0/1.6E1)+sin(3.141592653589793*(1\
//.0/4.5E1)+q2)*(5.9E1/4.0E2)+7.0/1.0E2;
//  A0[3][0] = asin(cos(3.141592653589793*(7.0/1.8E2)+q2-q3-q4));
//}

void moveTo( float* Q, float x, float y, float z, float qy, int T_ms, int dt_ms ) {
  float Q1[4];
  float X0[4];
  const int n = round(T_ms*1.0/dt_ms) + 1;
  float X[n], Y[n], Z[n], QY[n];
  
//  Serial.print("Moving to \t");
//  Serial.print( x*1e2 );
//  Serial.print('\t');
//  Serial.print( y*1e2 );
//  Serial.print('\t');
//  Serial.print( z*1e2 );
//  Serial.print('\t');
//  Serial.print( qy*180/PI );
//  Serial.print('\n');
  
  DKFunc( Q, X0 );
  
  getLinePath( X0[0], x, n, X );
  getLinePath( X0[1], y, n, Y );
  getLinePath( X0[2], z, n, Z );
  getLinePath( X0[3], qy, n, QY );
  
  for( int k = 0; k < n; k++ ) {
    long int tinit = millis();
    
    float Xg[] = { X[k], Y[k], Z[k], QY[k] };
    float Qg[] = { Q[0], Q[1], Q[2], Q[3] };
    int iter = solveNLS( &DKFunc, Xg, 4, Qg );
    
    if( iter >= 0 )
      for( int i = 0; i < 4; i++ ) {
        Q[i] = Qg[i];
        servo[i].write( Q[i] * 180.0/PI );
      }
    else {
      Serial.println("System Failure!");
      printMatrix( (float**) Xg, 1, 4, "Unreachable position = " );
    }
      
      
    int tdelay = dt_ms - (millis()-tinit);
    if( tdelay < 0 )
      Serial.println("Overun");
    delay( tdelay > 0 ? tdelay : 0 );
  }
  
}

void moveHome( float* Q ) {
  const float XHOME[] = { 50e-3, 0, 250e-3, 0 };
  
  moveTo( Q, 15e-2, 0, 25e-2, 0, 200, 50 );
  moveTo( Q, 15e-2, 0, 20e-2, 0, 200, 50 );
  moveTo( Q, 15e-2, 0, 25e-2, 0, 200, 50 );
  moveTo( Q, 15e-2, 0, 20e-2, 0, 200, 50 );
  moveTo( Q, 15e-2, 0, 25e-2, 0, 200, 50 );
  moveTo( Q, 15e-2, 0, 20e-2, 0, 200, 50 );
  
}

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  
  pinMode( 7, INPUT_PULLUP );
  
  servo[0].attach( 8 );
  servo[1].attach( 9 );
  servo[2].attach( 10 );
  servo[3].attach( 11 );
  servo[4].attach( 12 );
  
  Q[0] = Q[1] = Q[2] = Q[3] = PI/2;
  servo[4].write( 60 );
}

//void loop() {
//  
//  for( int k = 0; k < 50; k++ ) {
//    float th = k*2*PI/50;
//    
//    float Xg[] = { 200e-3, 50e-3*cos(th), 50e-3*sin(th) + 200e-3, 0 };
//    float Qg[] = { Q[0], Q[1], Q[2], Q[3] };
//    
//    int iter = solveNLS( &DKFunc, Xg, 4, Qg );
//    if( iter >= 0 )
//      for( int n = 0; n < 4; n++ ) {
//        Q[n] = Qg[n];
//        servo[n].write( Q[n] * 180/PI );
//      }
//      
//    delay(50);
//  }
//}


void loop() {
  Serial.println("\nNew game =====================================");
  
  const long int TIMEOUT =  15000;
  const int N = 5;
  
  
  moveTo( Q, 27e-2, 0, 20e-2, -20*PI/180.0, 1000, 50 );
  while( digitalRead( 7 ) == HIGH );
  
  long int cumClock = 0;
  for( int k = 0; k < N; k++ ) {
    
    moveTo( Q, 15e-2, 0, 25e-2, 0, 500, 50 );
    moveHome( Q );
    
    float th = random(-15, 45) * PI/180.0;
    float r = random(10, 15) * 1e-2;
    
    Serial.print("\n  New setpoint: \t th = ");
    Serial.print( th*180/PI );
    Serial.print(",\t r = ");
    Serial.println( r*1e2 );
    
    moveTo( Q, 400e-3-r*cos(th), 0, 250e-3-r*sin(th), 0, 1000, 50 );
    
    Serial.println("    Go!");
    long int t0 = millis();
    
    while( digitalRead( 7 ) == HIGH && millis() - t0 < TIMEOUT );    
    Serial.print( millis() - t0 >= TIMEOUT ? "    Timeout! (" : "    Hit! (" );
    Serial.print( (millis() - t0)/1000.0);
    Serial.println(" s)");
    cumClock += ( millis() - t0 );

    
  }
  

  
  Serial.print("\n  Final Score: ");
  Serial.println( 100 * ( 1 - cumClock*1.0 / (N * TIMEOUT) ) );
  moveTo( Q, 15e-2, 0, 25e-2, 0, 500, 50 );
  moveHome( Q );
  moveHome( Q );
  moveHome( Q );
}

