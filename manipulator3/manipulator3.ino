#include <Servo.h>
#include "numericalTools.h"

float Q[4];
Servo servo[5];

void getLinePath( float x0, float x1, int n, float *x ) {
  for( int k = 0; k < n; k++ )
    x[k] = (x1-x0)*(-cos(k*PI/n)/2.0 + 0.5) + x0;
}

void DKFunc( float* Q, float* S ) {
  float q1 = 80*PI/180.0 - Q[0], q2 = -Q[1] + 97*PI/180.0, q3 = 110*PI/180.0 - Q[2], q4 = -PI/2 + Q[3];
  const float l1 = 18.25e-3, l2 = 147.5e-3, l3 = 187.5e-3, l4 = 85e-3, h1 = 70e-3;
  
  float ra  = l1 + l2*sin(q2) + l3*cos(q2+q3);
  float rb  = l4*cos(q2+q3)*cos(q4);
  float z = h1 + l2*cos(q2) - l3*sin(q2+q3) - l4*sin(q2+q3)*cos(q4);
  float qz = q1 + q4;

  float x = ra*cos(q1) + rb*cos(qz);
  float y = ra*sin(q1) + rb*sin(qz);
  
  S[0] = x;
  S[1] = y;
  S[2] = z;
  S[3] = qz;
}

void moveTo2( float* Q, float* S ) {
  
  // Calculate final angles
  float Qg[4];
  if( solveNLS( &DKFunc, S, 4, Qg ) < 0 ) {
    Serial.println( "Problem!");
    return;
  }
  
  // Calculate current position
  float S0[4];
  DKFunc( Q, S0 );
  
  // Calculate time bottleneck
  float dS[] = { S[0] - S0[0], S[1] - S0[1], S[2] - S0[2], S[3] - S0[3] };
  const float MAX_SPEED[] = { 10e-2/1, 10e-2/1, 10e-2/1, 90*PI/180/1 };
  float timeDelay = 0;
  for( int n = 0; n < 4; n++ )
    timeDelay = max( timeDelay, dS[n] / MAX_SPEED[n] );
    
  // Move motors by steps
  const int T_ms = 50;
  int iters = timeDelay*1000/T_ms;
  for( int k = 0; k < iters; k++ ) {
    for( int n = 0; n < 4; n++ )
      servo[n].write( ( Q[n] + (Qg[n]-Q[n])*k*1.0/iters ) * 180/PI );
    delay( T_ms );
  }
  
  // Update angles vector
  for( int n = 0; n < 4; n++ )
    Q[n] = Qg[n];
}
  
void moveTo( float* Q, float x, float y, float z, float qz, int T_ms, int dt_ms ) {
  float Q1[4];
  float S0[4];
  const int n = round(T_ms*1.0/dt_ms) + 1;
  float X[n], Y[n], Z[n], QZ[n];
  
//  Serial.print("Moving to \t");
//  Serial.print( x*1e2 );
//  Serial.print('\t');
//  Serial.print( y*1e2 );
//  Serial.print('\t');
//  Serial.print( z*1e2 );
//  Serial.print('\t');
//  Serial.print( qy*180/PI );
//  Serial.print('\n');
  
  DKFunc( Q, S0 );
  
  getLinePath( S0[0], x, n, X );
  getLinePath( S0[1], y, n, Y );
  getLinePath( S0[2], z, n, Z );
  getLinePath( S0[3], qz, n, QZ );
  
  for( int k = 0; k < n; k++ ) {
    long int tinit = millis();
    
    float Sg[] = { X[k], Y[k], Z[k], QZ[k] };
    float Qg[] = { Q[0], Q[1], Q[2], Q[3] };
    int iter = solveNLS( &DKFunc, Sg, 4, Qg );
    
    if( iter >= 0 )
      for( int i = 0; i < 4; i++ ) {
        Q[i] = acos(cos(Qg[i]));
        
        servo[i].write( Q[i] * 180.0/PI );
      }
    else {
      Serial.println("System Failure!");
      printMatrix( (float**) Sg, 1, 4, "Unreachable position = " );
      printMatrix( (float**) S0, 1, 4, "Current position = " );
    }
      
      
    int tdelay = dt_ms - (millis()-tinit);
    if( tdelay < 0 )
      Serial.println("Overun");
    delay( tdelay > 0 ? tdelay : 0 );
  }
  
}

void moveHome( float* Q ) {
  const float XHOME[] = { 20e-2, 0, 250e-3, 0 };
  
  moveTo( Q, 25e-2, 0, 25e-2, 0, 200, 50 );
  moveTo( Q, 25e-2, 0, 20e-2, 0, 200, 50 );
  moveTo( Q, 25e-2, 0, 25e-2, 0, 200, 50 );
  moveTo( Q, 25e-2, 0, 20e-2, 0, 200, 50 );
  moveTo( Q, 25e-2, 0, 25e-2, 0, 200, 50 );
  moveTo( Q, 25e-2, 0, 20e-2, 0, 200, 50 );
  
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
  
//  servo[0].write( 110 );
//  servo[1].write( 90 );
//  servo[2].write( 90 );
//  servo[3].write( 90 );
//  
//  float Q[] = {PI/2 + PI/6, PI/2, PI/2, PI/2 + PI/6};
//  float S[4];
//  DKFunc( Q, S );
//  
//  printMatrix( (float**) S, 4, 1, "S @ 90 deg" );
//  
//  
//  while(1);

  while( 1 ) {
    float t = millis()*1e-3;
    float S[] = { 25e-2, 0, 20e-2, 0 };
    
    moveTo2( Q, S );
  }
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
  
  
  moveTo( Q, 25e-2, -20e-2, 20e-2, 0, 1000, 50 );
  while( digitalRead( 7 ) == HIGH );
  
  long int cumClock = 0;
  for( int k = 0; k < N; k++ ) {
    
    //moveHome( Q );
    
    float th = random(20, 60) * PI/180.0;
    float r = random(20, 25) * 1e-2;
    
    //th = 44 * PI/180.0;
    //r = 23e-2;
    
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
//  moveTo( Q, 15e-2, 0, 25e-2, 0, 500, 50 );
//  moveHome( Q );
//  moveHome( Q );
//  moveHome( Q );
}

