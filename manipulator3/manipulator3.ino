#include <Servo.h>
#include "numericalTools.h"

float Q[4];
Servo servo[5];

void getLinePath( float x0, float x1, int n, float *x ) {
  for( int k = 0; k < n; k++ )
    x[k] = (x1-x0)*(-cos(k*PI/n)/2.0 + 0.5) + x0;
}

/*
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
*/

void DKFunc( float* Q, float* S ) {
  float q1 = 80*PI/180.0 - Q[0], q2 = -Q[1] + 97*PI/180.0, q3 = 110*PI/180.0 - Q[2], q4 = -PI/2 + Q[3];

  float (*F)[1] = (float(*)[1]) S;
  
  F[0][0] = cos(q1+q2+q3)*9.375000000000001E-2+cos(-q1-q2-q3+q4*1.0)*2.125\
E-2+cos(-q1+q2*1.0+q3*1.0-q4)*2.125E-2+cos(q1+q2+q3+q4)*2.125E-2-cos(-q1+q\
4*1.0)*4.25E-2+cos(q1+q4)*4.25E-2+sin(-q1+q2*1.0)*7.375E-2+cos(-q1+q2*1.0+\
q3*1.0)*9.374999999999999E-2+sin(q1+q2)*7.375E-2+cos(-q1+q2*1.0+q3*1.0+q4*\
1.0)*2.125E-2+cos(q1)*1.825E-2;
  F[1][0] = sin(q1+q2+q3)*9.375000000000001E-2-sin(-q1-q2-q3+q4*1.0)*2.125\
E-2-sin(-q1+q2*1.0+q3*1.0-q4)*2.125E-2+sin(q1+q2+q3+q4)*2.125E-2+cos(-q1+q\
2*1.0)*7.375E-2-cos(q1+q2)*7.375E-2+sin(-q1+q4*1.0)*4.25E-2+sin(q1+q4)*4.2\
5E-2-sin(-q1+q2*1.0+q3*1.0)*9.374999999999999E-2-sin(-q1+q2*1.0+q3*1.0+q4*\
1.0)*2.125E-2+sin(q1)*1.825E-2;
  F[2][0] = sin(q2+q3+q4)*-4.25E-2+sin(-q2-q3+q4*1.0)*4.25E-2-sin(q2+q3)*1\
.875E-1+cos(q2)*1.475E-1+sin(q4)*5.204748896376251E-18+7.0E-2;
  F[3][0] = q1 + q4;
}

void moveTo2( float* Q, float* S ) {

  // Calculate current position
  float S0[4];
  DKFunc( Q, S0 );
  
  // Calculate time bottleneck
  float dS[] = { S[0] - S0[0], S[1] - S0[1], S[2] - S0[2], S[3] - S0[3] };
  float disp = sqrt( dS[0]*dS[0] + dS[1]*dS[1] + dS[2]*dS[2] );
  const float MAX_SPEED = 20e-2/1.0;
  float timeDelay = disp / MAX_SPEED;  
    
  // Move motors by steps
  const long int T_ms = 50;
  int iters = timeDelay * 1000.0 / T_ms;
  for( int k = 1; k <= iters; k++ ) {
    long int t0 = millis();
    
    float s[] = { S0[0] + k*dS[0]/iters, S0[1] + k*dS[1]/iters, S0[2] + k*dS[2]/iters, S0[3] + k*dS[3]/iters };
    float Qg[] = { Q[0], Q[1], Q[2], Q[3] };
    
    if( solveNLS( &DKFunc, s, 4, Qg ) < 0 ) {
      Serial.println( "Problem!");
      for( int n = 0; n < 4; n++ )
        Q[n] += random(-5,5)*PI/180;
      continue;
    }

    for( int n = 0; n < 4; n++ ) {
      Q[n] = acos( cos( Qg[n] ) );
      servo[n].write( Q[n] * 180/PI );
    }
    long int dt = T_ms - ( millis() - t0 );
    delay( max( dt, 0 ) );
  }
}

void moveTo4( float* Q, float* S ) {
  
  // Calculate current position
  float S0[4];
  DKFunc( Q, S0 );
  
  // Calculate time bottleneck
  float dS[] = { S[0] - S0[0], S[1] - S0[1], S[2] - S0[2], S[3] - S0[3] };
  float ds = sqrt( dS[0]*dS[0] + dS[1]*dS[1] + dS[2]*dS[2] );
  const float MAX_SPEED = 30e-2/1.0;
  float timeDelay = ds / MAX_SPEED;  
    
  printMatrix( (float**) dS, 4, 1, "dS = ") ;
  
  // Move motors by steps
  const int T_ms = 50;
  int iters = timeDelay*1000.0/T_ms;
  Serial.println( iters );
  for( int k = 0; k < iters; k++ ) {
    long int t0 = millis();
    float dQ[4];
    
    // calculate dQ
    float J[4][4];
    float B[] = { dS[0]/iters, dS[1]/iters, dS[2]/iters, dS[3]/iters }; 
    calcJacobian( &DKFunc, Q, 4, (float**) J );
    solveLS( (float**) J, (float**) B, 4, (float**) dQ );
    
    for( int n = 0; n < 4; n++ )
      if( isnan( dQ[n] ) ) {
        for( int n = 0; n < 4; n++ )
          dQ[n] = random(-5,5)*PI/180;
        Serial.println("____________________________________________________________");
        printMatrix( (float**) Q, 4, 1, "Q = " );
      }
    
    for( int n = 0; n < 4; n++ ) {
        
      Q[n] = acos( cos( Q[n] + dQ[n] ) );
      servo[n].write( Q[n] * 180/PI );
    }
    long int dt = T_ms - ( millis() - t0 );
    delay( max( dt, 0 ) );
  }
}

void returnHome( int times ) {
  float S_HOME[] = { 15e-2, 0, 25e-2, 0 };
  
  moveTo4( Q, S_HOME );
  for( int k = 0; k < times; k++ ) {
    float Sup[] = { S_HOME[0], S_HOME[1], S_HOME[2] + 5e-2, S_HOME[3] };
    moveTo4( Q, Sup );
    float Sdown[] = { S_HOME[0], S_HOME[1], S_HOME[2] - 5e-2, S_HOME[3] };
    moveTo4( Q, Sdown );
  }
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


#define DT 200


void loop() {
  Serial.println("\nNew game =====================================");
  
  const long int TIMEOUT =  15000;
  const int N = 5;
  float S_START[] = { 25e-2, -5e-2, 20e-2, 0 };
  
  moveTo4( Q, S_START );
  while( digitalRead( 7 ) == HIGH );
  
  long int cumClock = 0;
  for( int k = 0; k < N; k++ ) {
    
    returnHome( 2 );
    
    float x = random(10, 25) * 1e-2;
    float y = random(0, 20) * 1e-2;
    
//    th = 0 * PI/180.0;
//    r = 23e-2;
    
    Serial.print("\n  New setpoint: \t x = ");
    Serial.print( x*1e2 );
    Serial.print(",\t y = ");
    Serial.println( y*1e2 );
    
    float S_point[] = { 480e-3-x, -5e-2, 250e-3-y, 0 };
    moveTo4( Q, S_point );
    
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
  returnHome( 10 );
}

