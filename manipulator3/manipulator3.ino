#include <Servo.h>
#include "numericalTools.h"

#define DOF 5
#define DT_MS 50
#define MAX_SPEED 20e-2

float Q[DOF];
Servo joint[DOF];
Servo gripper;

void increment( float* Q, float* dQ ) {
  
  float dir[] = { -1, 1, -1, 1, -1 };
  float offset[] = { 100*PI/180, 90*PI/180, 90*PI/180, 70*PI/180, 90*PI/180 };
        
  const float TOL = 0.2;
  
  bool fail = false;
  for( int n = 0; n < 5; n++ )
    if( fabs( dQ[n] ) > TOL || isnan( dQ[n] ) )
      fail = true;
    
  if( fail ) {
    Serial.println( "Excessive angle increment!" );
    for( int n = 0; n < 5; n++ )
      dQ[n] = random( -100, 100 ) / 100.0 * 5 * PI / 180;
  }
    
  for( int n = 0; n < 5; n++ ) {
    Q[n] = asin( sin( Q[n] + dQ[n] ) );
    joint[n].write( ( dir[n] * Q[n] + offset[n] ) * 180 / PI );
  }
}
  

void DKFunc( float* Q, float* S ) {
  
  float q1 = Q[0], 
        q2 = Q[1],
        q3 = Q[2],
        q4 = Q[3],
        q5 = Q[4];

  float (*F)[1] = (float(*)[1]) S;
  
  F[0][0] = cos(q1)*cos(3.141592653589793*(-1.0/2.0)+q2)*(2.9E1/2.0E2)+cos\
(3.141592653589793*(1.0/2.0)+q5)*sin(q1)*sin(q4)*(1.0/1.0E1)-cos(q1)*cos(q\
3)*sin(3.141592653589793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)-cos(q1)*cos(3.14159\
2653589793*(-1.0/2.0)+q2)*sin(q3)*(1.97E2/1.0E3)-cos(q1)*cos(q3)*sin(3.141\
592653589793*(-1.0/2.0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1\
)-cos(q1)*cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3)*sin(3.1415926535897\
93*(1.0/2.0)+q5)*(1.0/1.0E1)+cos(q1)*cos(q3)*cos(q4)*cos(3.141592653589793\
*(-1.0/2.0)+q2)*cos(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1)-cos(q1)*co\
s(q4)*cos(3.141592653589793*(1.0/2.0)+q5)*sin(q3)*sin(3.141592653589793*(-\
1.0/2.0)+q2)*(1.0/1.0E1);
  F[1][0] = cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q1)*(2.9E1/2.0E2)-cos\
(q3)*sin(q1)*sin(3.141592653589793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)-cos(3.141\
592653589793*(-1.0/2.0)+q2)*sin(q1)*sin(q3)*(1.97E2/1.0E3)-cos(q1)*cos(3.1\
41592653589793*(1.0/2.0)+q5)*sin(q4)*(1.0/1.0E1)-cos(q3)*sin(q1)*sin(3.141\
592653589793*(-1.0/2.0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1\
)-cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q1)*sin(q3)*sin(3.1415926535897\
93*(1.0/2.0)+q5)*(1.0/1.0E1)+cos(q3)*cos(q4)*cos(3.141592653589793*(-1.0/2\
.0)+q2)*cos(3.141592653589793*(1.0/2.0)+q5)*sin(q1)*(1.0/1.0E1)-cos(q4)*co\
s(3.141592653589793*(1.0/2.0)+q5)*sin(q1)*sin(q3)*sin(3.141592653589793*(-\
1.0/2.0)+q2)*(1.0/1.0E1);
  F[2][0] = sin(3.141592653589793*(-1.0/2.0)+q2)*(-2.9E1/2.0E2)-cos(q3)*co\
s(3.141592653589793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)+sin(q3)*sin(3.1415926535\
89793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)-cos(q3)*cos(3.141592653589793*(-1.0/2.\
0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1)+sin(q3)*sin(3.14159\
2653589793*(-1.0/2.0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1)-\
cos(q3)*cos(q4)*cos(3.141592653589793*(1.0/2.0)+q5)*sin(3.141592653589793*\
(-1.0/2.0)+q2)*(1.0/1.0E1)-cos(q4)*cos(3.141592653589793*(-1.0/2.0)+q2)*co\
s(3.141592653589793*(1.0/2.0)+q5)*sin(q3)*(1.0/1.0E1)+6.877E-2;
  F[3][0] = asin(cos(3.141592653589793*(1.0/2.0)+q5)*(sin(q4)*(cos(q3)*cos\
(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17-sin(q3)*sin(3.1415\
92653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.123233995736766E-17)+co\
s(q4)*(cos(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*\
(-1.0/2.0)+q2)*sin(q3)))-sin(3.141592653589793*(1.0/2.0)+q5)*(-cos(q3)*cos\
(3.141592653589793*(-1.0/2.0)+q2)+sin(q3)*sin(3.141592653589793*(-1.0/2.0)\
+q2)-cos(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.1232339957367\
66E-17-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+\
6.123233995736766E-17)*6.123233995736766E-17+sin(q4)*(cos(q3)*sin(3.141592\
653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.1\
23233995736766E-17+3.749399456654644E-33));
  F[4][0] = -atan((cos(3.141592653589793*(1.0/2.0)+q5)*(-cos(q3)*cos(3.141\
592653589793*(-1.0/2.0)+q2)+sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)-c\
os(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17\
-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.1232\
33995736766E-17)*6.123233995736766E-17+sin(q4)*(cos(q3)*sin(3.141592653589\
793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.1232339\
95736766E-17+3.749399456654644E-33)*6.123233995736766E-17-cos(q3)*cos(3.14\
1592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+sin(3.141592653589793*(\
1.0/2.0)+q5)*(sin(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.1232\
33995736766E-17-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.12323399573\
6766E-17+6.123233995736766E-17)+cos(q4)*(cos(q3)*sin(3.141592653589793*(-1\
.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3)))*6.1232339957367\
66E-17+sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+\
cos(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-1\
7-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.123\
233995736766E-17)-sin(q4)*(cos(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)+co\
s(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))+2.295845021658468E-49)/(cos(3.\
141592653589793*(1.0/2.0)+q5)*(-cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q\
2)+sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)-cos(q4)*(cos(q3)*cos(3.141\
592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17-sin(q3)*sin(3.1415926535\
89793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.123233995736766E-17)*6.123233\
995736766E-17+sin(q4)*(cos(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)+cos(3.\
141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.123233995736766E-17+3.7493994566\
54644E-33)+cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*3.749399456654644E\
-33+sin(3.141592653589793*(1.0/2.0)+q5)*(sin(q4)*(cos(q3)*cos(3.1415926535\
89793*(-1.0/2.0)+q2)*6.123233995736766E-17-sin(q3)*sin(3.141592653589793*(\
-1.0/2.0)+q2)*6.123233995736766E-17+6.123233995736766E-17)+cos(q4)*(cos(q3\
)*sin(3.141592653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2\
)*sin(q3)))-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*3.749399456654644\
E-33-cos(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.1232339957367\
66E-17-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+\
6.123233995736766E-17)*6.123233995736766E-17+sin(q4)*(cos(q3)*sin(3.141592\
653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.1\
23233995736766E-17-1.405799628556214E-65));
}

float equivDisp( float* S0, float* S1 ) {
  float dS[] = { S1[0]-S0[0], S1[1]-S0[1], S1[2]-S0[2] };
  return( sqrt( dS[0]*dS[0] + dS[1]*dS[1] + dS[2]*dS[2] ) );
}

void moveTo( float* Q, float* S, int dof ) { // S[] = [x, y, z, --orientation--]
  
  // Calculate current position
  float S0[dof];
  DKFunc( Q, S0 );
  
  float timeDelay = equivDisp( S0, S ) / MAX_SPEED;
  
  // Move motors by steps
  int iters = ceil( timeDelay*1000.0/DT_MS );
  float B[dof];
  for( int n = 0; n < dof; n++ )
    B[n] = (S[n]-S0[n])/iters;

  for( int k = 0; k < iters; k++ ) {
    long int t0 = millis();
    float dQ[dof];
    
    // calculate dQ
    float J[dof][dof];
    
    calcJacobian( &DKFunc, Q, dof, (float**) J );
    solveLS( (float**) J, (float**) B, dof, (float**) dQ );

    increment( Q, dQ );
    
    long int dt = DT_MS - ( millis() - t0 );
    delay( max( dt, 0 ) );
  }
  printMatrix( (float**) S0, dof, 1, "S = " );
}

void returnHome( int times ) {
  float S_HOME[] = { 15e-2, 0, 25e-2, 0 };
  
  moveTo( Q, S_HOME, 4 );
  for( int k = 0; k < times; k++ ) {
    float Sup[] = { S_HOME[0], S_HOME[1], S_HOME[2] + 5e-2, S_HOME[3] };
    moveTo( Q, Sup, 4 );
    float Sdown[] = { S_HOME[0], S_HOME[1], S_HOME[2] - 5e-2, S_HOME[3] };
    moveTo( Q, Sdown, 4 );
  }
}

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  
  joint[0].attach( 31 );
  joint[1].attach( 33 );
  joint[2].attach( 35 );
  joint[3].attach( 37 );
  joint[4].attach( 39 );
  gripper.attach( 41 );
  
  Q[0] = Q[1] = Q[2] = Q[3] = Q[4] = -0 * PI / 180;
//  float dQ[] = { 0, 0, 0, 0, 0 };
  
//  float QQ[] = {0, 0, 0, 0, 0};
//  //angleTest( Q );
//  increment( Q, dQ );
//  
//  while(1);
//  
//  float S0[5];
//  DKFunc( Q, S0 );
//  printMatrix( (float**) S0, DOF, 1, "S0 = " );
//  
  float S1[] = { 25e-2, 15e-2, 10e-2, -0.5, 1 };
  float S2[] = { 25e-2, 15e-2, 30e-2, 0, 0 };
//  //float S[] = { 20e-2, 0, 21.38e-2, 0, 0 };

//  DKFunc( Q, S );
//  printMatrix( (float**) S, DOF, 1, "S here = " );
//  while(1);
  
  while(1) {
    float t = millis() / 1000.0;
    float S[] = { 25e-2, 10e-2*cos(2*PI*0.3*t), 10e-2*sin(2*PI*0.3*t)+25e-2, 0, 0 };
    moveTo( Q, S, DOF );
    //moveTo( Q, S2, DOF );
//    DKFunc( Q, S0 );
//    printMatrix( (float**) Q, DOF, 1, "Q now = " );
//    printMatrix( (float**) S0, DOF, 1, "S now = " );
  }

  while(1);
  
  
  gripper.write( 60 );
}


#define DT 200


void loop() {
  Serial.println("\nNew game =====================================");
  
  const long int TIMEOUT =  15000;
  const int N = 5;
  float S_START[] = { 25e-2, -5e-2, 20e-2, 0 };
  
  moveTo( Q, S_START, DOF );
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
    moveTo( Q, S_point, DOF );
    
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

