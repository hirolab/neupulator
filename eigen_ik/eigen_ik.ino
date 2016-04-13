#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Dense>
#include <Servo.h>
#include "AL5Dm.h"

#define PIN_SV1 3
#define PIN_SV2 4
#define PIN_SV3 5
#define PIN_SV4 6
#define PIN_SVE 7

#define TRAJ_INTERP

Servo sv[4];
Servo gripper;

void write_robot(float *q) {
// Convert virtual joint angles to physical and actuate
//  q: pointer to virtual joints
  
  sv[0].write(-q[0]*180/PI + 90);
  sv[1].write(q[1]*180/PI + 90);
  sv[2].write(q[2]*180/PI + 90);
  sv[3].write(q[3]*180/PI + 90);
}

float q[4] = {0, 0, 0, 0};  // virtual joints

void setup() {

  Serial.begin(115200);

  // Attach servos and set to neutral
  int pins[] = {PIN_SV1, PIN_SV2, PIN_SV3, PIN_SV4};
  for(int k = 0; k < 4; k++) {
    sv[k].attach(pins[k]);
    sv[k].write(90);
  }
  gripper.attach(PIN_SVE);  // attach and close gripper
  gripper.write(60);
}

void loop() {

#if defined(TRAJ_CIRCLE)  // circular trajectory
  float t = millis()/1000.0;
  float target[4] = {0.2, 0.15*sin(2*PI*0.3*t), 0.2+0.15*cos(2*PI*0.3*t), 0};
  
#elif defined(TRAJ_INTERP)  // Interpolated trajectory
  const long int tref[5] = {0, 2000, 4000, 6000, 8000};
  const float sref[][5] = {
    {0.2,  0.15,  0.2+0.15, 0},
    {0.2,  -0.15, 0.2+0.15, 0},
    {0.2,  -0.15, 0.2-0.15, 0},
    {0.2,  0.15,  0.2-0.15, 0},
    {0.2,  0.15,  0.2+0.15, 0}
  };
  int t = millis() % tref[4];
  int k = 0;
  while(t > tref[++k]);
  
  float target[] = {
    map(t, tref[k-1], tref[k], sref[k-1][0]*1e3, sref[k][0]*1e3)*1e-3,
    map(t, tref[k-1], tref[k], sref[k-1][1]*1e3, sref[k][1]*1e3)*1e-3,
    map(t, tref[k-1], tref[k], sref[k-1][2]*1e3, sref[k][2]*1e3)*1e-3,
    map(t, tref[k-1], tref[k], sref[k-1][3]*1e3, sref[k][3]*1e3)*1e-3,
  };

#elif defined(TRAJ_JOYSTICK)  // Trajectory from joystick
  float target[] = {0.2, 0, 0.3, 0};  // complete me
  
#endif

  // Set initial guess slightly closer to center to avoid farther solution
  for(int k = 0; k < 4; k++)
    q[k] *= 0.75;

  float error = ik(target, 4, q); // subscribe q with solution

  write_robot(q);

  // Check error on screen before powering the robot!
  Serial.println(error, DEC);
}
