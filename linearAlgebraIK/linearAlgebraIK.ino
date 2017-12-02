#include "LinearAlgebra.h"
#include "Robotics.h"
#include <Servo.h>
#include "pitches.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_7segment matrix = Adafruit_7segment();
float _q[4] = {0, 0, 0, 0};

#define ROMEO
//#define JULIET

#define PIN_SENSOR 8
#define PIN_BUZZ 9

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

Servo servo[4], gripper;

void robot_setup() {

  servo[0].attach(2);
  servo[1].attach(3);
  servo[2].attach(4);
  servo[3].attach(5);
  gripper.attach(6);

  servo[0].write(90);
  servo[1].write(90);
  servo[2].write(90);
  servo[3].write(85);
  gripper.write(160);
}

void robot_write(float *s, float *q) {

  const float CONTR = 0.8; // find solution closer to neutral state
  float q0[4] = {q[0]*CONTR, q[1]*CONTR, q[2]*CONTR, q[3]*CONTR};
  float error = ik(s, 4, q0, fk_juliet);

  if (error < 1e-1) {
    int N = 30;
    for(int k = 0; k <= N; k++) {
      float qq[] = {q[0]*(N-k)/N + q0[0]*k/N,
        q[1]*(N-k)/N + q0[1]*k/N,
        q[2]*(N-k)/N + q0[2]*k/N,
        q[3]*(N-k)/N + q0[3]*k/N};

        servomotor_write(qq);
        delay(500/N);
    }
    memcpy(q, q0, sizeof(float) * 4);
    p("Solution:", q, 1, 4);
  }
  else
    Serial.println("Could not reach target");
}

#ifdef ROMEO
void servomotor_write(float *q) {
  servo[0].write(-q[0] * 180 / PI + 90);
  servo[1].write(q[1] * 180 / PI + 90);
  servo[2].write(-q[2] * 180 / PI + 90);
  servo[3].write(-q[3] * 180 / PI + 85);
  gripper.write(160);
}
#endif

#ifdef JULIET
void servomotor_write(float *q) {
  servo[0].write(-q[0] * 180 / PI + 105);
  servo[1].write(q[1] * 180 / PI + 90);
  servo[2].write(q[2] * 180 / PI + 90);
  servo[3].write(q[3] * 180 / PI + 110);
  gripper.write(70);
}
#endif

void p(const char *str, float *_mat, size_t m, size_t n) {
  float (*mat)[n] = (float (*)[n]) _mat;
  int i, j;

  Serial.println();
  Serial.println(str);
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      Serial.print("\t");
      Serial.print(mat[i][j], DEC);
    }
    Serial.println();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  randomSeed(analogRead(A0));

  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_SENSOR, INPUT_PULLUP);

  robot_setup();

  matrix.begin(0x70);


}

void sing_song() {
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    int noteDuration = 1000 / noteDurations[thisNote];
    tone(PIN_BUZZ, melody[thisNote], noteDuration);

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    noTone(PIN_BUZZ);
  }
}

void move_new_target() {

  float r = random(15, 30) * 1e-2,
        th = random(10, 75) * PI / 180,
        x0 = 25e-2,
        z0 = 35e-2,
        y0 = 10e-2;

  float *q = _q;
//  float s[4] = {.4 - r * sin(th), 0, 0.4 - r * cos(th), 0}; // XZ plane
  float s[4] = {x0, y0 - r*sin(th), z0 - r*cos(th), 0}; // YZ plane
  p("Target:", s, 1, 4);

  robot_write(s, q);

  p( "pos:", q, 1, 4);
}

void loop() {
  
  float *q = _q;
  float s[] = {0.25, 0, 0.2, 0}; // home
  robot_write(s, q);

  matrix.print(0);
  matrix.writeDisplay();

  while (digitalRead(PIN_SENSOR) == HIGH);
  tone(PIN_BUZZ, NOTE_G3, 500);
  delay(500);

  const float DURATION = 10000;
  const int REPETITIONS = 10;

  float cumtimer = 0;

  for(int tries = 0; tries < REPETITIONS; tries++) {

    long int t0 = millis();
  
    move_new_target();
    while (digitalRead(PIN_SENSOR) == HIGH && millis() - t0 < DURATION) {
      matrix.print( 1e-3*(millis() - t0) );
      matrix.writeDisplay();
      delay(100);
    }

    cumtimer += millis() - t0;
    
    tone(PIN_BUZZ, NOTE_G3, 500);
    delay(500);
  }
  
  Serial.print("Score: ");
  Serial.println((REPETITIONS * DURATION - cumtimer) * 100.0 / (REPETITIONS * DURATION));

  matrix.print((REPETITIONS * DURATION - cumtimer) * 100.0 / (REPETITIONS * DURATION));
  matrix.writeDisplay();

  sing_song();
  delay(1000);
}

