#include "LinearAlgebra.h"
#include "Robotics.h"
#include <Servo.h>
#include "pitches.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_7segment matrix = Adafruit_7segment();

//#define ROMEO
#define JULIET

#define PIN_SENSOR 11
#define PIN_BUZZ 12

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

  servo[0].attach(3);
  servo[1].attach(4);
  servo[2].attach(5);
  servo[3].attach(6);
  gripper.attach(7);

  servo[0].write(90);
  servo[1].write(90);
  servo[2].write(90);
  servo[3].write(85);
  gripper.write(160);
}

void robot_write(float *s, float *q) {

  const float CONTR = 0.8;
  float q0[4] = {q[0]*CONTR, q[1]*CONTR, q[2]*CONTR, q[3]*CONTR};
  float error = ik(s, 4, q0, fk_juliet);

  if (error < 1e-1) {
    memcpy(q, q0, sizeof(float) * 4);
    servomotor_write(q);
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
  servo[3].write(q[3] * 180 / PI + 90);
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
  randomSeed(100);

  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_SENSOR, INPUT_PULLUP);

  robot_setup();

//  servo[0].write(120);
//  servo[1].write(120);
//  servo[2].write(120);
//  servo[3].write(120);
//  gripper.write(55);
//
//  while (true);

  float q[] = {0, 0, 0, 0};
  float s[] = {0.2, 0, 0.3, 0};
  robot_write(s, q);

  matrix.begin(0x70);

  matrix.print(0);
  matrix.writeDisplay();

  while (digitalRead(PIN_SENSOR) == HIGH);
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

  float r = random(15, 25) * 1e-2,
        th = random(10, 90) * PI / 180;

  float q[4] = {0.0, 0.0, 0.0, 0.0};
  float s[4] = {.4 - r * sin(th), 0, 0.4 - r * cos(th), 0};
  p("Target:", s, 1, 4);

  digitalWrite(PIN_BUZZ, HIGH);
  delay(500);
  robot_write(s, q);
  digitalWrite(PIN_BUZZ, LOW);

  p( "pos:", q, 1, 4);
}

void loop() {
  // put your main code here, to run repeatedly:

  const float DURATION = 10;
  const int REPETITIONS = 10;

  static float cumtimer = 0;
  static int tries = 0;

  float t = millis() / 1000.0;
  static float tlast = t;

  matrix.print(DURATION - (t - tlast));
  matrix.writeDisplay();

  if (t - tlast > DURATION || digitalRead(PIN_SENSOR) == LOW) {

    move_new_target();

    cumtimer += min(t - tlast, DURATION);
    tries++;

    matrix.print(t - tlast);
    matrix.writeDisplay();

    tlast = millis() / 1000.0;
  }

  if (tries >= REPETITIONS) {
    Serial.print("Score: ");
    Serial.println((REPETITIONS * DURATION - cumtimer) * 100.0 / (REPETITIONS * DURATION));

    matrix.print((REPETITIONS * DURATION - cumtimer) * 100.0 / (REPETITIONS * DURATION));
    matrix.writeDisplay();

    sing_song();

    while (true);
  }
}

