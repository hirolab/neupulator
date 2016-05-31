#include "LinearAlgebra.h"
#include "Robotics.h"
#include <Servo.h>
#include "pitches.h"

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

//  servo[0].write(100);
//  servo[1].write(90);
//  servo[2].write(90);
//  servo[3].write(90);
//  gripper.write(60);
}

void robot_write(float *s, float *q) {

  const float CONTR = 0.8;
  float q0[4] = {q[0]*CONTR, q[1]*CONTR, q[2]*CONTR, q[3]*CONTR};
  float error = ik(s, 4, q0);

  if (error < 1e-1) {
    memcpy(q, q0, sizeof(float) * 4);

    servo[0].write(-q[0] * 180 / PI + 100);
    servo[1].write(q[1] * 180 / PI + 90);
    servo[2].write(q[2] * 180 / PI + 90);
    servo[3].write(q[3] * 180 / PI + 90);

    p("Solution:", q, 1, 4);
  }
  else
    Serial.println("Could not reach target");
}

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
  randomSeed(600);

  pinMode(8, OUTPUT);

  robot_setup();
}

void loop() {
  // put your main code here, to run repeatedly:

  const float DURATION = 5;
  const int REPETITIONS = 5;

  static float score = -DURATION;
  static int tries = 0;
  
  float t = millis() / 1000.0;
  static float tlast = -DURATION;

  if (t - tlast > DURATION || digitalRead(48) == LOW) {

    float r = random(15, 25)*1e-2,
          th = random(30, 70)*PI/180;
          
    float x = 35e-2 - r*cos(th),
          z = 35e-2 - r*sin(th);

    float q[4] = {0, 0, 0, 0};
    float s[4] = {x, 0, z, 0};
    p("Target:", s, 1, 4);
    digitalWrite(8, HIGH);
    delay(500);
    robot_write(s, q);
    digitalWrite(8, LOW);

    score += min(t - tlast, DURATION);
    tries++;

    tlast = t;
  }

  if(tries >= REPETITIONS) {
    Serial.print("Score: ");
    Serial.println(score);
    Serial.println((REPETITIONS*DURATION - score) * 100.0/(REPETITIONS*DURATION));

    for (int thisNote = 0; thisNote < 8; thisNote++) {
  
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / noteDurations[thisNote];
      tone(8, melody[thisNote], noteDuration);
  
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(8);
    }
    while(true);
  }

}
