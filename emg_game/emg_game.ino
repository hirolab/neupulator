long int minval, maxval;
float ref, cumerror = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(analogRead(A3));

  while(millis() < 5000) {
    int volt = analogRead(A0);
    minval = min(minval, volt);
    maxval = max(maxval, volt);
    delay(10);
  }
  ref = minval/2 + maxval/2;

//  ref = analogRead(A0);
//  minval = ref;
//  maxval = ref;
}

void loop() {
  // put your main code here, to run repeatedly:

  static float speed = 1;
  speed += 2e-2;
  ref += speed*random(-100, 100) * (maxval - minval)/2e3;
  ref = constrain(ref, minval, maxval);

  float volt = 0;
  for(int n = 0; n < 10; n++) {
    volt += analogRead(A0) / 10.0;
    delay(10);
  }

  float error = fabs(volt - ref);
  if(error < 10)
    error = 0;
  cumerror += 0.1*error;
  if(cumerror > 5*(maxval - minval) && millis() > 5000)
    while(true) {
      static long int score = millis();
      Serial.print(String(score) + '\t' + score + '\t' + score + '\t' + score + '\n');
    }

  Serial.print(String(volt) + '\t' + maxval + '\t' + ref + '\t' + minval +  '\n');
}
