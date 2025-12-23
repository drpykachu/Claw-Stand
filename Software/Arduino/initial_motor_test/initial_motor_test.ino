#define dirPin 23
#define stepPin 22
#define M0 4
#define M1 16
#define M2 17

// full-step motor = 2048 steps/rev; using 1/32 microstepping → 65536 steps/rev
const float stepsPerRev = 2048.0 * 32.0;

// angle list
float angles[] = {
  112.5635019,112.0942875,111.6184066,111.1359074,110.6468443,
  110.151278,109.6492757,109.1409112,108.6262652,108.1054253,
  107.5784864,107.0455506,106.5067271,105.9621328,105.4118921,
  104.8561369,104.2950066,103.7286483,103.1572166,102.5808736,
  101.9997891,101.4141399,100.8241101,100.2298911,99.63168098,
  99.02968458,98.42411316,97.8151842,97.20312106,96.58815267,
  95.97051322,95.35044175,94.72818181,94.10398098,93.47809052,
  92.85076488,92.22226121,91.59283895,90.96275928,90.33228466,
  89.7016783,89.07120368,88.44112401,87.81170175,87.18319808,
  86.55587243,85.92998198,85.30578115,84.6835212,84.06344974,
  83.44581029,82.8308419,82.21877875,81.60984979,81.00427838,
  80.40228197,79.80407185,79.20985285,78.61982311,78.03417387,
  77.45308932,76.87674639,76.3053147,75.73895639,75.17782607,
  74.62207083,74.07183013,73.52723588,72.98841239,72.45547652,
  71.92853764,71.4076978,70.89305179,70.38468728,69.88268495,
  69.38711865,68.89805557,68.41555637,67.93967545,67.47046103,
  67.47046103,68.3500728,69.88948033,71.93386758,74.33021622,
  76.95764294,79.73411328,82.60827156,85.54658633,88.5226412,
  91.51132176,94.48737663,97.42569139,100.2998497,103.07632,
  105.7037467,108.1000954,110.1444826,111.6838902,112.5635019
};
int angleCount = sizeof(angles) / sizeof(float);

long currentPosition = 0; // in microsteps

long angleToSteps(float deg) {
  return (long)(deg * (stepsPerRev / 360.0));
}

void stepOnce() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(7);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(7);
}

void goToAngle(float targetAngle) {
  long targetSteps = angleToSteps(targetAngle);
  long delta = targetSteps - currentPosition;

  // direction
  digitalWrite(dirPin, delta > 0 ? HIGH : LOW);

  long steps = abs(delta);
  for (long i = 0; i < steps; i++) {
    stepOnce();
  }

  currentPosition = targetSteps;
}

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // 1/32 microstepping
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW.); 
}

void loop() {
  for (int i = 0; i < angleCount; i++) {
    goToAngle(angles[i]);
  }
}
