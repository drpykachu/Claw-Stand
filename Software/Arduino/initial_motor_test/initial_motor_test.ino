#include <Arduino.h>

const int motorPins[4] = {2, 4, 16, 17};
const uint8_t seq[8] = {0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001};

void setup() {
  for (int i=0;i<4;i++) pinMode(motorPins[i], OUTPUT);
}

void loop() {
  for (int step=0; step<8; step++) {
    uint8_t out = seq[step];
    for (int i=0;i<4;i++) digitalWrite(motorPins[i], (out>>i)&1);
    delayMicroseconds(4000); // try 800, 600, 400, etc.
  }
}
