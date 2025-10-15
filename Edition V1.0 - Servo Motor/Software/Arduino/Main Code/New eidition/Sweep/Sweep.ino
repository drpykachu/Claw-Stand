
/*************************************************** 
THE CLAW
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// Servo Motor Bounds
#define BOTTOM_MIN 1020
#define BOTTOM_MAX 2217
#define MIDDLE_MIN 1020
#define MIDDLE_MAX 2217
#define TOP_MIN 1050
#define TOP_MAX 1850

int BOTTOM_MID_VAL = (BOTTOM_MIN + BOTTOM_MAX) / 2;
int MIDDLE_MID_VAL = (MIDDLE_MIN + MIDDLE_MAX) / 2;
int TOP_MID_VAL = (TOP_MIN + TOP_MAX) / 2;


#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates




// Servo Position Initializations
int val_BOTTOM;
int val_MIDDLE;
int val_TOP;

int val_checker = 0;
int val_checker_new = 1;

// Physical System Parameters
float T = 105;
float M = 124;
float E = 41;
float P = 82;
float offset_R = 31.75;

float angles[] = { 50, 50, 50 };
float positioner[] = { 50, 50, 50 };
int trip = 1;
int degree_max = 135;
int degree_min = 45;
int height = 240;
int radius = 75;
float speeder = 5;
int rate = 5;

//////////////////////////////////// Setup ////////////////////////////////////
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(1000);

  BOTTOM_check();

  MIDDLE_check();

  TOP_check();

  delay(10);
}
/////////////////////////////////////////////////////////////////////////////////////////




void loop() {


}


void BOTTOM_check(){
  Serial.println(BOTTOM_MID_VAL);
  Serial.println((BOTTOM_MIN+BOTTOM_MID_VAL)/2);
  Serial.println((BOTTOM_MAX+BOTTOM_MID_VAL)/2);

  for (int val = BOTTOM_MID_VAL; val >= (BOTTOM_MIN+BOTTOM_MID_VAL)/2 ; val -= rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(0 + 3 * i, val);
    }
    delay(speeder);
  }

  delay(1000);
  for (int val = (BOTTOM_MIN+BOTTOM_MID_VAL)/2; val <= (BOTTOM_MAX+BOTTOM_MID_VAL)/2 ; val += rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(0 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);

  
  for (int val = (BOTTOM_MAX+BOTTOM_MID_VAL)/2; val >= BOTTOM_MID_VAL ; val -= rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(0 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);
}

void MIDDLE_check(){

  for (int val = MIDDLE_MID_VAL; val >= MIDDLE_MIN ; val -= rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(1 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);
  
  for (int val = MIDDLE_MIN; val <= MIDDLE_MID_VAL ; val += rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(1 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);


}


void TOP_check(){

  for (int val = TOP_MID_VAL; val >= TOP_MIN ; val -= rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(2 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);

  for (int val = TOP_MIN; val <= TOP_MAX ; val += rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(2 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);

  for (int val = TOP_MAX; val >= TOP_MIN ; val -= rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(2 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);
  
  for (int val = TOP_MIN; val <= TOP_MID_VAL ; val += rate) {
      for (int i = 0; i < 5; i++) {
      pwm.writeMicroseconds(2 + 3 * i, val);
    }
    delay(speeder);
  }
  delay(1000);
}


