
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
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
int degree_max = 100;
int degree_min = 75;
int height = 235;
int radius = 55;

// Setup
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(500);
}

int returner = -10;
int forward = 1;
int new_height = height - 25;

int p_off = (degree_max - degree_min)/6;
int p_break = p_off/5;

float speeder = 100;
int rate_1 = 1;
int degree_1 = degree_min;
int height_1 = height;

int rate_2 = 1;
int degree_2 = p_off*1 + p_break + degree_min;
int height_2 = height;

int rate_3 = 1;
int degree_3 = p_off*2 + p_break + degree_min;
int height_3 = height;

int rate_4 = 1;
int degree_4 = p_off*3 + p_break + degree_min;
int height_4 = height;

int rate_5 = 1;
int degree_5 = p_off*4 + p_break + degree_min;
int height_5 = height;



void loop() {

  // Finger one logic
  if(degree_1 <= degree_min){
    rate_1 = forward;
    height_1 = height;
  }
  if(degree_1 >= degree_max){
    rate_1 = returner;
    height_1 = new_height;
  }
  degree_1 = degree_1 + rate_1;
  circle(radius, height_1, degree_1, offset_R);
  tracker(degree_1);
  pwm.writeMicroseconds(0, angles[0]);
  pwm.writeMicroseconds(1, angles[1]);
  pwm.writeMicroseconds(2, angles[2]);

  // Finger two logic
  if(degree_2 <= degree_min){
    rate_2 = forward;
    height_2 = height;
  }
  if(degree_2 >= degree_max){
    rate_2 = returner;
    height_2 = new_height;
  }
  degree_2 = degree_2 + rate_2;
  circle(radius, height_2, degree_2, offset_R);
  tracker(degree_2);
  pwm.writeMicroseconds(0 + 3*3, angles[0]);
  pwm.writeMicroseconds(1 + 3*3, angles[1]);
  pwm.writeMicroseconds(2 + 3*3, angles[2]);


  // Finger three logic
  if(degree_3 <= degree_min){
    rate_3 = forward;
    height_3 = height;
  }
  if(degree_3 >= degree_max){
    rate_3 = returner;
    height_3 = new_height;
  }
  degree_3 = degree_3 + rate_3;
  circle(radius, height_3, degree_3, offset_R);
  tracker(degree_3);
  pwm.writeMicroseconds(0 + 3*1, angles[0]);
  pwm.writeMicroseconds(1 + 3*1, angles[1]);
  pwm.writeMicroseconds(15, angles[2]);
  
  
  // Finger four logic
  if(degree_4 <= degree_min){
    rate_4 = forward;
    height_4 = height;
  }
  if(degree_4 >= degree_max){
    rate_4 = returner;
    height_4 = new_height;
  }
  degree_4 = degree_4 + rate_4;
  circle(radius, height_4, degree_4, offset_R);
  tracker(degree_4);
  pwm.writeMicroseconds(0 + 3*4, angles[0]);
  pwm.writeMicroseconds(1 + 3*4, angles[1]);
  pwm.writeMicroseconds(2 + 3*4, angles[2]);
  
  // Finger five logic
  if(degree_5 <= degree_min){
    rate_5 = forward;
    height_5 = height;
  }
  if(degree_5 >= degree_max){
    rate_5 = returner;
    height_5 = new_height;
  }
  degree_5 = degree_5 + rate_5;
  circle(radius, height_5, degree_5, offset_R);
  tracker(degree_5);
  pwm.writeMicroseconds(0 + 3*2, angles[0]);
  pwm.writeMicroseconds(1 + 3*2, angles[1]);
  pwm.writeMicroseconds(2 + 3*2, angles[2]);
  
  
  delay(speeder);

}



//////////////////////////////////// Liner Function /////////////////////////////////////
float liner(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////// Cirle Function /////////////////////////////////////

float circle(float r, float h, float degree, float offset_R) {

  positioner[0] = r * cos(degree * 3.145 / 180);
  positioner[1] = r * sin(degree * 3.145 / 180) - offset_R;
  positioner[2] = h;
}

///////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////// Tracker Function ///////////////////////////////////

void tracker(float degree) {

  float x = positioner[0];
  float y = positioner[1];
  float z = positioner[2];
  float temp_theta_b;
  int flipper_2 = 0;

  float theta_b = atan(z / x);
  if (theta_b <= 0) {

    theta_b = -theta_b;
    flipper_2 = 1;
  }

  float c1 = (z - E * sin(theta_b)) / M;
  float c2 = (y - P) / M;
  float A = 2 * T * c1 / M;
  float B = -2 * T * c2 / M;

  float phi = asin((pow((T / M), 2) + pow(c1, 2) + pow(c2, 2) - 1) / (pow((pow(A, 2) + pow(B, 2)), 0.5))) - atan(B / A);
  float theta_m = asin((z - E * sin(theta_b) - T * sin(phi)) / M);
  float theta_t = phi + theta_m;


  theta_b = theta_b * 180.0 / 3.1415;
  theta_m = theta_m * 180.0 / 3.1415;
  theta_t = theta_t * 180.0 / 3.1415;

  if (flipper_2 == 1) {
    theta_b = -theta_b;
    theta_b = 90 + abs(90 + theta_b);
  }

  angles[0] = liner(theta_b, 0, 90, BOTTOM_MIN, BOTTOM_MID_VAL);
  angles[1] = liner(theta_m, 0, 90, MIDDLE_MIN, MIDDLE_MID_VAL);
  angles[2] = liner(theta_t, 90, 180, MIDDLE_MID_VAL, TOP_MIN);
}

/////////////////////////////////////////////////////////////////////////////////////////