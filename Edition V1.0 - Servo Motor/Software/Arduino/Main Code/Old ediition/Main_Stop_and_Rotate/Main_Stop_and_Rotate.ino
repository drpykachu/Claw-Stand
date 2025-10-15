
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
#define BOTTOM_MIN  1020
#define BOTTOM_MAX  2217
#define MIDDLE_MIN  1020
#define MIDDLE_MAX  2217
#define TOP_MIN  1050
#define TOP_MAX  1850

int BOTTOM_MID_VAL = (BOTTOM_MIN  + BOTTOM_MAX) / 2;
int MIDDLE_MID_VAL = (MIDDLE_MIN  + MIDDLE_MAX) / 2;
int TOP_MID_VAL = (TOP_MIN  + TOP_MAX) / 2;


#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


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

float angles[] = {50, 50, 50};
float positioner[] = {50, 50, 50};
int trip = 1;
int degree_max = 130;
int degree_min = 60;
int height = 250;
int radius = 65;

int finger_iteration = 0;
int finger_offset = 12; // ~90/5
int num_finger = 2;



// Setup
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

//////////////////////////////////// Main Loop Function /////////////////////////////////////
float degree_1 = 0*finger_offset + degree_min;
float degree_2 = 1*finger_offset + degree_min;
float degree_3 = 2*finger_offset + degree_min;
float degree_4 = 3*finger_offset + degree_min;
float degree_5 = 4*finger_offset + degree_min;


float height_high = height ;
float height_low = height-20;



float speeder = 1; 
float rate = 2;

void loop() {


  for(int i = degree_min; i< degree_max; i++){
    circle(radius,height_high,i,offset_R);
    tracker(); 
    for(int ii = 0; ii< 5; ii++){
      pwm.writeMicroseconds(0+ii, angles[0]);
      pwm.writeMicroseconds(5+ii, angles[1]);
      pwm.writeMicroseconds(10+ii, angles[2]);
    }
  delay(speeder);
  }

  delay(1000);

  for(int i = 0; i< 5; i++){
    for(int ii = degree_max; ii> degree_min; ii--){
      circle(radius,height_low,ii,offset_R);
      tracker(); 
      pwm.writeMicroseconds(0+3*i, angles[0]);
      pwm.writeMicroseconds(1+3*+i, angles[1]);
      pwm.writeMicroseconds(2+3*+i, angles[2]);
      delay(speeder);
    }
    circle(radius,height_high,degree_min,offset_R);
    tracker(); 
      pwm.writeMicroseconds(0+3*i, angles[0]);
      pwm.writeMicroseconds(1+3*+i, angles[1]);
      pwm.writeMicroseconds(2+3*+i, angles[2]);
    
    delay(100);
  }

  delay(1000);
}
/////////////////////////////////////////////////////////////////////////////////////////

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

void tracker() {

  float  x = positioner[0];
  float  y = positioner[1];
  float  z = positioner[2];
  float temp_theta_b;
  int flipper_2 = 0;

  float theta_b = atan(z / x);
  if (theta_b  <= 0 ) {

    theta_b = -theta_b;
    flipper_2 = 1;
  }

  float c1 = (z - E * sin(theta_b)) / M;
  float c2 = (y - P) / M;
  float A = 2 * T * c1 / M;
  float B = -2 * T * c2 / M;

  float phi = asin((pow((T / M), 2) + pow(c1, 2) + pow(c2, 2) - 1) / (  pow((pow(A, 2) + pow(B, 2)), 0.5))) - atan(B / A);
  float theta_m = asin((z - E * sin(theta_b) - T * sin(phi)) / M);
  float theta_t = phi + theta_m;


  theta_b = theta_b * 180.0 / 3.1415 ;
  theta_m = theta_m * 180.0 / 3.1415;
  theta_t = theta_t * 180.0 / 3.1415;

  if (flipper_2 == 1) {
    theta_b = -theta_b;
    theta_b = 90 +  abs(90 + theta_b);
  }

  angles[0] = liner(theta_b, 0, 90, BOTTOM_MIN, BOTTOM_MID_VAL);
  angles[1] = liner(theta_m, 0, 90, MIDDLE_MIN, MIDDLE_MID_VAL);
  angles[2] = liner(theta_t, 90, 180, MIDDLE_MID_VAL, TOP_MIN);
}

/////////////////////////////////////////////////////////////////////////////////////////