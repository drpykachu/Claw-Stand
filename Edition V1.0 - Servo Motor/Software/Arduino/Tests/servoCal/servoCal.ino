 #include <Servo.h>

Servo servo_b_1;  // create servo object to control a servo
Servo servo_m_1;  // create servo object to control a servo
Servo servo_t_1;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position
int bot = 50;
int top = 168;
int waiter = 25;
int midpot = A0;
int mid_val;


// bot range: : 
// mid range: 4-65: 0-90
// top range: 110-28: 0-90

int bot_check;
int mid_check;
int top_check;


void setup() {
  servo_m_1.attach(11);  // middle servo
  Serial.begin(9600);
//  mid_val = map(analogRead(midpot), 0, 1023, top, bot);
//
//  mid_check = mid_val;

//  for (pos = bot; pos <= top; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    servo_m_1.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(waiter);                       // waits waiterms for the servo to reach the position
//  }
//  for (pos = top; pos >= bot; pos -= 1) { // goes from 180 degrees to 0 degrees
//    servo_m_1.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(waiter);                       // waits waiterms for the servo to reach the position
//  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {

    servo_m_1.write(99);              // tell servo to go to position in variable 'pos'
    delay(100);                       // waits waiterms for the servo to reach the position
    
//  for (pos = bot- 3; pos <= top; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    servo_m_1.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(waiter);                       // waits waiterms for the servo to reach the position
//  }
//  Serial.print("Top");
//  Serial.println(top);
//  delay(1000);
//  for (pos = top + 3; pos >= bot; pos -= 1) { // goes from 180 degrees to 0 degrees
//    servo_m_1.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(waiter);                       // waits waiterms for the servo to reach the position
//  }
//  Serial.print("bottom");
//  Serial.println(bot);
//  delay(1000);
}
