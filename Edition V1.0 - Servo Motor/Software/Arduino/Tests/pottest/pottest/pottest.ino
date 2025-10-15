#define pot1 A1
#define pot2 A2
#define pot3 A3


void setup() 
{
  Serial.begin(9600);
}

void loop()
{
  int data1 = analogRead(pot1);
  int data2 = analogRead(pot2);
  int data3 = analogRead(pot3);
  
  // int percentage1 = map(data1, 0, 860, 0, 100);
  // int percentage2 = map(data2, 0, 860, 0, 100);
  // int percentage3 = map(data3, 0, 860, 0, 100);

  int percentage1 = data1;
  int percentage2 = data2;
  int percentage3 = data3;

  Serial.print(percentage1);
  Serial.print("  ");
  Serial.print(percentage2);
  Serial.print("  ");
  Serial.print(percentage3);
  Serial.println(" ");
  delay(100);
}
