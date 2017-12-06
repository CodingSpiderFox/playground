#include <SoftwareSerial.h>

//SoftwareSerial swSer(RX, TX, false);

void setup()
{
  Serial.begin(115200);
  //swSer.begin(115200);  
}
 
void loop()
{
  String lastStr;
  while (Serial.available() > 0) {
    lastStr = Serial.readString();
    Serial.println(lastStr);
  }
  delay(1000);
  //Serial.println("test");
}
