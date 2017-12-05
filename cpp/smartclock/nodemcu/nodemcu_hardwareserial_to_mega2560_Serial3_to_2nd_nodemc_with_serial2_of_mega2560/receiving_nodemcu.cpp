#include <SoftwareSerial.h>

//SoftwareSerial swSer(RX, TX, false);

void setup()
{
  Serial.begin(115200);
  //swSer.begin(115200);  
}
 
void loop()
{
  byte lastByte;
  while (Serial.available() > 0) {
    lastByte = Serial.read();
    Serial.println(lastByte);
  }
  delay(1000);
  Serial.println("test");
}
