void setup()
{
  randomSeed(analogRead(0));
  
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial.begin(115200);
}

void readSwSer() {
  Serial.println("Str");
  Serial.println(Serial3.available());
  byte incomingByte;
  while (Serial3.available() > 0) {
    incomingByte = Serial3.read();
    Serial.println(incomingByte);
    Serial2.write(incomingByte);
    
  }
}

void loop()
{
  readSwSer();
  delay(1000);
}
