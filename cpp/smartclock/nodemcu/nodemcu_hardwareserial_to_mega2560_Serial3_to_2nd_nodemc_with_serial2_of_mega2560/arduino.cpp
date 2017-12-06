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
  String inStr;
  while (Serial3.available() > 0) {
    inStr = Serial3.readString();
    for (int i = 0; i < inStr.length(); i++) {
      Serial.write(inStr[i]);
      Serial2.write(inStr[i]);
    }
  }
}

void loop()
{
  readSwSer();
  delay(1000);
}
