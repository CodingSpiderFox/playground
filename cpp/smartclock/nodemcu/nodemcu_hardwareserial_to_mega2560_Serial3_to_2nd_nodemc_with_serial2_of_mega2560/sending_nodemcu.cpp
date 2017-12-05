#include <SoftwareSerial.h>

//SoftwareSerial swSer(RX, TX, false);

void setup()
{
  Serial.begin(115200);
  //swSer.begin(115200);  
}

void printSerial(String str) {
  Serial.print(str);
  
}

void printlnSerial(String str) {
  printSerial(str + "\r\n");
}

void handleInterrupt () {
    printSerial("GPIO Expander 1 ");
    delay(1050);
}
 
void loop()
{
  handleInterrupt();
}
