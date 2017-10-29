/*
 
NonBlockingBreathingLed 0.1
by Luca Soltoggio - 2015
edited by CodingSpiderFox 2017-10-28
12 May 2015
http://www.arduinoelettronica.com/
https://arduinoelectronics.wordpress.com/
http://minibianpi.wodpress.com/
 
Use a exp + sin function to recreate a
non-blocking breathing led effect
 
Released under GPL v.2 license




*/
#include <SoftwareSerial.h>

SoftwareSerial swSer(D0, D3, false);


#include <pcf8574_esp.h>
#include <Wire.h>
TwoWire testWire;
// Initialize a PCF8574 at I2C-address 0x20, using GPIO5, GPIO4 and testWire for the I2C-bus
PCF857x pcf8574(0x20, &testWire);
PCF857x pcf8574_2(0x27, &testWire);
 
#define BRIGHT    300     //max led intensity (1-500)
#define INHALE    1250    //Inhalation time in milliseconds.
#define PULSE     INHALE*1000/BRIGHT
#define REST      1000    //Rest Between Inhalations.

#define PIN_INT D5
#define PIN_SDA D1
#define PIN_SCL D2

#include <math.h>
#define ledPin D8


int i=0;
int breathe_delay = 15;   // delay between loops
unsigned long breathe_time = millis();
void setup()
{
  Serial.begin(115200);
  Wire.pins(PIN_SDA, PIN_SCL);//SDA - D1, SCL - D2
  Wire.begin();
  testWire.begin();
  swSer.begin(115200);  
  pinMode(PIN_INT, INPUT_PULLUP);

  pcf8574.begin( 0xFF); 
  pcf8574.resetInterruptPin();
  pcf8574_2.begin( 0xFF); 
  pcf8574_2.resetInterruptPin();
  pinMode(ledPin, OUTPUT);   // LED pin as output.  
}

bool CheckKey(byte key, byte num){ //0, 1, 2, 3
  return key & (1 << num);
}

void printSerial(String str) {
  Serial.print(str);
  swSer.print(str + '\0');
  swSer.flush();
}

void printlnSerial(String str) {
  printSerial(str + "\r\n");
}

void handleInterrupt () {
   if( digitalRead(PIN_INT)==LOW ){

    byte data1 = pcf8574.read8();
    byte data2 = pcf8574_2.read8();
    byte data = 0;
    
    if(data1 < 255) {
      data = data1;
      printSerial("GPIO Expander 1 ");
    }
    if(data2 < 255) {
      data = data2;
      printSerial("GPIO Expander 2 ");
    }


    if(data == 254) {
      printlnSerial("Button 0");
    }
    if(data == 253) {
      printlnSerial("Button 1");
    }
    if(data == 251) {
      printlnSerial("Button 2");
    }
    if(data == 247) {
      printlnSerial("Button 3");
    }
    if(data == 239) {
      printlnSerial("Button 4");
    }
    if(data == 223) {
      printlnSerial("Button 5");
    }
    if(data == 191) {
      printlnSerial("Button 6");
    }
    if(data == 127) {
      printlnSerial("Button 7");
    }
        
    delay(150);
    /*if(pcf8574.read(5)==LOW) Serial.println("pin5");
    else Serial.println("!pin5");
    */
//    byte b = pcf8574.read8();
//    Serial.println( "INT: " + String(b));

//    byte keys = ((~b)) & 0x0F;
    
//    delay(0);
  }
}
 
void loop()
{
  handleInterrupt();
  
  // nonBlockingBreath();  // call the nonblocking function
  // yourOtherCodeHere();
}
 
void nonBlockingBreath()
{
  if( (breathe_time + breathe_delay) < millis() ){
    breathe_time = millis();
    float val = (exp(sin(i/2000.0*PI*10)) - 0.36787944)*108.0; 
    // this is the math function recreating the effect
    analogWrite(ledPin, val);  // PWM
    i=i+1;
    handleInterrupt();
  }
  handleInterrupt();
}
