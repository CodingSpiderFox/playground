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


#include <pcf8574_esp.h>
#include <Wire.h>
TwoWire testWire;
// Initialize a PCF8574 at I2C-address 0x20, using GPIO5, GPIO4 and testWire for the I2C-bus
PCF857x pcf8574(0x20, &testWire);
 
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
  testWire.begin();
  
  pinMode(PIN_INT, INPUT_PULLUP);

  pcf8574.begin( 0xFF); 
  pcf8574.resetInterruptPin();
  pinMode(ledPin, OUTPUT);   // LED pin as output.  
}

bool CheckKey(byte key, byte num){ //0, 1, 2, 3
  return key & (1 << num);
}

void handleInterrupt () {
   if( digitalRead(PIN_INT)==LOW ){


    if(pcf8574.read(5)==LOW) Serial.println("pin5");
    else Serial.println("!pin5");
  
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
