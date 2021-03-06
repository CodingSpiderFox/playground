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

SoftwareSerial swSer(D0, D2, false);

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
#define PIN_SDA D7
#define PIN_SCL D8

#include <math.h>
#define ledPin D1


int i=0;
int breathe_delay = 15;   // delay between loops
unsigned long breathe_time = millis();
void setup()
{
  swSer.begin(115200);
  Serial.begin(115200);
  Wire.pins(PIN_SDA, PIN_SCL);//SDA - D1, SCL - D2
  Wire.begin();
  
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
    delay(50);
  
    byte b = pcf8574.read8();
    Serial.println( "INT: " + String(b));
    swSer.println("INT: " + String(b)+ '\0');
    byte keys = ((~b)) & 0x0F;
    
    if( CheckKey(keys, 8) ){
      Serial.println( "KEY 7");
      delay(0);
      
    }
  }
}
 
void loop()
{
  swSer.flush();
  swSer.println("f\r\n");
  delay(7000);
  //nonBlockingBreath();  // call the nonblocking function
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
