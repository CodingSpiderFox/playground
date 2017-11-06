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
#include <math.h>
#include <pcf8574_esp.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "TimerObject.h"

TimerObject *timer1 = new TimerObject(60000);
TimerObject *timer2 = new TimerObject(500);
TimerObject *timer3 = new TimerObject(500);
TimerObject *everySec = new TimerObject(1000);

//MQTT related
const char* ssid = "berlin.freifunk.net";
const char* password = "";
const char* mqtt_server = "10.230.145.13";
const char* mqtt_userName = "user";
const char* mqtt_password = "user";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
int timeout = 200;

//NTP related
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
unsigned int localNTPPort = 2390;      // local port to listen for UDP packets
bool sentInitialRequest;
unsigned long unixTime = 0;

//GPIO related
TwoWire testWire;
// Initialize a PCF8574 at I2C-address 0x20, using GPIO5, GPIO4 and testWire for the I2C-bus
PCF857x pcf8574(0x20, &testWire);
PCF857x pcf8574_2(0x27, &testWire);

SoftwareSerial swSer(D0, D3, false);
 
#define BRIGHT    300     //max led intensity (1-500)
#define INHALE    1250    //Inhalation time in milliseconds.
#define PULSE     INHALE*1000/BRIGHT
#define REST      1000    //Rest Between Inhalations.
#define PIN_INT D5
#define PIN_SDA D1
#define PIN_SCL D2
#define MOTIONSENSOR D4
#define ledPin D8

int i=0;
int breathe_delay = 15, message_delay = 200;   // delay between loops
unsigned long breathe_time = millis();
unsigned long message_time = millis();

void sendMQTTMessage() {
  ++value;
  snprintf(msg, 175, "hello worldAAAAsaaawo#%ld", value);
  
  Serial.print("Publish message: ");
  Serial.println(msg);
  client.publish("outTopic", msg);
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 01000
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void processNTPResponse() {
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    unixTime = epoch;
    printSerial("t:" + unixTime);
    /* print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    */
  }
}

void getNTPTime() {
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
}

void setup_ntp() {
  udp.begin(localNTPPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
}

void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); 
}

void reconnect_mqtt() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("SmartClockControl", mqtt_userName, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outSCC", "hello world");
      // ... and resubscribe
      client.subscribe("inSCC");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void everySecFunc() {
  unixTime++;
  Serial.println(unixTime);
}

void setup()
{
  Serial.begin(115200);
  swSer.begin(115200);  

  Wire.pins(PIN_SDA, PIN_SCL);//SDA - D1, SCL - D2
  Wire.begin();
  testWire.begin();
  pinMode(PIN_INT, INPUT_PULLUP);
  pcf8574.begin( 0xFF); 
  pcf8574.resetInterruptPin();
  pcf8574_2.begin( 0xFF); 
  pcf8574_2.resetInterruptPin();
  
  pinMode(ledPin, OUTPUT);   // LED pin as output.
  pinMode(MOTIONSENSOR, INPUT);
  
  setup_wifi();
  setup_ntp();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("Start task1");
  timer1->setOnTimer(&getNTPTime);
  timer1->Start();

  timer2->setOnTimer(&processNTPResponse);
  timer2->Start();

  timer3->setOnTimer(&sendMQTTMessage);
  timer3->Start();

  everySec->setOnTimer(&everySecFunc);
  everySec->Start();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  /* Switch on the LED if an 1 was received as first character
  Serial.print((char)payload[0]);
  if ((char)payload[0] == '1') {
    digitalWrite(LEDPIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
                      // but actually the LED is on; this is because
                      // it is acive low on the ESP-01)
  }
  else {
    digitalWrite(LEDPIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }
  */
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
   

   if( (message_time + message_delay) < millis() ){
     message_time = millis();
     Serial.println("handleInterrupt");
   }
   
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
        
    /*if(pcf8574.read(5)==LOW) Serial.println("pin5");
    else Serial.println("!pin5");
    */
//    byte b = pcf8574.read8();
//    Serial.println( "INT: " + String(b));

//    byte keys = ((~b)) & 0x0F;
   
//    delay(0);
  }
  timer1->Update();
  timer2->Update();
  timer3->Update();
  everySec->Update();
}
 
void loop()
{
  handleInterrupt();

  if (!client.connected()) {
    reconnect_mqtt();
  }
  
  client.loop();

  if(!sentInitialRequest) {
    getNTPTime();
    sentInitialRequest = true;
  }
  
  nonBlockingBreath();  // call the nonblocking function
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
