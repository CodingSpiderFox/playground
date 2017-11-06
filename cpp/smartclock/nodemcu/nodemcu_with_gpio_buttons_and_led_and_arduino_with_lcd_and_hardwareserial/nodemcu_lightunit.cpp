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
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "TimerObject.h"

TimerObject *timer3 = new TimerObject(500);
TimerObject *everySec = new TimerObject(1000);

//MQTT related
const char* ssid = "berlin.freifunk.net";
const char* password = "";
const char* mqtt_server = "10.230.145.13";
const char* mqtt_userName = "SCLight";
const char* mqtt_password = "SCLight";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
int timeout = 200;

#define BRIGHT    300     //max led intensity (1-500)
#define INHALE    1250    //Inhalation time in milliseconds.
#define PULSE     INHALE*1000/BRIGHT
#define REST      1000    //Rest Between Inhalations.
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
    if (client.connect("SmartClockLight", mqtt_userName, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outSCLight", "hello world");
      // ... and resubscribe
      client.subscribe("inSCLight");
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

}

void setup()
{
  Serial.begin(115200);
  
  pinMode(ledPin, OUTPUT);   // LED pin as output.
  pinMode(MOTIONSENSOR, INPUT);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("Start task1");
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


void handleInterrupt () {
   if( (message_time + message_delay) < millis() ){
     message_time = millis();
     Serial.println("handleInterrupt");
   }

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
