// Communication & upload via Wifi
// Communication with Arduino Mega2560 Pro over TXD0 & RXD0
// Written by Ivan Burvenich - 24-05-2021
//
// This is the current production version on ESP8266
//
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
// OTA = Over The Air sketch upload, thus over "wifi".
#include <ArduinoOTA.h>
#include <basicOTA.h>
#include <Wire.h>

#define WIFI_SSID "telenet-6768627"
#define WIFI_PASS "avonduur2013"
int flash_switch=10;

//serial comm variables
uint8_t dataToSend = 0;
char dataFromAtmega[100];

ESP8266WebServer server(80);
String dataFromRequest = "";

// Set your Static IP address
IPAddress local_IP(192, 168, 2, 30);
// Set your Gateway IP address
IPAddress gateway(192, 168, 2, 1);

IPAddress subnet(255, 255, 0, 0);
//IPAddress primaryDNS(8, 8, 8, 8);   //optional
//IPAddress secondaryDNS(8, 8, 4, 4); //optional

void setup() {
  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

// setup Wifi for communication with Smartphone
  WiFi.mode(WIFI_STA); //esp01 initialized in station mode
  if (!WiFi.config(local_IP, gateway, subnet)) {
//    Serial.println("STA Failed to configure");
  }
  
  WiFi.begin(WIFI_SSID, WIFI_PASS); //connect to internet via WiFi

  // Connecting to WiFi...
  while (WiFi.status() != WL_CONNECTED){ 
    delay(500);     
  }
 
  server.onNotFound(handleRoot);
  server.begin();

  // OTA code start - Setup Firmware update over the air (OTA)
  setup_OTA();
  // OTA code end

  // Setup serial port for communication with Arduino - HDW serial
  Serial.begin(9600);

}

void loop() {

  // OTA code start - Check for OTA updates & blink led 10x fast
  ArduinoOTA.handle();
  if (flash_switch > 0) {
    // Blink LED fast as confirmation that the new code is loaded and working
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  flash_switch--;
  }
  // Stop LED blinking as visual confirmation that OTA is finished - HIGH = led OFF!
  digitalWrite(LED_BUILTIN, HIGH);    // HIGH = LED off

  server.handleClient();

}


void handleRoot(){

  dataFromRequest = server.uri();     //the data that comes with the GET request . Format : "/(data)"
  dataFromRequest.remove(0,1);        // deleting the '/' from the data
  Serial.print(dataFromRequest);

  while(!Serial.available());

  uint8_t i = 0;
  delay(500);
  while ((Serial.available() > 0) and (i < 100)) {
    dataFromAtmega[i] = Serial.read();
    i++;
  }
//  if(Serial.available())
//     Serial.readBytesUntil('#',dataFromAtmega,30); //reads the input until "#"  and stores it in the data array.

  //response and data sent to client 
  server.send(200,"text/plain",dataFromAtmega);
  //reset data array 
  for (uint8_t j = 0; j < 100; j++) {
    dataFromAtmega[j] = ' ';
  }

}
