//
//  www.blinkenlight.net
//
//  Copyright 2016 Udo Klein
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see http://www.gnu.org/licenses/

#include <SPI.h>                      // SPI interface for VS1053B
#include <Adafruit_VS1053.h>          // library for VS1053B
#include <SD.h>                       // library for SD on VS1053B
#include <dcf77.h>                    // DCF77 library from Udo Klein
#include <Wire.h>                     // I2C library - used to communicate with different sensors and interfaces
#include "RTClib.h"                   // Real Time clock library
#include <Adafruit_NeoPixel.h>        // Neopixel library to drive the leds
#include <Adafruit_BME280.h>          // Temperature, humidity & pressure sensor driver
#include "Adafruit_FRAM_I2C.h"        // FRAM - memory module library
//#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>

// DCF77 code

#if defined(__AVR__)
const uint8_t dcf77_analog_sample_pin = 3;
const uint8_t dcf77_sample_pin = 57;       // A3 == D57 - Mega 2560 PRO
const uint8_t dcf77_inverted_samples = 1;
const uint8_t dcf77_analog_samples = 0;
//const uint8_t dcf77_pin_mode = INPUT;  // disable internal pull up
const uint8_t dcf77_pin_mode = INPUT_PULLUP;  // enable internal pull up

const uint8_t dcf77_monitor_led = 18;  // A6 == D60

uint8_t ledpin(const uint8_t led) {
    return led;
}
#endif

uint8_t sample_input_pin() {
    const uint8_t sampled_data =
        #if defined(__AVR__)
        dcf77_inverted_samples ^ (dcf77_analog_samples? (analogRead(dcf77_analog_sample_pin) > 200)
                                                      : digitalRead(dcf77_sample_pin));
        #else
        dcf77_inverted_samples ^ digitalRead(dcf77_sample_pin);
        #endif

    digitalWrite(ledpin(dcf77_monitor_led), sampled_data);
    return sampled_data;
}
// DCF77 code - end

// Adafruit FRAM
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint32_t fram_index;
uint32_t fram_index_last;
uint32_t fram_indexmax;
uint8_t fram_index_hi;
uint8_t fram_index_lo;
uint8_t fram_indexmax_hi;
uint8_t fram_indexmax_lo;

// Adafruit DS3231 - RTC - Real Time Clock
RTC_DS3231 rtc;

// PWM interface for MH-Z14A CO2 sensor
SoftwareSerial SerialCom(13, 12); // RX, TX

 Adafruit_BME280 bme; // I2C
uint8_t temp;       // temperature
uint8_t humi;       // humidity
uint16_t pres;      // pressure

//LDR sensor - analog
int LDRpin = A2;
uint16_t LDRvalue;
// LDRaverage is average over last 10 seconds, therefore this value is multiplied by 10 and the current LDRvalue added and devided by 11 to get the new average
// This sufficient te be able to react on a sudden rise of the light (turn on a light) and to react on a slow rise of the light (daylight).
uint16_t LDRaverage = 500;
uint16_t LDRmin = 70;

// Definitions Neopixel string
// 24 neopixels! 12 on top, 12 on bottom
// make sure to set this to the correct pin, ignored for Esp8266

// Which pin on the Arduino is connected to the NeoPixels?
#define NEOPIXELPIN 29

// How many NeoPixels are attached to the Arduino?
#define NEOPIXELNUM 24 // NeoPixel string size + 1

//NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
Adafruit_NeoPixel pixels1(NEOPIXELNUM, NEOPIXELPIN, NEO_GRBW + NEO_KHZ800);

// Adafruit_VS1053_FilePlayer musicPlayer breakout - pin definitions
// first three use hardware SPI - no definitions neccesary
//#define MISO          50    // Input data, from VS1053/SD card (50)
//#define MOSI          51    // Output data, to VS1053/SD card (51)
//#define CLK           52    // SPI Clock, shared with SD card (52)
// These are the pins used for the breakout version
#define BREAKOUT_CS     53    // VS1053 chip select pin (output) (48)
#define BREAKOUT_RESET  62    // VS1053 reset pin (output) (49)
#define BREAKOUT_DCS    64    // VS1053 Data/command select pin (output) (XDCS) (47)
#define CARDCS          66    // Card chip select pin (SDCS) (53)
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ            18     // VS1053 Data request, ideally an Interrupt pin (3)

// variables for sound
//byte TrackLength;
char SND;
char number = 0;
char playing = 1;

uint8_t volume = 25;
// array containing 20 possible files to be played
// the sound files on the SD card NEED to have these names
const uint8_t arraySize = 20;
const uint8_t stringSize = 15;
char songs[arraySize][stringSize] =
{
  {"track000.mp3"},
  {"track001.mp3"},
  {"track002.mp3"},
  {"track003.mp3"},
  {"track004.mp3"},
  {"track005.mp3"},
  {"track006.mp3"},
  {"track007.mp3"},
  {"track008.mp3"},
  {"track009.mp3"},
  {"track010.mp3"},
  {"track011.mp3"},
  {"track012.mp3"},
  {"track013.mp3"},
  {"track014.mp3"},
  {"track015.mp3"},
  {"track016.mp3"},
  {"track017.mp3"},
  {"track018.mp3"},
  {"track019.mp3"},
};

// create breakout-example object!
Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);

// ESP8266 variables
char buffer[100];  // transmit buffer for ESP
char ESPflag = 0;    // 

uint8_t oldtime;
// 4 sets of datestamps are defined:
// - old...[0] - current timestamp (from DCF77 or RTC) - previously YY, MM, DD, hh, mm, ss
// - old...[1] - previous valid timestamp from DCF77   - previously oldyear, oldmonth, etc...
// - old...[2] - calculated = old...[1] + 1 second     - previously oldyear2, oldmonth2, etc...
// - old...[3] - calculated = old...[1] + 2 seconds    - previously oldyear3, oldmonth3, etc...
uint16_t oldyear[4];
uint8_t oldmonth[4];
uint8_t oldday[4];
uint8_t oldhour[4];
uint8_t oldminute[4];
uint8_t oldsecond[4];
uint16_t YY_exp;
uint8_t MM_exp;
uint8_t DD_exp;
uint8_t hh_exp;
uint8_t mm_exp;
uint8_t ss_exp;
uint8_t MM_exp0;
uint8_t DD_exp0;
uint8_t hh_exp0;
uint8_t mm_exp0;
uint8_t ss_exp0;
uint16_t YY_DCF77;
uint8_t MM_DCF77;
uint8_t DD_DCF77;
uint8_t hh_DCF77;
uint8_t mm_DCF77;
uint8_t ss_DCF77;
uint8_t MM_DCF770;
uint8_t DD_DCF770;
uint8_t hh_DCF770;
uint8_t mm_DCF770;
uint8_t ss_DCF770;
uint8_t oldyear_high;
uint8_t oldyear_low;
uint8_t YY_high;
uint8_t YY_low;
uint8_t grace = 5;    // general grace: no testing will bedone on the date and time coming from DFC77
uint8_t grace2count = 0;    // count the thicks to 6 after at least 1 error from DCF77

// "default" definitions for the pixels top and bottom side (need to be correctly set)
uint8_t rgbw[4][8];
uint8_t a;
uint8_t b;
uint8_t c;
uint8_t d;
uint8_t TopKill;
uint8_t BottomKill;
uint8_t TopWake;
uint8_t BottomWake;
float Frgbw[8];

// status variables for nixie tubes and the neopixels, top and bottom side. Status 1=ON, 0= OFF
// NT - Nixie Tubes
// LE - on/off-status LEds - digit 1 = status Top LEDs, digit 2 = status Bottom LEDs
// LE_old - previous on/off status LEds - digit 1 = status Top LEDs, digit 2 = status Bottom LEDs
uint8_t NT;
uint8_t LE;
uint8_t LE_old;

// Copy colors Top to Bottom checkbox
uint8_t CTB;

// E-command definitions & defaults
uint8_t Etrans;
uint8_t Enumber;
uint8_t Evalue[10];

// functions definitions for E-command
//int8_t Dir;            // direction for LEDs on the run
//uint8_t Min;            // intensity lowest admitted value - 0 to 127
//uint8_t Max;            // intensity highest admitted value - 0 to 127
//float Speed;        // increment for the next intensity value
//uint8_t Nc;
int8_t NTRdir;
int8_t NTGdir;
int8_t NTBdir;
int8_t NBRdir;
int8_t NBGdir;
int8_t NBBdir;

float TRspeed;
float TGspeed;
float TBspeed;
float BRspeed;
float BGspeed;
float BBspeed;

int16_t TRmin;
int16_t TGmin;
int16_t TBmin;
int16_t BRmin;
int16_t BGmin;
int16_t BBmin;

int16_t TRmax;
int16_t TGmax;
int16_t TBmax;
int16_t BRmax;
int16_t BGmax;
int16_t BBmax;

uint8_t LEDchanged = 1; // indicator: LED values are changed

// values of the LEDpixels can be schanged manually by a remote control via BlueTooth (BT). In this case via an app on a smartphone
// changing the RGBW values will change the color mix.

// set temp switch
uint8_t onoff;

// definitions for nixie driver digit 1 & 2
#define DIN1_PIN    23          // Nixie driver (shift register) serial data input pin 23             
#define EN1_PIN     25          // Nixie driver enable input pin 25
#define CLK1_PIN    27          // Nixie driver clock input pin 27
// definitions for nixie driver digit 3 & 4
#define DIN2_PIN    42          // Nixie driver (shift register) serial data input pin 42            
#define EN2_PIN     40          // Nixie driver enable input pin 40
#define CLK2_PIN    38          // Nixie driver clock input pin 38
// definitions for nixie driver digit 5 & 6
#define DIN3_PIN    36          // Nixie driver (shift register) serial data input pin 48             
#define EN3_PIN     34          // Nixie driver enable input pin 52
#define CLK3_PIN    32          // Nixie driver clock input pin 50

// Nixie driver
// array for 6 nixie tubes & definitions for fade function
uint8_t digit[] = { 15, 15, 15, 15, 15, 15};
uint8_t olddigit[] = { 15, 15, 15, 15, 15, 15};
uint8_t fade = 1;
uint8_t fadeSteps;
uint16_t fadeDelay;
uint8_t fadeCycle;
uint8_t fadePass; 

uint8_t countdown = 0;
uint8_t countdowntime = 6;

uint8_t reason = 1;
uint8_t tracknumber = 0;

// sequence back to front of the cathodes in a tube
uint8_t cathode[10] = { 1, 0, 2, 3, 9, 4, 8, 5, 7, 6 };

// slotmachine: delay between each displaychange - adapt to slow down (bigger number) or accelerate (smaller number) the movement
uint16_t delayms;
uint8_t cycles;

// Serial monitor on/off
uint8_t SMON;


// variables for sound
//uint8_t SND;
uint8_t slaveAddress = 9;

// variables for HM-Z14A CO2 sensor
int16_t ppm_PWM;

//  ============================ SETUP ============================
void setup() {
// DCF77 code
    using namespace Clock;

//    #if defined(__STM32F1__)
//    Serial1.begin(115200);
//    #else
    Serial.begin(115200);
//    #endif
    Serial.println();Serial.println();
    sprintln();
    sprintln(F("Based on and thanks to:"));
    sprintln(F("Simple DCF77 Clock V3.1.1"));
    sprintln(F("(c) Udo Klein 2016"));
    sprintln(F("www.blinkenlight.net"));
    sprintln();
    sprint(F("Sample Pin:      ")); sprintln(dcf77_sample_pin);
    sprint(F("Sample Pin Mode: ")); sprintln(dcf77_pin_mode);
    sprint(F("Inverted Mode:   ")); sprintln(dcf77_inverted_samples);
    #if defined(__AVR__)
    sprint(F("Analog Mode:     ")); sprintln(dcf77_analog_samples);
    #endif
    sprint(F("Monitor Pin:     ")); sprintln(ledpin(dcf77_monitor_led));
    sprintln();
    sprintln();
    sprintln(F("Initializing..."));

    pinMode(ledpin(dcf77_monitor_led), OUTPUT);
    pinMode(dcf77_sample_pin, dcf77_pin_mode);
//  DCF77 code - end

    Serial.println(F("DCF77 Nixie clock - Start SETUP"));

// Nixie driver setup
// Pins for digits 1 & 2
    pinMode(DIN1_PIN, OUTPUT); 
    digitalWrite(DIN1_PIN, LOW);    
    
    pinMode(CLK1_PIN, OUTPUT);
    digitalWrite(CLK1_PIN, LOW);         
  
    pinMode(EN1_PIN, OUTPUT);
    digitalWrite(EN1_PIN, LOW);

// Pins for digits 3 & 4
    pinMode(DIN2_PIN, OUTPUT); 
    digitalWrite(DIN2_PIN, LOW);    
    
    pinMode(CLK2_PIN, OUTPUT);
    digitalWrite(CLK2_PIN, LOW);         
  
    pinMode(EN2_PIN, OUTPUT);
    digitalWrite(EN2_PIN, LOW);

// Pins for digits 5 & 6
    pinMode(DIN3_PIN, OUTPUT); 
    digitalWrite(DIN3_PIN, LOW);    
    
    pinMode(CLK3_PIN, OUTPUT);
    digitalWrite(CLK3_PIN, LOW);         
  
    pinMode(EN3_PIN, OUTPUT);
    digitalWrite(EN3_PIN, LOW);

  oldtime = 99;
  Etrans = 0;
  NTRdir = 1;
  NTGdir = -1;
  NTBdir = 1;
  NBRdir = -1;
  NBGdir = 1;
  NBBdir = -1;
  
  TRspeed = 1.0 / 3;
  TGspeed = 2.0 / 3;
  TBspeed = 3.0 / 3;
  BRspeed = 1.0 / 4;
  BGspeed = 3.0 / 4;
  BBspeed = 2.0 / 4;
  
  TRmin = 0;
  TGmin = 0;
  TBmin = 0;
  BRmin = 0;
  BGmin = 0;
  BBmin = 0;
  
  TRmax = 127;
  TGmax = 127;
  TBmax = 127;
  BRmax = 127;
  BGmax = 127;
  BBmax = 127;
  TopKill = 0;
  BottomKill = 0;
  TopWake = 1;
  BottomWake =1;

  LEDchanged = 1; // indicator: LED values are changed
  ESPflag = 0;
  countdown = 0;
  countdowntime = 6;
  delayms = 10;
  cycles = 66;

// Start PWM serial communication for HM-Z14A CO2 sensor
  SerialCom.begin(9600);
  ppm_PWM = gas_concentration_uart();
  if ((ppm_PWM > 0) and (ppm_PWM < 5001)) {
    Serial.print(F("HM-Z14A sensor found, value: "));
    Serial.println(ppm_PWM);
  }
  else {
    Serial.print(F("HM-Z14A sensor NOT found, invalid value: "));
    Serial.println(ppm_PWM);
  }

// start BME280
  if (bme.begin(0x76)) {
    Serial.println(F("BME280 sensor found"));
  } 
  else {
    Serial.println(F("No BME280 sensor found ... check wiring!"));
    while (1) delay(10);
  } 

// start Adafruit FRAM
  if (fram.begin()) {     // default address 0x50 is used;
    Serial.println(F("Found I2C FRAM"));
  }
  else {
    Serial.println(F("No I2C FRAM found ... check wiring!"));
    while(1);
  }

// Restore parameters from FRAM
// FRAM memory mapping
// 0x00(000) -> 0x1F(031) - not used
// 0x20(032) -> 0x3F(063) - values from array rgbw[4][8] = 32 bytes
//          0x20(032) - rgbw[0][0]    0x28(040) - rgbw[1][0]    0x30(048) - rgbw[2][0]   0x38(056) - rgbw[3][0]
//          0x21(033) - rgbw[0][1]    0x29(041) - rgbw[1][1]    0x31(049) - rgbw[2][1]   0x39(057) - rgbw[3][1]
//          0x22(034) - rgbw[0][2]    0x2A(042) - rgbw[1][2]    0x32(050) - rgbw[2][2]   0x3A(058) - rgbw[3][2]
//          0x23(035) - rgbw[0][3]    0x2B(043) - rgbw[1][3]    0x33(051) - rgbw[2][3]   0x3B(059) - rgbw[3][3]
//          0x24(036) - rgbw[0][4]    0x2C(044) - rgbw[1][4]    0x34(052) - rgbw[2][4]   0x3C(060) - rgbw[3][4]
//          0x25(037) - rgbw[0][5]    0x2D(045) - rgbw[1][5]    0x35(053) - rgbw[2][5]   0x3D(061) - rgbw[3][5]
//          0x26(038) - rgbw[0][6]    0x2E(046) - rgbw[1][6]    0x36(054) - rgbw[2][6]   0x3E(062) - rgbw[3][6]
//          0x27(039) - rgbw[0][7]    0x2F(047) - rgbw[1][7]    0x37(055) - rgbw[2][7]   0x3F(063) - rgbw[3][7]
// 0x40(064) -> 0x4E(078) - program variables - 6 bytes & 10 bytes free 
//          0x40(064) - SND
//          0x41(065) - nvt
//          0x42(066) - NT
//          0x43(067) - LE
//          0x44(068) - CTB
//          0x45(069) - nvt
//          0x46(070) - SMON
// 0x50(080) -> 0x5F(095) - values for array Evalue[10] = 10 bytes & 3 bytes free
//          0x50(080) - Evalue[0]
//          0x51(081) - Evalue[1]
//          0x52(082) - Evalue[2]
//          0x53(083) - Evalue[3]
//          0x54(084) - Evalue[4]
//          0x55(085) - Evalue[5]
//          0x56(086) - Evalue[6]
//          0x57(087) - Evalue[7]
//          0x58(088) - Evalue[8]
//          0x59(089) - Evalue[9]
//          0x5A(090) - fade
//          0x5B(091) - fadeDelayHi ( fadeDelayHi = fadeDelay / 256 )
//          0x5C(092) - fadeDelayLo ( fadeDelayLo = fadeDelay - (fadeDelayHi *256) )
//          0x5D(093) - fadeSteps
//          0x5E(094) - fadeCycle
// 0x90(144) -> addresses for free slots for DCF77 error reporting
//          0x90(144) - fram_index_hi - first free address - high
//          0x91(145) - fram_index_lo - first free address - low
//          0x92(146) - fram_indexmax_hi - highest free address - high
//          0x93(147) - fram_indexmax_lo - highest free address - low
// 0xA0(160) -> 0xxx(nnn) - log data: DCF77 not sync with RTC - x times 16 bytes - 16 bytes for each error
//          0xA0(160) - YY_high - DCF77 date
//          0xA1(161) - YY_low - DCF77 date
//          0xA2(162) - MM - DCF77 date
//          0xA3(163) - DD - DCF77 date
//          0xA4(164) - hh - DCF77 time
//          0xA5(165) - mm - DCF77 time
//          0xA6(166) - ss - DCF77 time
//          0xA7(167) - oldyear_high - RTC date
//          0xA8(168) - oldyear_low - RTC date
//          0xA9(169) - oldmonth - RTC date
//          0xAA(170) - oldday - RTC date
//          0xAB(171) - oldhour - RTC time
//          0xAC(172) - oldmin - RTC time
//          0xAD(173) - oldsec - RTC time
//          0xAE(174) - free
//          0xAF(175) - free

// write instructions for documentation purposes only - values are set by another sketch "Simple_clock_Ivan_3_Init_FRAM.ino"
//for(int x = 0; x <= 3; x++) {
//  for(int y = 0; y <= 7; y++) {
//    fram.write8(x * 8 + y + 32, rgbw[x][y]);
//  }
//}
//fram.write8(64, SND);
//fram.write8(66, NT);
//fram.write8(67, LE);
//fram.write8(68, CTB);
//fram.write8(70, SMON);
//for(int i=0;i<=9;i++) {
//  fram.write8(i+80, Evalue[i]);
//}
//fram.write8(90, fade);
//int fadeDelayHi = fadeDelay/256;
//int fadeDelayLo = fadeDelay - (fadeDelayHi * 256); 
//fram.write8(91, fadeDelayHi);
//fram.write8(92, fadeDelayLo);
//fram.write8(93, fadeSteps);
//fram.write8(94, fadeCycle);
//
//fram.write8(144, fram_index_hi);
//fram.write8(145, fram_index_lo);
//fram.write8(146, fram_indexmax_hi);
//fram.write8(147, fram_indexmax_lo);

for(int x = 0; x <= 3; x++ ) {
  for(int y = 0; y <= 7; y++ ) {
    rgbw[x][y] = fram.read8(x * 8 + y + 32);
  }
}
SND = fram.read8(64);
NT  = fram.read8(66);
LE  = fram.read8(67);
CTB = fram.read8(68);
SMON = fram.read8(70);
SMON = 1;   //Tijdelijk gedurende tests
for(int i = 0; i <= 9; i++ ) {
  Evalue[i] = fram.read8(i + 80);
}
fade = 1;
//fade = 1; // tijdelijk tot FRAM geïnstalleerd en geInitialiseerd is
//NT = 1;   // tijdelijk tot FRAM geïnstalleerd en geInitialiseerd is

Enumber = 2;
Evalue[Enumber] = 1;

// RTC setup
  if (! rtc.begin()) {
    Serial.println(F("No RTC DS3231 found ... check wiring!"));
    while (1);
  }
  else Serial.println(F("RTC DS3231 found"));
//rtc.begin();
  if (rtc.lostPower())
  {
    Serial.println(F("RTC lost power! Set time to compile time of sketch."));
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
   }
   else
    {
    Serial.print(F("RTC already setup, super cap backup maintained date and time: "));
    // Print date and time from RTC clock
    DateTime nowrtc (rtc.now());
    Serial.print(nowrtc.day());
    Serial.print(F("/"));
    Serial.print(nowrtc.month());
    Serial.print(F("/"));
    Serial.print(nowrtc.year());
    Serial.print(F("  "));
    Serial.print(nowrtc.hour());
    Serial.print(F(":"));
    Serial.print(nowrtc.minute());
    Serial.print(F(":"));
    Serial.println(nowrtc.second());
  }

  //LDR reading test - luminosity
  LDRvalue = analogRead(LDRpin);
  if (LDRvalue == 0) Serial.println(F("WARNING - LDR not connected to the board - readings will be false"));
  else {
    Serial.print(F("LDR detected, current value: "));
    Serial.println(LDRvalue);
  }
  
// start VS1053B - music player
  SND = 1;
  if (! musicPlayer.begin()) { 
    Serial.println(F("Couldn't find VS1053B, check wiring and definitions."));
    // disable sound
    SND = 0;  
  }
  else Serial.println(F("VS1053B started."));
  delay (1000);
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present - can't play anything. Fix SD card."));
    // disable sound
    SND = 0;  
  }
  else Serial.println(F("SD card found on VS1053B."));

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(25, 25);

  // DREQ is on an interrupt pin (on mega #3) we can do background audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int

  // set GPIO pin 7 to output
  musicPlayer.GPIO_pinMode(7, OUTPUT);
  // set GPIO pin 7 LOW (= disable amplifiers)
//  musicPlayer.GPIO_digitalWrite(7, LOW);
//  delay (1000);

  if(SND == 1) {
  // set GPIO pin 7 HIGH (= enable amplifiers)
//  musicPlayer.GPIO_digitalWrite(7, HIGH);
  tracknumber = 1;
  PlayTrack(tracknumber);
  }

// start serial communication with ESP8266
// hardware serial D14(TX3) & D15(RX3)
  Serial3.begin(9600); 

// neopixel setup
  pixels1.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels1.clear(); // Set all pixel colors to 'off'
  pixels1.show();  // Send the updated pixel colors to the hardware.

// DCF77 code
    uint8_t count = 0;
    uint8_t countm = 0;

    DCF77_Clock::setup();
    DCF77_Clock::set_input_provider(sample_input_pin);

    // Wait till clock is synced, depending on the signal quality this may take
    // rather long. About 5 minutes with a good signal, 30 minutes or longer
    // with a bad signal
    for (uint8_t state = Clock::useless;
        state == Clock::useless || state == Clock::dirty;
        state = DCF77_Clock::get_clock_state()) {

        // wait for next sec
        Clock::time_t now;
        DCF77_Clock::get_current_time(now);

        // render one dot per second while initializing - modified by Ivan Burvenich
        if (count == 0) {
          if (countm < 10) Serial.print(" ");
          Serial.print(countm);
          Serial.print(" ");
        }
        if (count == 0) sprint('0');
        else if ((count == 5) || (count == 15) ||(count == 25) ||(count == 35) ||(count == 45) || (count == 55)) sprint('+');
        else if (count == 10) sprint('1');
        else if (count == 20) sprint('2');
        else if (count == 30) sprint('3');
        else if (count == 40) sprint('4');
        else if (count == 50) sprint('5');
        else sprint('-');
        ++count;
        if (count == 60) {
            count = 0;
            ++countm;
            sprintln();
        }
// DCF77 code - end

// ********************************************************
// *            PRINT RTC time to nixie tubes             *
// ********************************************************
        DateTime nowrtc (rtc.now());
        digit[1] = nowrtc.second() / 10;
        digit[0] = nowrtc.second() - (digit[1] * 10);
        digit[3] = nowrtc.minute() / 10;
        digit[2] = nowrtc.minute() - (digit[3] * 10);
        digit[5] = nowrtc.hour() / 10;
        digit[4] = nowrtc.hour() - (digit[5] * 10);
        NixieDisplayAll();
        oldsecond[1] = nowrtc.second();
        oldminute[1] = nowrtc.minute();
        oldhour[1] = nowrtc.hour();
        oldday[1] = nowrtc.day();
        oldmonth[1] = nowrtc.month();
        oldyear[1] = nowrtc.year();


// "FLITS" effect on Neopixels
          // pixels.Color() takes RGBW values, from 0,0,0,0 up to 255,255,255,255
          // Here we're using a bright Warm White color:
        for(int i = NEOPIXELNUM / 2; i < NEOPIXELNUM; i++) { // For each pixel...
          pixels1.setPixelColor(i, pixels1.Color(0, 0, 0, 75));
        }
          pixels1.show();   // Send the updated pixel colors to the hardware.
          delay(5000); // Pause before next pass through loop
          // Here we're using no color:
        for(int i = NEOPIXELNUM / 2; i < NEOPIXELNUM; i++) { // For each pixel...
          pixels1.setPixelColor(i, pixels1.Color(0, 0, 0, 0));
        }
          pixels1.show();   // Send the updated pixel colors to the hardware.
    }
  fade = fram.read8(90);
  uint8_t fadeDelayHi = fram.read8(91);
  uint8_t fadeDelayLo = fram.read8(92);
  fadeDelay = (fadeDelayHi * 256) + fadeDelayLo;
  fadeSteps = fram.read8(93);
  fadeCycle = fram.read8(94);   // devider for fadeSteps
  fade = fadeSteps;     // this forces fading on the nixie tubes. Can be manually overwritten by BT device.
  fadePass = fadeCycle / 2; 

}
// Setup - end

  
// 3 functions for ESP8266 communications: getcharacter(), getnextdata() and get1Data()
// function 1: read 1 byte from ESP buffer - no conversion
char getcharacter()
{
  if (Serial3.available() > 0) {
    ESPflag = Serial3.read(); 
  }
  else {
    if (SMON == 1) Serial.println(F("Short on received data"));
    ESPflag = 'Z'; 
  }
  return ESPflag;
}

// function 2: read 3 bytes from BT buffer and convert to numeric value
uint16_t getnextdata()
{
uint16_t v;
  v = 0;
  if (Serial3.available() > 0) {
    ESPflag = Serial3.read(); 
    if (ESPflag >= '0' or ESPflag <= '9') v = (ESPflag-48) * 100;
  }
  else {
    if (SMON == 1) Serial.println(F("Short on received data"));
  }
  
  if (Serial3.available() > 0) {
    ESPflag = Serial3.read(); 
    if (ESPflag >= '0' or ESPflag <= '9') v = v + ((ESPflag-48) * 10);
  }
  else {
    if (SMON == 1) Serial.println(F("Short on received data"));
    v = v / 100;
    return v;
  }

  if (Serial3.available() > 0) {
    ESPflag = Serial3.read(); 
    if (ESPflag >= '0' or ESPflag <= '9') v = v + (ESPflag-48);
  }
  else {
    if (SMON == 1) Serial.println(F("Short on received data"));
    v = v / 10;
  }
  return v;
}     

// function 3: read 1 byte from BT buffer and convert to numeric value
uint8_t get1data()
{
  uint8_t v;
  if (Serial3.available() > 0) {
    ESPflag = Serial3.read(); 
    if (ESPflag >= '0' or ESPflag <= '9') {
      v = ESPflag-48;
    }
  }
  else {
    if (SMON == 1) Serial.println(F("Short on received data"));
  }
  return v;
}

// unique routine for setting LED values depending on the status LE
void setLEDS () {
  // available information needed to execute this function correctly:
  //  - LE_old - value between 0 and 3
  //  - LE - value between 0 and 3
  //  - LE = 0 - Top LEDs & Bottom LEDs OFF
  //  - LE = 1 - Top LEDs ON & Bottom LEDs OFF
  //  - LE = 2 - Top LEDs OFF & Bottom LEDs ON
  //  - LE = 3 - Top LEDs & Bottom LEDs  ON
  //  - LE > 3 - all LEDs OFF, but status saved: 4 means saved status 0, 5 means 1, 6 means 2, 7 means 3
  //  - a - start wake index colors LEDs - 0-3: Top & 4-7: Bottom
  //  - b - end wake index colors LEDs - 0-3: Top & 4-7: Bottom
  //  - c - start kill index colors LEDs - 0-3: Top & 4-7: Bottom
  //  - d - end kill index colors LEDs - 0-3: Top & 4-7: Bottom

  if ((LE == 0) || (LE > 3)) {
  // wake parameters - first & second bit = 0
    a = 0;
    b = 0;
  // kill parameters - first & second bit = 0
    c = 0;
    d = 7;
    if (TopKill == 1) c = 4;
    if (BottomKill == 1) d = 3;
    TopKill = 1;
    BottomKill = 1;
    TopWake = 0;
    BottomWake = 0;
  }
  if ((LE == 1)) {
  // wake parameters - first bit = 1 (Top)
    a = 0;
    b = 3;
    if (TopWake == 1) a = 4;
    TopKill = 0;
    TopWake = 1;
  // kill parameters - second bit = 0 (Bottom)
    c = 4;
    d = 7;
    if (BottomKill == 1) d = 3;
    BottomKill = 1;
    BottomWake = 0;
  }
  if ((LE == 2)) {
  // wake parameters - second bit = 1 (Bottom)
    a = 4;
    b = 7;
    if (BottomWake == 1) b = 3;
    BottomKill = 0;
    BottomWake = 1;
  // kill parameters - first bit = 0 (Top)
    c = 0;
    d = 3;
    if (TopKill == 1) c = 4;
    TopKill = 1;
    TopWake = 0;
  }
  if ((LE == 3)) {
  // wake parameters - first & second bit = 1
    a = 0;
    b = 7;
    if (TopWake == 1) a = 4;
    if (BottomWake == 1) b = 3;
    TopKill = 0;
    BottomKill = 0;
    TopWake = 1;
    BottomWake = 1;
  // kill parameters - none = 0
    c = 0;
    d = 0;
  }

  // kill LEDs
  if (c < d) {
    for (int8_t i = c ; i <= d ; i++) {
      // save RGBW led settings to temp storage - will be called back when LEDS are activated
      rgbw[2][i] = rgbw[1][i]; //byte S** = N**;
//      fram.write8(2 * 8 + i + 32, rgbw[2][i]);
      // set LEDS OFF        
      rgbw[1][i] = 0; //byte N** = 0;
//      fram.write8(1 * 8 + i + 32, rgbw[1][i]);
    }
  }

  // wake LEDs
  if (a < b) {
    for (int8_t i = a ; i <= b ; i++) {
      // restore led settings from temp storage
      rgbw[1][i] = rgbw[2][i]; //byte N** = S**;
//      fram.write8(1 * 8 + i + 32, rgbw[1][i]);
    }
  }
  // restart actity on leds, like:
  // "LEDS OP WANDEL" Evalue[4] set to 1 if current value is 2. 1 = active
  if (Evalue[4] == 2) {
    Evalue[4] = 1;
//    fram.write8(4+69, Evalue[4]);
  }
}

// 2 functions for EXTRA E4 - LEDS OP WANDEL
// Calculate a new color value with 2 return values:
// Nc is the new color value, as the return value of the function
// Dir via type 'int8_t&'
float Newcolor(float Nc, int8_t& Dir, int16_t Min, int16_t Max, float Speed) {
  if (Nc >= Max) Dir = -1;
  if (Nc <= Min) Dir = 1;
  Nc = Nc + (Dir * Speed);
  if (Nc > 255) Nc = 255;
  if (Nc < 0) Nc = 0;
  return Nc;
}

// Set RGBW[1][x] to new colors
// this function sets new values for all rgb colors. White is discarded from the colormix
// the rules are described in the function, starting rgb values are the current values
// static parameters are: minimum, maximum & speed values, direction evolves with the colors in function Newcolor 
// speed could possibly evolve too, but not yet ... thinking about that.
void LEDSONTHERUN() {
  // TOP change
  //    R = R+speed, until R>=127, then reverse until R=0, speed = 1 / 3
  //    G = G-speed, until G<=0, then reverse until G=127, speed = 2 / 3
  //    B = B+speed, until B>=127, then reverse until B=0, speed = 3 / 3
  //    W=0
  // BOTTOM change
  //    R = R-speed, until R=0, then reverse until R=127, speed = 3 / 4
  //    G = G+speed, until G=127, then reverse until G=0, speed = 1 / 4
  //    B = B-speed, until B=0, then reverse until B=127, speed = 2 / 4
  //    W=0

  if (TopWake == 1) {
    // TOP RED
    if ((rgbw[1][0] != 0) || ((rgbw[1][1] == 0) || (rgbw[1][2] == 0))) {
      Frgbw[0] = Newcolor(Frgbw[0], NTRdir, TRmin, TRmax, TRspeed);
      rgbw[1][0] = round(Frgbw[0]);
      //    fram.write8(1 * 8 + 0 + 32, rgbw[1][0]);
    }
    // TOP GREEN
    if ((rgbw[1][1] != 0) || ((rgbw[1][0] == 0) || (rgbw[1][2] == 0))) {
      Frgbw[1] = Newcolor(Frgbw[1], NTGdir, TGmin, TGmax, TGspeed);
      rgbw[1][1] = round(Frgbw[1]);
      //    fram.write8(1 * 8 + 1 + 32, rgbw[1][1]);
    }
    // TOP BLUE
    if ((rgbw[1][2] != 0) || ((rgbw[1][0] == 0) || (rgbw[1][1] == 0))) {
      Frgbw[2] = Newcolor(Frgbw[2], NTBdir, TBmin, TBmax, TBspeed);
      rgbw[1][2] = round(Frgbw[2]);
  //    fram.write8(1 * 8 + 2 + 32, rgbw[1][2]);
    }  
  }
  // BOTTOM LEDs - copy TOP?
  if (CTB == 1) {
    for (int8_t i = 0; i <= 2; i++) {
      // save all led settings to temp storage - will be called back when LEDS are activated
      rgbw[1][i+4] = rgbw[1][i]; //byte NB* = NT*;
//      fram.write8(1 * 8 + i + 4 + 32, rgbw[1][i+4]);
    }
  }
  else {
    if (BottomWake == 1) {
      // BOTTOM RED
      if ((rgbw[1][4] != 0) || ((rgbw[1][5] == 0) || (rgbw[1][6] == 0))) {
        Frgbw[4] = Newcolor(Frgbw[4], NBRdir, BRmin, BRmax, BRspeed);
        rgbw[1][4] = round(Frgbw[4]);
  //      fram.write8(1 * 8 + 4 + 32, rgbw[1][4]);
      }
      // BOTTOM GREEN
      if ((rgbw[1][5] != 0) || ((rgbw[1][4] == 0) || (rgbw[1][6] == 0))) {
        Frgbw[5] = Newcolor(Frgbw[5], NBGdir, BGmin, BGmax, BGspeed);
        rgbw[1][5] = round(Frgbw[5]);
  //      fram.write8(1 * 8 + 5 + 32, rgbw[1][5]);
      }
      // BOTTOM BLUE
      if ((rgbw[1][6] != 0) || ((rgbw[1][4] == 0) || (rgbw[1][5] == 0))) {
        Frgbw[6] = Newcolor(Frgbw[6], NBBdir, BBmin, BBmax, BBspeed);
        rgbw[1][6] = round(Frgbw[6]);
  //      fram.write8(1 * 8 + 6 + 32, rgbw[1][6]);
      }
    }
  }
}

void padded2Digits(uint8_t& n0, uint8_t& n) {
  n0 = n / 10;
  n = n - (n0 * 10);
}

void paddedPrint2Digits(uint8_t n) {
  if (n < 10) Serial.print("0");
  Serial.print(n);
}

// DCF77 code
void paddedPrint(BCD::bcd_t n) {
  sprint(n.digit.hi);
  sprint(n.digit.lo);
}
uint8_t getBCDhi(BCD::bcd_t n) {
    return n.digit.hi;
}
uint8_t getBCDlo(BCD::bcd_t n) {
    return n.digit.lo;
}
// DCF77 code - end

// Dump date & time data to serial monitor in specific format - start
void DumpDateTime() {
  Serial.println ("Previous date & time   ! DCF77 date & time");
  Serial.println ("YYYY-MM-DD  hh:mm:ss   ! YYYY-MM-DD  hh:mm:ss");
  Serial.print (oldyear[1]); Serial.print ("-");
  paddedPrint2Digits(oldmonth[1]); Serial.print ("-");
  paddedPrint2Digits(oldday[1]); Serial.print ("  ");
  paddedPrint2Digits(oldhour[1]); Serial.print (":");
  paddedPrint2Digits(oldminute[1]); Serial.print (":");
  paddedPrint2Digits(oldsecond[1]); Serial.print ("   ! ");
  Serial.print (oldyear[0]); Serial.print ("-");
  paddedPrint2Digits(oldmonth[0]); Serial.print ("-");
  paddedPrint2Digits(oldday[0]); Serial.print ("  ");
  paddedPrint2Digits(oldhour[0]); Serial.print (":");
  paddedPrint2Digits(oldminute[0]); Serial.print (":");
  paddedPrint2Digits(oldsecond[0]); Serial.println();
  Serial.print (oldyear[2]); Serial.print ("-");
  paddedPrint2Digits(oldmonth[2]); Serial.print ("-");
  paddedPrint2Digits(oldday[2]); Serial.print ("  ");
  paddedPrint2Digits(oldhour[2]); Serial.print (":");
  paddedPrint2Digits(oldminute[2]); Serial.print (":");
  paddedPrint2Digits(oldsecond[2]); Serial.println ("   !");
  Serial.print(oldyear[3]); Serial.print ("-");
  paddedPrint2Digits(oldmonth[3]); Serial.print ("-");
  paddedPrint2Digits(oldday[3]); Serial.print ("  ");
  paddedPrint2Digits(oldhour[3]); Serial.print (":");
  paddedPrint2Digits(oldminute[3]); Serial.print (":");
  paddedPrint2Digits(oldsecond[3]); Serial.println ("   !");
}
// Dump date & time data to serial monitor in specific format - end

// ********** MAIN LOOP - START **********
void loop() {
  // SMON - switch - 1 = output messages to the serial monitor - can be set by ESP
  //                 0 = no output to the serial monitor
  // DCF77 code

  Clock::time_t now;
  DCF77_Clock::get_current_time(now);
  if (now.month.val > 0) {
    if (SMON == 1) {
      switch (DCF77_Clock::get_clock_state()) {
          case Clock::useless: sprint(F("useless: ")); break;
          case Clock::dirty:   sprint(F("dirty  : ")); break;
          case Clock::synced:  sprint(F("synced : ")); break;
          case Clock::locked:  sprint(F("locked : ")); break;
      }
//      sprint(" ");
      
      // prepare values for date and time treatment
      oldyear[0] = 2000 + getBCDhi(now.year) * 10 + getBCDlo(now.year);
      oldmonth[0] = getBCDhi(now.month) * 10 + getBCDlo(now.month);
      oldday[0] = getBCDhi(now.day) * 10 + getBCDlo(now.day);
      oldhour[0] = getBCDhi(now.hour) * 10 + getBCDlo(now.hour);
      oldminute[0] = getBCDhi(now.minute) * 10 + getBCDlo(now.minute);
      oldsecond[0] = getBCDhi(now.second) * 10 + getBCDlo(now.second);
    
      Serial.print(oldyear[0]);
      sprint('-');
      paddedPrint(now.month);
      sprint('-');
      paddedPrint(now.day);
      sprint(" - ");

      paddedPrint(now.hour);
      sprint(':');
      paddedPrint(now.minute);
      sprint(':');
      paddedPrint(now.second);

      sprint(" UTC: +");
      sprint(now.uses_summertime? '2': '1');
      sprintln();
    }
  // DCF77 code - end

  // display time on nixie tubes and use transition mode - Evalue[2]: 0: Instant (default, fade = 1), 1 = cross dissolve (fade = fadeSteps) - can be set by BT  
    if (countdown == 0) {
      digit[0] = getBCDlo(now.second);
      digit[1] = getBCDhi(now.second);
      digit[2] = getBCDlo(now.minute);
      digit[3] = getBCDhi(now.minute);
      digit[4] = getBCDlo(now.hour);
      digit[5] = getBCDhi(now.hour);
//      NixieDisplayAll();
    }
    if (countdown > 0) {
      countdown--;
//      if (countdown == 0) touchcount = 0;
    }

  }

  // Actions once every hour:
  // - bell sound between 07:00 and 23:00, and only if NT = 1
  // - start slot machine at minute 13, second 13

  if (!musicPlayer.playingMusic) {
    // no longer playing - shut down amplifiers and reset playing parameter
    if (playing == 1) {
      musicPlayer.GPIO_digitalWrite(7, LOW);
      playing = 0;
    }
  }

  // check to ring a bell every hour (HH:00:00) when NT = 1 (NT = 0 means all visible and audible activity is shut down)
  if (NT == 1 && oldminute[0] == 0 && oldsecond[0] == 0 && oldhour[0] >= 7 && oldhour[0] <= 23) {
    tracknumber = 1;
    PlayTrack(tracknumber);
  }

  // check time (every HH:13:13) to start automated slot machine maintenance - not in sleep mode
  if (NT == 1 && oldminute[0] == 13 && oldsecond[0] == 13) {
  Serial3.println(F("E31"));
    slotmachine();
  }


  // grace: do this tests only when grace = 0. grace is decremented after this step
  // grace let the clock skip these test to avoid declaring unnecessary 'errors', such as after displaying information for longer then 1 second.
  // In that case old time and date are not incremented. Usually grace is set to 5 (seconds). Old time & date will then be reset and control can carry on from there.
  if (grace == 0) {
    // check if DCF77 time & date is in line with previous DCF77 time & date
    // prepare old date & time (increment by 1 second) - start
    // note - initial first time 'old' time & date came from RTC in setup
    // - old...[0] - DCF77 time/date - 3 timestamps will be checked against it:
    // - old...[1] - previous timestamp
    // - old...[2] - expected timestamp - this should be the perfect match
    // - old...[3] - next timestamp
    //
    // first calculate the expected "old...[2]" timestamp, based on 'previous' timestamp "old...[1]" ("old...[1]" is set at the end of this procedure)
    // call function CalcReferenceDatestamp (i, j)
    // - from i = reference timestamp in table old...[i] - input
    // - to j = reference new timestamp in table old...[j] - output
    // - increment is fixed = 1 second

    CalcReferenceDatestamp (1, 2);
    
    // then calculate the next "old...[3]" timestamp, based on timestamp "old...[2]"
    
    CalcReferenceDatestamp (2, 3);
    
    // prepare old date & time (increment by 1 second) - end
  
    // compare old time & date with current DCF77 time & date - 1 test on old dates (old, old2 or old3) must match
//    if (((oldyear[0] == oldyear[1]) && (oldmonth[0] == oldmonth[1]) && (oldday[0] == oldday[1]) && (oldhour[0] == oldhour[1]) && (oldminute[0] == oldminute[1]) && (oldsecond[0] == oldsecond[1])) ||
    if ((oldyear[0] == oldyear[2]) && (oldmonth[0] == oldmonth[2]) && (oldday[0] == oldday[2]) && (oldhour[0] == oldhour[2]) && (oldminute[0] == oldminute[2]) && (oldsecond[0] == oldsecond[2])) {
//        ((oldyear[0] == oldyear[3]) && (oldmonth[0] == oldmonth[3]) && (oldday[0] == oldday[3]) && (oldhour[0] == oldhour[3]) && (oldminute[0] == oldminute[3]) && (oldsecond[0] == oldsecond[3]))) {
//      if (SMON == 1) {
//        Serial.println ();
//        Serial.println ("DCF77 library is in line with RTC");
//        DumpDateTime();
//      }
    }
    // mismatch found, but:
    // accept change at wintertime and sommertime at night when 2 becomes 3 or 3 becomes 2 
    else if ((((oldyear[0] == oldyear[1]) && (oldmonth[0] == oldmonth[1]) && (oldday[0] == oldday[1]) && (oldminute[0] == oldminute[1]) && (oldsecond[0] == oldsecond[1]))
              && (((oldhour[0] == 2) && (oldhour[1] == 3)) or ((oldhour[0] == 3) && (oldhour[1] == 2)))) ||
             (((oldyear[0] == oldyear[2]) && (oldmonth[0] == oldmonth[2]) && (oldday[0] == oldday[2]) && (oldminute[0] == oldminute[2]) && (oldsecond[0] == oldsecond[2]))
              && (((oldhour[0] == 2) && (oldhour[2] == 3)) or ((oldhour[0] == 3) && (oldhour[2] == 2)))) ||  
             (((oldyear[0] == oldyear[3]) && (oldmonth[0] == oldmonth[3]) && (oldday[0] == oldday[3]) && (oldminute[0] == oldminute[3]) && (oldsecond[0] == oldsecond[3]))
              && (((oldhour[0] == 2) && (oldhour[3] == 3)) or ((oldhour[0] == 3) && (oldhour[3] == 2))))) {
//      if (SMON == 1) {
//        Serial.println ();
//        Serial.println ("DCF77 signal is in line with RTC");
//        Serial.println ("Previous date & time   ! DCF77 date & time");
//        DumpDateTime();
//      }
    }
    else {
      // mismatch found - report the mismatch to serial monitor & to FRAM
      if (SMON == 1) {
          Serial.println ();
          Serial.println ("DCF77 library generated wrong date or/and time");
          Serial.println ("Previous date & time   ! DCF77 date & time");
          DumpDateTime();
      }
          // get location in FRAM to put the data
          fram_index_hi = fram.read8(144);
          fram_index_lo = fram.read8(145);
          fram_indexmax_hi = fram.read8(146);
          fram_indexmax_lo = fram.read8(147);
          fram_index = (fram_index_hi * 256) + fram_index_lo;
          fram_indexmax = (fram_indexmax_hi * 256) + fram_indexmax_lo;

          YY_high = oldyear[2] / 100;
          YY_low = oldyear[2] - (YY_high * 100);
          fram.write8(fram_index, YY_high);
          fram.write8(fram_index + 1, YY_low);
          fram.write8(fram_index + 2, oldmonth[2]);
          fram.write8(fram_index + 3, oldday[2]);
          fram.write8(fram_index + 4, oldhour[2]);
          fram.write8(fram_index + 5, oldminute[2]);
          fram.write8(fram_index + 6, oldsecond[2]);
          oldyear_high = oldyear[0] / 100;
          oldyear_low = oldyear[0] - (oldyear_high * 100);
          fram.write8(fram_index + 7, oldyear_high);
          fram.write8(fram_index + 8, oldyear_low);
          fram.write8(fram_index + 9, oldmonth[0]);
          fram.write8(fram_index + 10, oldday[0]);
          fram.write8(fram_index + 11, oldhour[0]);
          fram.write8(fram_index + 12, oldminute[0]);
          fram.write8(fram_index + 13, oldsecond[0]);
          // calculate next slot address in FRAM
          fram_index = fram_index + 16;
          if (fram_index <= fram_indexmax) {
            fram_index_hi = fram_index / 256;
            fram_index_lo = fram_index - (fram_index_hi * 256);
            fram.write8(144, fram_index_hi);
            fram.write8(145, fram_index_lo);
          }
          else {
//            Serial.println ("DCF77 error reporting - FRAM memory full");
            // signal it to BT in order to free FRAM memory.
            // Actions:
            // - read the errors - program "Simple_Clock_Ivan_3_DUMP_DCF77_errors_FRAM"
            // - clean the momory locations by writing zeroes & reset the pointers - program "Simple_Clock_Ivan_3_init_FRAM"
            //
            // CODE TO WRITE !!!!
            //
          }
          
          grace2count = grace2count + 1;
//      }
    }
    // when at least 1 error occurred and 5 ticks after that error are passed: reset clock - return to setup and restart DCF77 sync procedure
    if (grace2count > 5) {
//      grace = 5;
      grace2count = 0;
//      setup();
    }
    if (grace2count > 0) grace2count = grace2count + 1;
  }
  else grace--;
  // date & time consistency check - end
  
  // save current timestamp (old...[0]) into previous timestamp (old...[1]) for next loop (timestamp consistancy check)
  oldyear[1] = oldyear[0];
  oldmonth[1] = oldmonth[0];
  oldday[1] = oldday[0];
  oldhour[1] = oldhour[0];
  oldminute[1] = oldminute[0];
  oldsecond[1] = oldsecond[0];
  
  // once a day at midnight: set DCF77 time to RTC
  // oldtime = previous day the time and date were saved to RTC (with battery backup & power outage proof)
  if (oldday[0] != oldtime) {
    rtc.adjust(DateTime(oldyear[0], oldmonth[0], oldday[0], oldhour[0], oldminute[0], oldsecond[0]));
    if (SMON == 1) Serial.println(F("RTC set to current DCF time & date"));
    oldtime = oldday[0];
  }

  // get CO2 value from MH-Z14A sensor
  if (oldsecond[0] == 30) {
    ppm_PWM = gas_concentration_uart();
    Serial.print(F("CO2: "));
    Serial.println(ppm_PWM);
  }
  
  //LDR reading - luminosity
  LDRvalue = analogRead(LDRpin);
//  if (SMON == 1) {
//    Serial.print(F("LDRvalue = "));
//    Serial.println(LDRvalue);
//  }
  
  // ********** Automatic sleep mode & wake mode based on time and LDR value **********
  LDRaverage = (LDRaverage * 10 + LDRvalue) / 11;
  uint8_t t5 = getBCDlo(now.hour);
  uint8_t t6 = getBCDhi(now.hour);
  // check if hour is between 22 and 7 - nixie tubes and LEDs will shut off and clock will be silent from 23:00:00 until 06:59:59 while dark (sleep mode)  
  if ((LDRaverage < LDRmin) && (NT == 1) && (((t6 == 2) && (t5 > 2)) or ((t6 == 0) && (t5 < 7)))) {
    NT = 2;
    reason = 1;
    SleepNixieClock(reason);
  }
  // check light when clock is in sleep mode - clock will only wake automaticcaly when there is enough light  
  if ((LDRaverage >= LDRmin) && (NT == 2)) {
    reason = 1;
    tracknumber = 1;
    PlayTrack(tracknumber);
    WakeNixieClock(reason);
  }

  // Change colors if LEDS OP WANDEL is activated
  if (Evalue[4] == 1) {
    LEDchanged = 1; 
    LEDSONTHERUN();
  }
  
  // when LED color values are changed, send them out to the neopixels
  if (LEDchanged == 1) {
    LEDchanged = 0;  
      if ((LE == 1) or (LE == 3) or ((rgbw[1][0] == 0) && (rgbw[1][1] == 0) && (rgbw[1][2] == 0) && (rgbw[1][3] == 0))){
        for(int i = 0 ; i < NEOPIXELNUM / 2 ; i++) {
          pixels1.setPixelColor(i, pixels1.Color(rgbw[1][0], rgbw[1][1], rgbw[1][2], rgbw[1][3]));
        }
      }
      if ((LE == 2) or (LE == 3) or ((rgbw[1][4] == 0) && (rgbw[1][5] == 0) && (rgbw[1][6] == 0) && (rgbw[1][7] == 0))){
        for(int i = NEOPIXELNUM / 2 ; i < NEOPIXELNUM ; i++) {
          pixels1.setPixelColor(i, pixels1.Color(rgbw[1][4], rgbw[1][5], rgbw[1][6], rgbw[1][7]));
        }
        pixels1.show();   // Send the updated pixel colors to the hardware.
      }
  }

  // ESP communication treatment - BEGIN -
  // ESP communication treatment. Message types:
  // 'S' - status sent to smartphone: status of Nixie tubes and LEDS & values of all RGBW leds & E3 ON/OFF & on/off SOUND
  // 'T' - treatment of input values from remote control for the Top LEDS
  // 'B' - treatment of input values from remote control for the Bottom LEDS
  // 'O' - LEDS, nixie tubes, backlight and sound ON/OFF switch
  // 'E' - treatment of EXTRA menu
  // 'C' - copy TOP to BOTTOM checkbox
  // 'G' - Test sound
  // 'R' - Report time/date errors to ESP
  
  if (Serial3.available() > 0) {
    ESPflag = Serial3.read(); 
    // examine what the inputbuffer 'ESPflag' contains
    if (ESPflag == 'S')
    { 
      if (SMON == 1) Serial.println(F("S-command received"));
      // Smartphone request status of all colors & ON/OFF status of Nixietubes and LEDs,etc.
      // prepare commandstring for transmission (convert int values into ascii)
      if (LE == 0) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[2][0], rgbw[2][1], rgbw[2][2], rgbw[2][3], rgbw[2][4], rgbw[2][5], rgbw[2][6], rgbw[2][7], Evalue[4], CTB, Evalue[2]);
      if (LE == 1) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[1][0], rgbw[1][1], rgbw[1][2], rgbw[1][3], rgbw[2][4], rgbw[2][5], rgbw[2][6], rgbw[2][7], Evalue[4], CTB, Evalue[2]);
      if (LE == 2) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[2][0], rgbw[2][1], rgbw[2][2], rgbw[2][3], rgbw[1][4], rgbw[1][5], rgbw[1][6], rgbw[1][7], Evalue[4], CTB, Evalue[2]);
      if (LE == 3) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[1][0], rgbw[1][1], rgbw[1][2], rgbw[1][3], rgbw[1][4], rgbw[1][5], rgbw[1][6], rgbw[1][7], Evalue[4], CTB, Evalue[2]);
      if (SMON == 1)  {
        Serial.print(F("ALL settings sent to smartphone: "));
        Serial.println(buffer);
      }
      Serial3.write(buffer);
      Etrans = 0;
    } 
    else if (ESPflag == 'T')
    {
      LEDchanged = 1; 
      if (SMON == 1) Serial.println(F("T-command received - change in settings TOP LEDS"));
      // Smartphone sends new status for LED string (settings changed by smartphone)
      // Format message: 'Txyyy' Fixed lenght of 'T' + 4 digits
      // x = R, G, B or W (Red, Green, Blue, White)
      // TOP RED  linked to TR
      // TOP GREEN linked to TG
      // TOP BLUE linked to TB
      // TOP WHITE linked to TW
  
      // read color
      char color = getcharacter();
      if (color == 'R') {
        rgbw[1][0] = getnextdata();
  //        fram.write8(1*8+0+32, rgbw[1][0]);
        if (SMON == 1) {
          Serial.print(F("NTR: "));
          Serial.println(rgbw[1][0]);
        }
      }
      else if (color == 'G') {
        rgbw[1][1] = getnextdata();
  //        fram.write8(1*8+1+32, rgbw[1][1]);
        if (SMON == 1)  {
          Serial.print(F("NTG: "));
          Serial.println(rgbw[1][1]);
        }
      }
      else if (color == 'B') {
        rgbw[1][2] = getnextdata();
  //        fram.write8(1*8+2+32, rgbw[1][2]);
        if (SMON == 1) {
          Serial.print(F("NTB: "));
          Serial.println(rgbw[1][2]);
        }
      }
      else if (color == 'W') {
        rgbw[1][3] = getnextdata();
  //        fram.write8(1*8+3+32, rgbw[1][3]);
        if (SMON == 1) {
          Serial.print(F("NTW: "));
          Serial.println(rgbw[1][3]);
        }
      }
    Serial3.println(F("T-Done"));
    } 

    else if (ESPflag == 'B')
    { 
      LEDchanged = 1; 
      if (SMON == 1) Serial.println(F("B-command received - change in settings BOTTOM LEDS"));
      // Smartphone sends new status for LED string (settings changed by smartphone)
      // Format message: 'Bxyyy' Fixed lenght of 'B' + 4 digits
      // x = R, G, B or W (Red, Green, Blue, White)
      // yyy = value between '000' and '255'
      // BOTTOM RED linked to BR
      // BOTTOM GREEN linked to BG
      // BOTTOM BLUE linked to BB
      // BOTTOM WHITE linked to BW
  
      // read color
      char color = getcharacter();
      if (color == 'R') {
        rgbw[1][4] = getnextdata();
  //        fram.write8(1*8+4+32, rgbw[1][4]);
        if (SMON == 1) {
          Serial.print(F("NBR: "));
          Serial.println(rgbw[1][4]);
        }
      }
      else if (color == 'G') {
        rgbw[1][5] = getnextdata();
  //        fram.write8(1*8+5+32, rgbw[1][5]);
        if (SMON == 1) {
          Serial.print(F("NBG: "));
          Serial.println(rgbw[1][5]);
        }
      }
      else if (color == 'B') {
        rgbw[1][6] = getnextdata();
  //        fram.write8(1*8+6+32, rgbw[1][6]);
        if (SMON == 1) {
          Serial.print(F("NBB: "));
          Serial.println(rgbw[1][6]);
        }
      }
      else if (color == 'W') {
        rgbw[1][7] = getnextdata();
  //        fram.write8(1*8+7+32, rgbw[1][7]);
        if (SMON == 1) {
          Serial.print(F("NBW: "));
          Serial.println(rgbw[1][7]);
        }
      }
    Serial3.println(F("B-Done"));
    } 

    else if (ESPflag == 'O')
    {
      if (SMON == 1) Serial.println(F("O-command received - change in ON/OFF settings"));
      // Smartphone sends new O-command and Nixie Tube replies with new status after change: O,t,u,n,m,r,s
      //    t = NT, 1=ON & 0=OFF
      //    u = LE, 0=Top & Bottom OFF, 1=Top ON, 2=Bottom ON, 3 Top & Bottom ON, values + 4=saved status when NT=0
      //    m = SND, 1=ON & 0=OFF
      //    s = SMON, 1=ON & 0=OFF
      // Format received message: 'Ont' - Fixed lenght of 'O' + 2 digits
      // n = 1 (Nixie Tubes & LEDS), 2 (LEDS Top alone), 3 (LEDS Bottom alone), 5 (SND), 7 (SMON)
      // t = 1 or 0 - 1=ON & 0=OFF
  
      // read n
      char n = getcharacter();
      if (n == '1') {     // NT = Nixie Tubes
        onoff = get1data();
        if (SMON == 1) {
          Serial.print(F("Nixie Tubes - on/off = "));
          Serial.println(onoff);
        }
        // Request ON/OFF of nixie tubes = mute/unmute all activities of the clock
        // in case of darkness when clock is in sleep mode - LDRaverage is lower then minimal light (LDRmin) and on-command is issued from smartphone
        // Then LDRaverage is set to 1000. This will light up de nixietubes for only 10-20 seconds.
        if ((LDRaverage < LDRmin) && (onoff == 1)) LDRaverage = 1000;
        NT = onoff;
  //        fram.write8(66, NT);
        reason = 2;
        if (NT == 0) {
          // Nixie tubes AND LEDs off & all activity stops = clock sleep status (ticking goes on)
          // This status can only be temporarely broken by the ON command of the nixie tubes
          SleepNixieClock(reason);
        }
        else {
          tracknumber = 1;
          PlayTrack(tracknumber);
          WakeNixieClock(reason);
        }
      }
      else if (n == '2') {     // LE = LEDs Top
        LEDchanged = 1; 
        onoff = get1data();
        if (SMON == 1) {
          Serial.print(F("LEDs Top - on/off = "));
          Serial.println(onoff);
        }
        if ((onoff == 1) && ((LE == 0) || (LE == 2))) LE = LE + 1;
        if ((onoff == 0) && ((LE == 1) || (LE == 3))) LE = LE - 1;
  //      fram.write8(67, LE);
        setLEDS();
      }
      else if (n == '3') {     // LB = LEDs Bottom
        LEDchanged = 1; 
        onoff = get1data();
        if (SMON == 1) {
          Serial.print(F("LEDs Bottom - on/off = "));
          Serial.println(onoff);
        }
        if ((onoff == 1) && ((LE == 0) || (LE == 1))) LE = LE + 2;
        if ((onoff == 0) && ((LE == 2) || (LE == 3))) LE = LE - 2;
  //      fram.write8(67, LE);
        setLEDS();
      }
      else if (n == '4') {     // SND: Sound
        onoff = get1data();
        if (SMON == 1) {
          Serial.print(F("Sound - on/off = "));
          Serial.println(onoff);
        }
        SND = onoff;
  //      fram.write8(64, SND);
      }

      else if (n == '7') {     // Serial monitor: SMON ON-OFF
        onoff = get1data();
        if (SMON == 1) {
          Serial.print(F("Serial Monitor - on/off = "));
          Serial.println(onoff);
        }
        SMON = onoff;
        fram.write8(70, SMON);
      }

//      if (reason != 2) {
        // prepare commandstring for transmission (convert int values into ascii)
        int CX = snprintf ( buffer, 100, "O,%d,%d,%d,%d", NT, LE, SND, SMON);
        Serial3.write(buffer);
        if (SMON == 1) {
          Serial.print(F("Reply on O command to smartphone: "));
          Serial.println(buffer);
        }
//      }
    } 
  
    else if (ESPflag == 'E')
    {
      Serial3.print(ESPflag);
      if (SMON == 1) Serial.print(ESPflag);
//      if (SMON == 1) Serial.println(F("E-command received - EXTRA command received"));
      // Smartphone sends new E-command and Nixie Tube replies with confimation on specific (persistent) commands (ON/OFF)
      // Format received message: 'En' - Fixed lenght of 'E' + 1 digit
      // n = 0 = TEMPORARY switch - Datum - formaat: DD MM JJ - 'countdowntime' seconden
      //     1 = TEMPORARY switch - Temperatuur & vochtigheid - formaat: temperatuur in 2 digtis links & vochtigheid in 2 digits rechts - 'countdowntime' seconden
      //     8 = TEMPORARY switch - Luchtdruk - formaat: luchtdruk in 3 of 4 digits rechts - 'countdowntime' seconden
      //     2 = PERSISTENT switch - Fade mode voor nixie tubes - overgang is 4/10de van een seconde - ook in te stellen via smartphone?
      //     3 = PERSISTENT switch - Slotmachine - bedoeld om de nixie tubes te 'healen' van degeneratie: snelwisselen van alle cijfers op alle tubes zodat er wave effect bekomen wordt
      //     4 = PERSISTENT switch - LEDS op de wandel - elke seconde worden de instellingen van de leds minimaal aagepast: boven en onder tegelijk en in tegenstelde richting.
      //         De vertrekinstelling wordt tijdelijk opgeslagen en teruggezet wanneer deze instelling terug wordt gedesactiveerd door een 2de 'O3' commando.
      //         Deze instelling wordt dus ook verder gezet wanneer de klok opnieuw wordt aangezet, indien deze optie actief stond.
      //     5 = TEMPORATY switch - LEDS op 'default' instellingen. Dit stopt automatisch commando E4.
      //     6 = PERSISTENT switch - (enkel actief op 25/12 op elk uur van de dag) Christmas
      //         - muziek afspelen (timing van de muziek volgen per seconde) ""We wish you a merry christmas (3x) ... and a happy new year" 
      //         - af en toe witte spikes door de leds genereren
      //         - de nixie tubes laten pinken.
      //         - wanneer het liedje bijna gedaan is (bij tekst 'happy new year') het nieuwe jaartal op 4 digits laten verschijnen: midden: "2 02 1" of rechts "20 21".
      //     7 = PERSISTENT switch - New year - zoiets als Christmas?
  
      // read n
      char n = getcharacter();
      if (SMON == 1) {
        Serial.print(n);
        Serial.println(F("-command received"));
      }

      if (n == '0') {
        // DATUM - laat dit 'countdowntime' seconden zien
        digit[0] = getBCDlo(now.year);
        digit[1] = getBCDhi(now.year);
        digit[2] = getBCDlo(now.month);
        digit[3] = getBCDhi(now.month);
        digit[4] = getBCDlo(now.day);
        digit[5] = getBCDhi(now.day);
        NixieDisplayAll();
        countdown = countdowntime;
//        touchcount = 1;
        grace = 5;
        Serial3.println(n);
      }
      else if (n == '1') {
        // TEMPERATUUR & VOCHTIGHEID - laat dit 7 seconden zien
        // get environment data: temperature, humidity & pressure
        temp = round(bme.readTemperature() * 0.82);
        humi = round(bme.readHumidity() * 1.55);
        
        digit[1] = temp / 10;
        digit[0] = temp - (digit[1] * 10);
        digit[2] = 15;
        digit[3] = 15;
        digit[5] = humi / 10;
        digit[4] = humi - (digit[5] * 10);
        NixieDisplayAll();
        countdown = countdowntime;
//        touchcount = 2;
        grace = 5;
        Serial3.println(n);
      }
      else if (n == '8') {
        // LUCHTDRUK - laat dit 7 seconden zien
        // get environment data: temperature, humidity & pressure
        pres = round(bme.readPressure() / 99.05F);

        digit[3] = pres / 1000;
        digit[2] = (pres - (digit[3] * 1000))/100;
        digit[1] = (pres - (digit[3] * 1000) - (digit[2] * 100)) / 10;
        digit[0] = pres - (digit[3] * 1000) - (digit[2] * 100) - (digit[1] * 10);
        digit[4] = 15;
        digit[5] = 15;
        if (digit[3] == 0) digit[3] = 15;
        NixieDisplayAll();
        countdown = countdowntime;
//        touchcount = 0;
        grace = 5;
        Serial3.println(n);
      }
      else if (n == '2') {
        // NIXIE TUBE FADE MODE - fade = 1: instant, fade = fadeSteps: fading - cross dissolve
        Enumber = 2;
        if (Evalue[Enumber] == 0) {
          Evalue[Enumber] = 1;
  //          fram.write8(Enumber+69, Evalue[Enumber]);
          fade = fadeSteps;
        }
        else {
          Evalue[Enumber] = 0;
  //          fram.write8(Enumber+69, Evalue[Enumber]);
          fade = 1;
        }
        Serial3.print(n);
        Serial3.println(Evalue[Enumber]);
      }
      else if (n == '3') {
        Serial3.print(n);
        Serial3.println(F("1"));
        // SLOTMACHINE
        slotmachine();
      }
      else if (n == '4') {
        // LEDS OP WANDEL
        Etrans = 1;
        Enumber = 4;
        if (Evalue[Enumber] == 0) {
          Evalue[Enumber] = 1;
  //          fram.write8(Enumber+69, Evalue[Enumber]);
          for (int8_t i = 0; i <= 7; i++) {
          rgbw[3][i] = rgbw[1][i]; //byte E3** = N**;
          Frgbw[i] = rgbw[1][i];
  //          fram.write8(3*8+i+32, rgbw[3][i]);
          }
          rgbw[1][3] = 0;
          Frgbw[3] = 0;
  //          fram.write8(1*8+3+32, rgbw[1][3]);
          rgbw[1][7] = 0;
          Frgbw[7] = 0;
  //          fram.write8(1*8+7+32, rgbw[1][7]);
        }
        else {
          Evalue[Enumber] = 0;
  //          fram.write8(Enumber+69, Evalue[Enumber]);
          LEDchanged = 1; 
          for (int8_t i = 0; i <= 7; i++) {
          rgbw[1][i] = rgbw[3][i]; //byte N** = E3**;
  //          fram.write8(1*8+i+32, rgbw[1][i]);
          }
        }
        Serial3.print(n);
        Serial3.println(Evalue[Enumber]);
      }
      else if (n == '5') {
        // LEDS DEFAULT KLEUR
        Etrans = 1;
        Enumber = 5;
        Evalue[Enumber] = 0;
  //        fram.write8(Enumber+69, Evalue[Enumber]);
        LEDchanged = 1; 
        for (int8_t i = 0; i <= 7; i++) {
          rgbw[1][i] = rgbw[0][i]; //byte N** = default;
  //        fram.write8(1*8+i+32, rgbw[1][i]);
        }
        Serial3.println(n);
      }
      else if (n == '6') {
        // KERSTDAG - code to write
        grace = 5;
        Serial3.println(n);
      }
      else if (n == '7') {
        // NIEUWJAAR - code to write
        grace = 5;
        Serial3.println(n);
      }
  
//      if (Etrans == 1) {
//        // prepare commandstring for conditionally transmission (convert int values into ascii)
//        // attention: Format is: Enn - NO seperators, just 3 charachters/numbers!!!
//        int CX = snprintf ( buffer, 100, "E%d%d", Enumber, Evalue[Enumber]);
//        if (SMON == 1) {
//          Serial.print(F("Reply on E command to smartphone: "));
//          Serial.println(buffer);
//        }
//        Serial3.write(buffer);
//        Etrans = 0;
//      }
    }
  
    else if (ESPflag == 'C')
    {
      if (SMON == 1) Serial.println(F("C-command received - copy TOP to BOTTOM checkbox"));
        // Smartphone sends new C-command
        // Format received message: 'Cn' - Fixed lenght of 'C' + 1 digit
        // n = 0 = checkbox OFF
        //     1 = checkbox ON
        
        // read n
        CTB = get1data();
      //    fram.write8(68, CTB);
      Serial3.println(CTB);
    }

    else if (ESPflag == 'G')
    {
    if (SMON == 1) Serial.println(F("G-command received - test sound on nixie tubes"));
      // Smartphone sends new G-command
      // Format received message: 'Gnnn' - Fixed lenght of 'G' followed by a number between 0 and 999
      // Remark: for now only values from 0 up to 19 wille be validated
      tracknumber = getnextdata();
      if ((tracknumber >= 0) and (tracknumber <= 19)) {
      reason = 0;
      tracknumber = 1;
      PlayTrack(tracknumber);
      }
    Serial3.println(tracknumber);
    }

    else if (ESPflag == 'R')
    {
      if (SMON == 1) Serial.println(F("R-command received - Report errors from DCF77 - FRAM reading"));
      // Smartphone sends new R-command
      // Format received message: 'Rn' - Fixed lenght of 'R' + 1 digit
      // n = 0 = RESET report index to 160 - clear report array
      //     1 = Report first entry from the FRAM report array - SHOW, start address = 160, end = address on location 144 & 145
      //     2 = Report next entry from the FRAM report array - MORE (or NEXT?)
      //     R = Resend last report
      // get location in FRAM to put the data

      // read n
      char n = getcharacter();
      if (n == 'R') {
        if (fram_index > 175) fram_index = fram_index - 16;
        ReportToBT();
      }
      else if (n == '0') {
        // reset report index to 160 - ="clear report array"
        fram.write8(144,0);
        fram.write8(145,160);
        Serial3.println(F("R0"));
      }
      else if (n == '1') {
        // "SHOW" pressed on BT - set fram_index to 160 (start of the table in fram)
        // Read the first entry in the table
        // Last entry to read = next free entry in the table: fram locations 144 & 145 - initially = 160 and incremented by 16 for every entry used.
        // 1 entry used means that this pointer contains 176, or = 160 + 16
        fram_index = 160;
        fram_index_last = (fram.read8(144) * 256) + fram.read8(145);
        ReportToBT();
      }
      else if (n == '2') {
        // "MORE" pressed on BT
        // Read the next entry in the table - pointer calculated in ReportToBT() function: fram_index incremented by 16 each call.
        fram_index_last = (fram.read8(144) * 256) + fram.read8(145);
        ReportToBT();
      }
    }

    else if(not ESPflag == 0)
    { 
      if (SMON == 1) {
        Serial.print(F("Unknown command: ")); Serial.println(ESPflag);
      }
    } 
  }
  // reset ESPflag;
  ESPflag = 0; 
  // ESP communication treatment - END -
  NixieDisplayAll();

}
// ********** MAIN LOOP - END **********


// ********** REPORTTOBT - start **********
void ReportToBT() {
// read 1 entry in the Report table in fram and format to be send to BT as timestamp: YYYY-MM-DD  hh:mm:ss
//if (fram_index < fram_index_last) {
  if (fram.read8(fram_index) != 0) {
    YY_exp = (fram.read8(fram_index) * 100) + fram.read8(fram_index + 1);
    MM_exp = fram.read8(fram_index + 2);
    padded2Digits(MM_exp0, MM_exp);
    DD_exp = fram.read8(fram_index + 3);
    padded2Digits(DD_exp0, DD_exp);
    hh_exp = fram.read8(fram_index + 4);
    padded2Digits(hh_exp0, hh_exp);
    mm_exp = fram.read8(fram_index + 5);
    padded2Digits(mm_exp0, mm_exp);
    ss_exp = fram.read8(fram_index + 6);
    padded2Digits(ss_exp0, ss_exp);
    YY_DCF77 = (fram.read8(fram_index + 7) * 100) + fram.read8(fram_index + 8);
    MM_DCF77 = fram.read8(fram_index + 9);
    padded2Digits(MM_DCF770, MM_DCF77);
    DD_DCF77 = fram.read8(fram_index + 10);
    padded2Digits(DD_DCF770, DD_DCF77);
    hh_DCF77 = fram.read8(fram_index + 11);
    padded2Digits(hh_DCF770, hh_DCF77);
    mm_DCF77 = fram.read8(fram_index + 12);
    padded2Digits(mm_DCF770, mm_DCF77);
    ss_DCF77 = fram.read8(fram_index + 13);
    padded2Digits(ss_DCF770, ss_DCF77);
    fram_index = fram_index + 16;
    if (fram_index >= fram_index_last) {
      int CX = snprintf ( buffer, 100, "R,%d-%d%d-%d%d  %d%d:%d%d:%d%d,%d-%d%d-%d%d  %d%d:%d%d:%d%d,SHOW", YY_exp, MM_exp0, MM_exp, DD_exp0, DD_exp, hh_exp0, hh_exp, mm_exp0, mm_exp, ss_exp0, ss_exp, YY_DCF77, MM_DCF770, MM_DCF77, DD_DCF770, DD_DCF77, hh_DCF770, hh_DCF77, mm_DCF770, mm_DCF77, ss_DCF770, ss_DCF77);
    }
    else {
      int CX = snprintf ( buffer, 100, "R,%d-%d%d-%d%d  %d%d:%d%d:%d%d,%d-%d%d-%d%d  %d%d:%d%d:%d%d,MORE", YY_exp, MM_exp0, MM_exp, DD_exp0, DD_exp, hh_exp0, hh_exp, mm_exp0, mm_exp, ss_exp0, ss_exp, YY_DCF77, MM_DCF770, MM_DCF77, DD_DCF770, DD_DCF77, hh_DCF770, hh_DCF77, mm_DCF770, mm_DCF77, ss_DCF770, ss_DCF77);
    }
    Serial3.write(buffer);
    if (SMON == 1) Serial.print("Report sent to smartphone: ");
    if (SMON == 1) Serial.println(buffer);
  }
  else {
    int CX = snprintf ( buffer, 100, "R,%d-%d%d-%d%d  %d%d:%d%d:%d%d,%d-%d%d-%d%d  %d%d:%d%d:%d%d,SHOW", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    Serial3.write(buffer);
  }
}
// ********** REPORTTOBT - end **********


// ********** CALCREFERENCEDATESTAMP - start **********
void CalcReferenceDatestamp (uint8_t i, uint8_t j)
{
  oldsecond[j] = oldsecond[i] + 1;
  oldminute[j] = oldminute[i];
  oldhour[j] = oldhour[i];
  oldday[j] = oldday[i];
  oldmonth[j] = oldmonth[i];
  oldyear[j] = oldyear[i];

  if (oldsecond[j] == 60) {
    oldsecond[j] = 0;
    oldminute[j] = oldminute[j] +1;
  }
  if (oldminute[j] == 60) {
    oldminute[j] = 0;
    oldhour[j] = oldhour[j] + 1;
  }
  if (oldhour[j] == 24) {
    oldhour[j] = 0;
    oldday[j] = oldday[j] + 1;
  }
    if (oldday[j]== 32) {
    oldday[j] = 1;
    oldmonth[j] = oldmonth[j] + 1;
  }
    if ((oldday[j] == 31) && ((oldmonth[j] == 4) || (oldmonth[j] == 6) || (oldmonth[j]== 9) || (oldmonth[j] == 11)))  {
    oldday[j] = 1;
    oldmonth[j] = oldmonth[j] + 1;
  }
    if ((oldday[j] == 30) && (oldmonth[j] == 2))   {
    oldday[j] = 1;
    oldmonth[j] = oldmonth[j] + 1;
  }
    if ((oldday[j] == 29) && (oldmonth[j] == 2) && ((oldyear[j]/ 4) * 4 != oldyear[j]))   {
    oldday[j] = 1;
    oldmonth[j]= oldmonth[j] + 1;
  }
  if (oldmonth[j] == 13) {
    oldmonth[j] = 1;
    oldyear[j]= oldyear[j] + 1;
  }
}
// ********** CALCREFERENCEDATESTAMP - end **********

// ********** SLOTMACHINE - start **********
void slotmachine()
{
  if (SMON == 1) {
    Serial.print(F("Reply on E command to smartphone: "));
    Serial.println("E31");
  }

    // cathode sequence to follow: 1, 0, 2, 3, 9, 4, 8, 5, 7, 6
    // First digit 1 on all tubes with a small delay from left to right, then the next digit, etc. After digit 6 start all over again.
    // 60 digits per 6 to display in 1 cycle. Delay is less then 1000ms/60 = 16,6ms. Take 16ms as a start
    // parameters:
    // - cycles
    // - delayms
    
    int delayT = (delayms * 100) + 59;
  for (int8_t k = 0; k <= cycles - 1; k++)
  {
  // forward
    for (int8_t j = 0; j <= 8; j++)
    {
      for (int8_t i = 5; i >= 1; i--)
      {
        digit[i] = digit[i-1];
        NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
        delay(delayT);       
      }
      digit[0] = cathode[j];
      NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
      delay(delayT);       
    }
  // reverse
    for (int8_t j = 9; j >= 1; j--)
    {
      for (int8_t i = 5; i >= 1; i--)
      {
        digit[i] = digit[i-1];
        NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
        delay(delayT);       
      }
      digit[0] = cathode[j];
      NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
      delay(delayT);       
    }

  }
// set display to blank
  for (int8_t d = 5; d >= 0; d--) {
    digit[d]=15;
  }
  NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
  Serial3.print("E30");
  if (SMON == 1) {
    Serial.print(F("Reply on E command to smartphone: "));
    Serial.println("E30");
  }
  grace = 5;
}
// ********** SLOTMACHINE - end **********

// ********** NixieDisplayAll - start **********
// Nixie driver - all 6 digits are used
// Use of transition mode - default, fade = 1, 1 = instant, >1 = cross dissolve (fade = fadeSteps) - can be set by BT
void NixieDisplayAll()
{
//          Serial.println("NixieDisplayAll... ");
//          Serial.print("fade = ");Serial.println(fade);
// Shut off nixie tubes - sleep mode - set all digits to blank (value 15) when NT differs from 1
//Serial.print("NT = ");Serial.println(NT);
  if (NT != 1) {
    for (int8_t d = 5; d >= 0; d--) {
      digit[d] = 15;  
    }
  }
  // fade = 1: instant mode
  if (fade == 1) {
    NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
  }
  // fade > 1: fade mode - fade in n steps - last step is new digits
  // Parameters: to be controlled via BT and saved in FRAM
  // - fade = number of fade steps between olddigit to digit
  // - fadeDelay - max delay per cycle
  // - fadeCycle - devider number of cycles (ie = 5)
  // - fadePass - passes to the next cycle (is calculated based on fadeCycle: FadeCycle / 2)
  else {
    for (int8_t k = 0; k < fade/fadeCycle*fadePass; k++) {
      NixieDisplayAll2(olddigit[0], olddigit[1], olddigit[2], olddigit[3], olddigit[4], olddigit[5]);
      delay(fadeDelay/(k + 1));
    }
    for (int8_t k = fade/fadeCycle*fadePass; k < fade/fadeCycle*(fadePass+1); k++) {
      NixieDisplayAll2(olddigit[0], olddigit[1], olddigit[2], olddigit[3], olddigit[4], olddigit[5]);
      delay(fadeDelay/(k + 1));

      NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
      delay(fadeDelay / (fade - k));
    }
    for (int8_t k = fade/fadeCycle*(fadePass+1); k < fade; k++) {
      NixieDisplayAll2(digit[0], digit[1], digit[2], digit[3], digit[4], digit[5]);
      delay(fadeDelay / (fade - k));
    }
  }
  for (int8_t d = 5; d >= 0; d--) {
    olddigit[d] = digit[d];
  }
}
// ********** NixieDisplayAll - end **********

// ********** NixieDisplayAll2 - start **********
// Nixie driver - Function with optional parameters - all 6 digits are used in this sketch
void NixieDisplayAll2(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5, uint8_t digit6)
{
//          Serial.println("NixieDisplayAll2... ");
  StartShiftOutData(EN3_PIN, DIN3_PIN, CLK3_PIN);
  ShiftOutData(digit6, DIN3_PIN, CLK3_PIN);
  ShiftOutData(digit5, DIN3_PIN, CLK3_PIN);
  EndShiftOutData(EN3_PIN);
  StartShiftOutData(EN2_PIN, DIN2_PIN, CLK2_PIN);
  ShiftOutData(digit4, DIN2_PIN, CLK2_PIN);
  ShiftOutData(digit3, DIN2_PIN, CLK2_PIN);
  EndShiftOutData(EN2_PIN);
  StartShiftOutData(EN1_PIN, DIN1_PIN, CLK1_PIN);
  ShiftOutData(digit2, DIN1_PIN, CLK1_PIN);
  ShiftOutData(digit1, DIN1_PIN, CLK1_PIN);
  EndShiftOutData(EN1_PIN);
}
// ********** NixieDisplayAll2 - end **********

// ********** StartShiftOutData: initialize shift procedure - start **********
void StartShiftOutData(int EN_PIN, int DIN_PIN, int CLK_PIN)
{
//          Serial.println("StartShiftOutData... ");
  // Ground EN pin and hold low for as long as you are transmitting
  digitalWrite(EN_PIN, 0); 
  // Clear everything out just in case to
  // prepare shift register for bit shifting
  digitalWrite(DIN_PIN, 0);
  digitalWrite(CLK_PIN, 0);  
}
// ********** StartShiftOutData - end **********

// ********** ShiftOutData: shift data on the shift registers - start **********
void ShiftOutData(uint8_t digitout, int DIN_PIN, int CLK_PIN)
{
//          Serial.println("ShiftOutData... ");
  // Send data to the nixie drivers 
  for (int8_t i = 15; i >= 0; i--) {
    // Set high only the bit that corresponds to the current nixie digit
    if(i == digitout) {
      digitalWrite(DIN_PIN, 1); 
    }
    else digitalWrite(DIN_PIN, 0);
    
    // Register shifts bits on upstroke of CLK pin 
    digitalWrite(CLK_PIN, 1);
    // Set low the data pin after shift to prevent bleed through
    digitalWrite(CLK_PIN, 0);  
  }   
}
// ********** ShiftOutData - start **********

// ********** EndShiftOutData: disable shifting action - start **********
void EndShiftOutData(int EN_PIN)
{
//          Serial.println("EndShiftOutData... ");
  // Return the EN pin high to signal chip that it 
  // no longer needs to listen for data
  digitalWrite(EN_PIN, 1);
}
// ********** EndShiftOutData - start **********

// ********** Wake Nixie Clock - start **********
void WakeNixieClock(uint8_t reason) {
  NT = 1;
//    fram.write8(66, NT);
  if (LE > 3) {
    LE = LE - 4;
//      fram.write8(67, LE);
    LEDchanged = 1; 
    setLEDS();
  }
  if (SND == 2) {
    SND = 1;
//      fram.write8(64, SND);
  }
  // resume LEDS OP WANDEL when active
  Enumber = 4;
  if (Evalue[Enumber] == 2) Evalue[Enumber] = 1;
  
  // prepare commandstring for transmission (convert int values into ascii)
  if (LE == 0) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[2][0], rgbw[2][1], rgbw[2][2], rgbw[2][3], rgbw[2][4], rgbw[2][5], rgbw[2][6], rgbw[2][7], Evalue[4], CTB, Evalue[2]);
  if (LE == 1) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[1][0], rgbw[1][1], rgbw[1][2], rgbw[1][3], rgbw[2][4], rgbw[2][5], rgbw[2][6], rgbw[2][7], Evalue[4], CTB, Evalue[2]);
  if (LE == 2) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[2][0], rgbw[2][1], rgbw[2][2], rgbw[2][3], rgbw[1][4], rgbw[1][5], rgbw[1][6], rgbw[1][7], Evalue[4], CTB, Evalue[2]);
  if (LE == 3) int CX = snprintf ( buffer, 100, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", NT, LE, SND, SMON, rgbw[1][0], rgbw[1][1], rgbw[1][2], rgbw[1][3], rgbw[1][4], rgbw[1][5], rgbw[1][6], rgbw[1][7], Evalue[4], CTB, Evalue[2]);
  Serial3.write(buffer);
  if (SMON == 1) {
    if (reason == 1) Serial.print(F("New status sent to smartphone to reflect 'automatic wake-up after light detection': "));
    if (reason == 2) Serial.print(F("New status sent to smartphone to reflect 'set manual wake-up': "));
    if (reason == 3) Serial.print(F("New status sent to smartphone after frame-TOUCH when nixie clock was OFF: "));
    Serial.println(buffer);
  }
}
// ********** Wake Nixie Clock - end **********

// ********** PlayTrack - start **********
void PlayTrack (uint8_t tracknumber) {

    // wake amplifiers - song is to be played
    musicPlayer.GPIO_digitalWrite(7, HIGH);
    // play song
//    musicPlayer.playFullFile(songs[tracknumber]);
    musicPlayer.startPlayingFile(songs[tracknumber]);

    // set variable 'playing' to indicate that a song is playing - serves as reference to shut down the amplifiers when the songs stops
    playing = 1;
}
// ********** PlayTrack - end **********

// ********** Sleep Nixie Clock - start **********
void SleepNixieClock(uint8_t reason) {
//    fram.write8(66, NT);
  if (LE < 4) {
    LE = LE + 4;
//      fram.write8(67, LE);
    LEDchanged = 1;
    setLEDS();
  }
  if (SND == 1) {
    SND = 2;
//      fram.write8(64, SND);
  }
  // suspend LEDS OP WANDEL when active
  Enumber = 4;
  if (Evalue[Enumber] == 1) Evalue[Enumber] = 2;

  // prepare commandstring for transmission (convert int values into ascii)
  int CX = snprintf ( buffer, 100, "O,%d,%d,%d,%d", NT, LE, SND, SMON);
  Serial3.write(buffer);
  if (SMON == 1) {
    if (reason == 1) Serial.print(F("New status to reflect 'automatic sleep mode after light detection' - sent to smartphone: "));
    if (reason == 2) Serial.print(F("New status to reflect 'set manual to sleep' - sent to smartphone: "));
    Serial.println(buffer);
  }
}
// ********** Sleep Nixie Clock - end **********

// ********** gas_concentration_uart - start **********
uint16_t gas_concentration_uart() {

//  Calibrate command:
//  byte addArray[] = {0xFF, 0x01, 0x88, 0x07, 0xD0, 0x00, 0x00, 0x00, 0xA0};
  byte addArray[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  char dataValue[9];

  SerialCom.write(addArray, 9);
  SerialCom.readBytes(dataValue, 9);

  uint8_t resHigh = (uint8_t) dataValue[2];
//  Serial.print(F("resHigh: "));
//  Serial.print(resHigh);
  uint8_t resLow  = (uint8_t) dataValue[3];
//  Serial.print(F("  resLow: "));
//  Serial.println(resLow);
  uint16_t ppm = (resHigh*256) + resLow;

  return ppm;
}
// ********** gas_concentration_uart - end **********

// ********** gas_callibration_uart - start **********
uint16_t gas_callibration_uart() {

//  Calibrate command:
  byte addArray[] = {0xFF, 0x01, 0x88, 0x07, 0xD0, 0x00, 0x00, 0x00, 0xA0};

  SerialCom.write(addArray, 9);
}
