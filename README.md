# DCF77-Nixie-Clock
DCF77 clock with 6 nixie tubes, ESP8266 Wemos D1 mini, sound module, FRAM, RTC &amp; light, CO2, temperature, pressure and humidity sensors

Main objective of this project is to make a clock and to glorify old technologies like nixie tubes and DCF77 precision time keeping over radio signals.
The central 'motor' of this project is an Arduino mega pro mini (for is size/number of pins ratio and the capability for running the DCF77 library of Udo Klein - you can find that on Github too).
A secondary 'motor' is added to manage wifi acces and to collect internet data that is used for world clock functions
Next to this a number of sensors are integrated:
- Temperature, humidty and pressure sensor (BME280)
- CO2 sensor (MH-Z14A)
- Light sensor (ldr - light dependant resistor)
Finally 2 more functionalities are integrated:
- RTC (for accuratly display time at startup) with a big capacitor - several days of power down are possible
- FRAM (for storing startup variables and storing log data from the clock: DCF77 abnormal behaviour)

The clock can be remote controlled by a smartphone. The app is develloped on the "MIT app inventor" platform.
The remote controls the display functions of the clock and the world clock functionality (access to internet required). Still to develop: configuring the different variables that are stored in FRAM.

REMAINING ISSUES:
- communication between Arduino and ESP8266 Wemos D1 mini only works from ESP8266 to Arduino. I can't figure out what's the problem.
I have another clock project (with LED's) with the same type Arduino and ESP8266 where the communication is no issue at all. That project also contains DCF77, the same sensors, RTC and FRAM. The main difference are the nixie tubes. The code used on that ESP8266 is the same and most of the Arduino code is that too. Anyway ...
I already tried to use other pins on the Arduino mega, I tried software serial and another hardware serial (Serial2). I tried another ESP8266 ... still and always the same problem.
I also tested the communication over wifi between ESP8266 and my smartphone. That works fine.

Files:
- sketch Arduino: Simple_Clock_Ivan_V2.0_1.ino
- sketch ESP8266: ESP8266_wifi_Nixie_v1.ino

The sketch on the ESP8266 works OK as long as there is a reply on each incoming  request or command. This is the case for all E-type commands. The E41 command replies with a E41 answer, ie. Other commands like E3 react with 2 replies: E31 and at the end a E30. This sequence is not yet implemented in the ESP8266 code. I have to work on that. But the first reply should already arrive at the ESP8266, but it is not the case.
