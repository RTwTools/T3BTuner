/*
  DABDUINO basic example 1
  DABDUINO is DAB+ digital radio shield for Arduino
  Created by Tomas Urbanek, Montyho technology Ltd., Januar 2, 2017.
  www.dabduino.com
*/

#include <SoftwareSerial.h>
#include "T3BTuner.h"

#define SERIAL_PORT Serial1
#define RESET_PIN 7
#define DAC_MUTE_PIN 9
#define SPI_CS_PIN 10

DABDUINO dab = DABDUINO(&SERIAL_PORT, STREAM_HARDWARE, RESET_PIN, DAC_MUTE_PIN, SPI_CS_PIN);

// DAB variables
char dabText[DAB_MAX_TEXT_LENGTH];
uint32_t programsIndex = 0;
uint32_t programIndex = 0;

void setup() {

  Serial.begin(57600);

  Serial.println("DAB RESET & START");
  dab.init();
  if (!dab.reset()) {
    Serial.println("DAB NOT READY");
    while (1) {}
  }

  /*
    if(!dab.resetCleanDB()) {
    Serial.println("DAB NOT READY");
    while (1) {}
    }
  */

  Serial.println("DAB READY");

  Serial.print("Search for DAB programs:");
  dab.searchDAB(1);

  uint32_t status;
  uint32_t lastStatus;

  while (true) {
    dab.playStatus(&status);
    if (status != lastStatus) {
      Serial.println();
      switch (status) {
        case 0:
          Serial.print("Playing");
          break;
        case 1:
          Serial.print("Searching");
          break;
        case 2:
          Serial.print("Tuning");
          break;
        case 3:
          Serial.print("Stop");
          break;
        case 4:
          Serial.print("Sorting");
          break;
        case 5:
          Serial.print("Reconfiguration");
          break;
      }
    }
    if (status == 0 || status == 3) break;
    lastStatus = status;
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");

  dab.getProgramIndex(&programsIndex);
  Serial.println("Available programs: ");
  for (uint32_t i = 0; i <= programsIndex; i++) {
    if (dab.getProgramLongName(i, dabText)) {
      Serial.print(i);
      Serial.print("\t ");
      Serial.println(dabText);
    }
  }
  Serial.println();

  if (dab.setAudioOutput(true, true)) { // 1st = spdiv (optical), 2st = cinch (analog)
    Serial.println("Set audio output");
  }

  if (dab.setVolume(8)) { // Set volume: 0..16
    Serial.println("Set volume");
  }

  programIndex = 0;

  if (dab.eventNotificationEnable()) {
    Serial.println("Event notification enabled");
  }
}

void loop() {

  if (millis() % 20000 == 0) {

    if(programIndex < programsIndex) {
      programIndex++;  
    } else {
      programIndex = 0;
    }

    if (dab.playDAB(programIndex)) {
      if (dab.getProgramLongName(programIndex, dabText)) {
        Serial.print("Tuned program: (");
        Serial.print(programIndex);
        Serial.print(") ");
        Serial.println(dabText);
      }
    }
  }

  // EVENTS
  // EVENT TYP: 1=scan finish, 2=got new DAB program text, 3=DAB reconfiguration, 4=DAB channel list order change, 5=RDS group, 6=Got new FM radio text, 7=Return the scanning frequency /FM/
  if (dab.isEvent()) {

    switch (dab.readEvent()) {
      case 1:
        Serial.println("DAB program search finished.");
        break;
      case 2:
        //do something when New DAB progam text
        if (dab.getProgramText(dabText)) { // new text
          Serial.print("DAB text event: ");
          Serial.println(dabText);
        }
        break;
    }
  }
}
