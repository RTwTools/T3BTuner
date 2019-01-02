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

T3BTuner tuner = T3BTuner(&SERIAL_PORT, StreamType::HardwareSerial, RESET_PIN, DAC_MUTE_PIN, SPI_CS_PIN);

// DAB variables
char dabText[DAB_MAX_TEXT_LENGTH];
uint32_t programsIndex = 0;
uint32_t programIndex = 0;

void setup() {

  Serial.begin(57600);

  Serial.println("DAB RESET & START");
  tuner.init();
  if (!tuner.Reset()) {
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
  tuner.DabSearch();

  TunerState status;
  TunerState lastStatus;

  while (true)
  {
    tuner.State(&status);
    if (status != lastStatus) {
      Serial.println();
      switch (status)
      {
      case TunerState::Playing:
        Serial.print("Playing");
        break;
      case TunerState::Searching:
        Serial.print("Searching");
        break;
      case TunerState::Tuning:
        Serial.print("Tuning");
        break;
      case TunerState::Stopped:
        Serial.print("Stopped");
        break;
      case TunerState::Sorting:
        Serial.print("Sorting");
        break;
      case TunerState::Reconfiguring:
        Serial.print("Reconfiguring");
        break;
      default:
        break;
      }
    }
    if (status == TunerState::Playing || status == TunerState::Stopped) break;
    lastStatus = status;
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");

  tuner.NowPlaying(&programsIndex);
  Serial.println("Available programs: ");
  for (uint32_t i = 0; i <= programsIndex; i++) {
    if (tuner.DabStationName(i, dabText)) {
      Serial.print(i);
      Serial.print("\t ");
      Serial.println(dabText);
    }
  }
  Serial.println();

  if (tuner.AudioOutput()) { // 1st = spdiv (optical), 2st = cinch (analog)
    Serial.println("Set audio output");
  }

  if (tuner.VolumeSet(8)) { // Set volume: 0..16
    Serial.println("Set volume");
  }

  programIndex = 0;

  if (tuner.Notifications(true)) {
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

    if (tuner.PlayDab(programIndex)) {
      if (tuner.DabStationName(programIndex, dabText)) {
        Serial.print("Tuned program: (");
        Serial.print(programIndex);
        Serial.print(") ");
        Serial.println(dabText);
      }
    }
  }

  // EVENTS
  // EVENT TYP: 1=scan finish, 2=got new DAB program text, 3=DAB reconfiguration, 4=DAB channel list order change, 5=RDS group, 6=Got new FM radio text, 7=Return the scanning frequency /FM/
  if (tuner.isEvent()) {

    switch (tuner.readEvent()) {
      case 1:
        Serial.println("DAB program search finished.");
        break;
      case 2:
        //do something when New DAB progam text
        if (tuner.DabStationText(dabText)) { // new text
          Serial.print("DAB text event: ");
          Serial.println(dabText);
        }
        break;
    }
  }
}
