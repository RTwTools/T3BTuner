/*
  DABDUINO basic example 2s
  DABDUINO is DAB+ digital radio shield for Arduino
  Created by Tomas Urbanek, Montyho technology Ltd., Januar 2, 2017.
  www.dabduino.com
*/

#include <SoftwareSerial.h>
#include "DABDUINO.h"

#define SERIAL_PORT Serial1
#define RESET_PIN 7
#define DAC_MUTE_PIN 9
#define SPI_CS_PIN 10

DABDUINO dab = DABDUINO(&SERIAL_PORT, STREAM_HARDWARE, RESET_PIN, DAC_MUTE_PIN, SPI_CS_PIN);

// DAB variables
char dabText[DAB_MAX_TEXT_LENGTH];



void setup() {
}

void loop() {
}




