/*
  DABDUINO basic example 1
  DABDUINO is DAB+ digital radio shield for Arduino
  Created by Tomas Urbanek, Montyho technology Ltd., Januar 2, 2017.
  www.dabduino.com
*/

#include "SerialStream.h"
#include "T3BTuner.h"
#include <SoftwareSerial.h>

#define SERIAL_TX 8
#define SERIAL_RX 9
#define PIN_RESET 3
#define PIN_MUTE 2

SoftwareSerial* serial = new SoftwareSerial(SERIAL_RX, SERIAL_TX);
ISerialStream* stream = new SerialStream(serial);
T3BTuner tuner = T3BTuner(stream, PIN_RESET, PIN_MUTE);
char buffer[T3BTunerMaxTextSize];
uint32_t stationCount = 0;
uint32_t stationId = 0;

void setup()
{
    Serial.begin(57600);
    Serial.println("DAB RESET & START");
    tuner.Init();

    if (!tuner.Reset())
    {
        Serial.println("DAB NOT READY");
        while (1)
        {
        }
    }

    Serial.println("DAB READY");
    Serial.print("Search for DAB programs:");
    tuner.DabSearch();

    TunerState status;
    TunerState lastStatus;

    while (true)
    {
        tuner.State(&status);
        if (status != lastStatus)
        {
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
            }
        }
        if (status == TunerState::Playing || status == TunerState::Stopped)
        {
            break;
        }
        lastStatus = status;
        Serial.print(".");
        delay(1000);
    }
    Serial.println("");

    tuner.DabStationCount(&stationCount);
    Serial.println("Available programs: ");
    for (uint32_t i = 0; i <= stationCount; i++)
    {
        if (tuner.DabStationName(i, buffer, sizeof(buffer)))
        {
            Serial.print(i);
            Serial.print("\t ");
            Serial.println(buffer);
        }
    }
    Serial.println();

    if (tuner.AudioOutput())
    {
        Serial.println("Set audio output");
    }

    if (tuner.VolumeSet(8))
    {
        Serial.println("Set volume");
    }

    stationId = 0;

    if (tuner.EventEnable(true))
    {
        Serial.println("Event notification enabled");
    }
}

void loop()
{
    if (millis() % 20000 == 0)
    {
        stationId = (stationId < stationCount) ? stationId + 1 : 0;

        if (tuner.PlayDab(stationId))
        {
            if (tuner.DabStationName(stationId, buffer, sizeof(buffer)))
            {
                Serial.print("Tuned program: (");
                Serial.print(stationId);
                Serial.print(") ");
                Serial.println(buffer);
            }
        }
    }

    if (tuner.EventReceived())
    {
        EventType eventType;
        if (tuner.EventRead(&eventType))
        {
            if (eventType == EventType::FinishedScan)
            {
                Serial.println("Program search finished.");
            }
            else if (eventType == EventType::DabStationText)
            {
                if (tuner.DabStationText(buffer, sizeof(buffer)))
                {
                    Serial.print("DAB text event: ");
                    Serial.println(buffer);
                }
            }
        }
    }
}
