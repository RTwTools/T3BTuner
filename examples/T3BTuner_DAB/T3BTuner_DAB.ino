/*
  DABDUINO basic example 1
  DABDUINO is DAB+ digital radio shield for Arduino
  Created by Tomas Urbanek, Montyho technology Ltd., Januar 2, 2017.
  www.dabduino.com
*/

#include "SerialStream.h"
#include "T3BTuner.h"
#include <SoftwareSerial.h>

static uint8_t const PinMute = 2U;
static uint8_t const PinReset = 3U;
static uint8_t const PinTunerTx = 8U;
static uint8_t const PinTunerRx = 9U;

auto tunerSerial = new SoftwareSerial(PinTunerRx, PinTunerTx);
auto tunerStream = new SerialStream(tunerSerial);
T3BTuner tuner(*tunerStream, PinReset, PinMute);
char buffer[T3BTunerMaxTextSize];
uint32_t stationCount = 0;
uint32_t stationId = 0;

void setup()
{
    Serial.begin(57600);
    Serial.println("DAB RESET & START");
    tuner.init();

    if (!tuner.reset())
    {
        Serial.println("DAB NOT READY");
        while (1)
        {
        }
    }

    Serial.println("DAB READY");
    Serial.print("Search for DAB programs:");
    tuner.dabSearch();

    TunerState status;
    TunerState lastStatus;

    while (true)
    {
        tuner.state(&status);
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

    tuner.dabStationCount(&stationCount);
    Serial.println("Available programs: ");
    for (uint32_t i = 0; i <= stationCount; i++)
    {
        if (tuner.dabStationName(i, buffer, sizeof(buffer)))
        {
            Serial.print(i);
            Serial.print("\t ");
            Serial.println(buffer);
        }
    }
    Serial.println();

    if (tuner.audioOutput())
    {
        Serial.println("Set audio output");
    }

    if (tuner.volumeSet(8))
    {
        Serial.println("Set volume");
    }

    stationId = 0;

    if (tuner.eventEnable(true))
    {
        Serial.println("Event notification enabled");
    }
}

void loop()
{
    if (millis() % 20000 == 0)
    {
        stationId = (stationId < stationCount) ? stationId + 1 : 0;

        if (tuner.playDab(stationId))
        {
            if (tuner.dabStationName(stationId, buffer, sizeof(buffer)))
            {
                Serial.print("Tuned program: (");
                Serial.print(stationId);
                Serial.print(") ");
                Serial.println(buffer);
            }
        }
    }

    if (tuner.eventReceived())
    {
        EventType eventType;
        if (tuner.eventRead(&eventType))
        {
            if (eventType == EventType::FinishedScan)
            {
                Serial.println("Program search finished.");
            }
            else if (eventType == EventType::DabStationText)
            {
                if (tuner.dabStationText(buffer, sizeof(buffer)))
                {
                    Serial.print("DAB text event: ");
                    Serial.println(buffer);
                }
            }
        }
    }
}
