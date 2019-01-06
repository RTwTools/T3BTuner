/*
 *  DABDUINO.cpp - Library for DABDUINO - DAB/DAB+ digital radio shield for Arduino.
 *  Created by Tomas Urbanek, Montyho technology Ltd., Januar 2, 2017.
 *  www.dabduino.com
 *  @license  BSD (see license.txt)
 */

#include "T3BTuner.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

#define TUNER_SERIAL_BAUDRATE 57600

#define COMMAND_SIZE_INDEX 5
#define COMMAND_START 0xFE
#define COMMAND_EMPTY 0x00
#define COMMAND_END 0xFD

#define CMD_SYSTEM 0x00
#define CMD_STREAM 0x01
#define CMD_RTC 0x02
#define CMD_EVENTS 0x07

#define SYSTEM_READY 0x00
#define SYSTEM_RESET 0x01
#define SYSTEM_AUDIO_OUTPUT 0x06

#define STREAM_PLAY 0x00
#define STREAM_PLAY_DAB 0x00
#define STREAM_PLAY_FM 0x01
#define STREAM_PLAY_BEEP 0x02
#define STREAM_STOP 0x01
#define STREAM_FM_SEARCH 0x02
#define STREAM_DAB_SEARCH 0x03
#define STREAM_STATUS 0x05
#define STREAM_MODE 0x06
#define STREAM_NOW_PLAYING 0x07
#define STREAM_SIGNAL_STRENGTH 0x08
#define STREAM_STEREO_MODE_SET 0x09
#define STREAM_STEREO_MODE_GET 0x0A
#define STREAM_STEREO_TYPE 0x0B
#define STREAM_VOLUME_SET 0x0C
#define STREAM_VOLUME_GET 0x0D
#define STREAM_STATION_TYPE 0x0E
#define STREAM_DAB_STATION_NAME 0x0F
#define STREAM_DAB_STATION_TEXT 0x10
#define STREAM_SAMPLE_RATE 0x11
#define STREAM_DAB_DATA_RATE 0x12
#define STREAM_DAB_SIGNAL_QUALITY 0x13
#define STREAM_DAB_FREQUENCY 0x14
#define STREAM_DAB_ENSEMBLE_NAME 0x15
#define STREAM_DAB_STATION_COUNT 0x16
#define STREAM_DAB_STATION_ON_AIR 0x17
#define STREAM_DAB_STATION_SERVICE_NAME 0x1A
#define STREAM_DAB_FOUND_STATIONS_COUNT 0x1B
#define STREAM_DAB_STATION_TYPE 0x1E
#define STREAM_MEMORY_SET 0x21
#define STREAM_MEMORY_GET 0x22
#define STREAM_DAB_STATION_INFO 0x23
#define STREAM_DAB_SORT_GET 0x24
#define STREAM_DAB_SORT_SET 0x25
#define STREAM_DAB_DRC_GET 0x26
#define STREAM_DAB_DRC_SET 0x27
#define STREAM_DAB_REMOVE_OFF_AIR 0x2B
#define STREAM_DAB_EXTENDED_COUNTRY_CODE 0x2D
#define STREAM_FM_RDS_PI_CODE 0x2E
#define STREAM_FM_STEREO_THRESHOLD_LEVEL_SET 0x30
#define STREAM_FM_STEREO_THRESHOLD_LEVEL_GET 0x31
#define STREAM_FM_RDS_DATA 0x32
#define STREAM_FM_SEEK_TRESHOLD_SET 0x35
#define STREAM_FM_SEEK_TRESHOLD_GET 0x36
#define STREAM_FM_STEREO_TRESHOLD_SET 0x37
#define STREAM_FM_STEREO_TRESHOLD_GET 0x38
#define STREAM_FM_EXACT_STATION 0x39

#define RTC_SET 0x00
#define RTC_GET 0x01
#define RTC_SYNC 0x02
#define RTC_SYNC_STATUS 0x03
#define RTC_STATUS_CLOCK 0x04

#define EVENTS_NOTIFY 0x00


T3BTuner::T3BTuner(Stream* serial, SerialType serialType, uint8_t resetPin, uint8_t mutePin, uint8_t spiCsPin) :
  serial(serial),
  serialType(serialType),
  pinReset(resetPin),
  pinMute(mutePin),
  pinSpiCs(spiCsPin)
{
}



void T3BTuner::Init()
{
  if (pinMute != UNUSED_PIN)
  {
    pinMode(pinMute, OUTPUT);
    digitalWrite(pinMute, HIGH);
  }

  if (pinSpiCs != UNUSED_PIN)
  {
    pinMode(pinSpiCs, OUTPUT);
    digitalWrite(pinSpiCs, LOW);
  }

  SerialBegin(TUNER_SERIAL_BAUDRATE);
  serial->setTimeout(50);

  pinMode(pinReset, OUTPUT);
  digitalWrite(pinReset, LOW);
  delay(100);
  digitalWrite(pinReset, HIGH);
  delay(1000);

  while (!Ready())
  {
    delay(100);
  }
}






// *************************
// ***** SYSTEM ************
// *************************

/*
 *   Test for DAB module is ready for communication
 */
bool T3BTuner::Ready()
{
  CommandCreate(CMD_SYSTEM, SYSTEM_READY);
  return CommandSend();
}

/*
 *   Reset module.
 *   FullReset => Reset module database & module.
 */
bool T3BTuner::Reset(bool fullReset)
{
  CommandCreate(CMD_SYSTEM, SYSTEM_RESET, (uint8_t)fullReset);
  if (CommandSend())
  {
    Init();
    return true;
  }
  return false;
}

/*
 *   Set audio output channels (SPDIV, CINCH /I2S DAC/)
 *   CINCH for analog output, SPDIV for optical digital output
 */
bool T3BTuner::AudioOutput(bool spdiv, bool cinch)
{
  uint8_t param = (uint8_t)spdiv | ((uint8_t)cinch << 0x1);
  CommandCreate(CMD_SYSTEM, SYSTEM_AUDIO_OUTPUT, param);
  return CommandSend();
}

// *************************
// ***** STREAM ************
// *************************

/*
 *   Play DAB program
 *   programIndex = 1..9999999 (see programs index)
 */
bool T3BTuner::PlayDab(uint32_t stationId)
{
  CommandCreatePlay(STREAM_PLAY_DAB, stationId);
  return CommandSend();
}

/*
 *   Play FM program
 *   frequency = 87500..108000 (MHz)
 */
bool T3BTuner::PlayFm(uint32_t frequency)
{
  CommandCreatePlay(STREAM_PLAY_FM, frequency);
  return CommandSend();
}

/* 
 *   Play Beep.
 */
bool T3BTuner::PlayBeep()
{
  CommandCreatePlay(STREAM_PLAY_BEEP, 0x00);
  return CommandSend();
}

/*
 *   Stop.
 */
bool T3BTuner::Stop()
{
  CommandCreate(CMD_STREAM, STREAM_STOP);
  return CommandSend();
}

/*
 * Seek FM program.
 */
bool T3BTuner::FmSearch(bool searchForward)
{
  CommandCreate(CMD_STREAM, STREAM_FM_SEARCH, (uint8_t)searchForward);
  return CommandSend();
}

/*
 * Search DAB bands for programs.
 */
bool T3BTuner::DabSearch(DabBand band)
{
  CommandStart(CMD_STREAM, STREAM_DAB_SEARCH);
  switch (band)
  {
  case DabBand::BandIII:
    CommandAppend((uint8_t)0);
    CommandAppend((uint8_t)40);
    break;
  case DabBand::ChinaBand:
    CommandAppend((uint8_t)41);
    CommandAppend((uint8_t)71);
    break;
  case DabBand::LBand:
    CommandAppend((uint8_t)72);
    CommandAppend((uint8_t)94);
    break;
  }
  CommandEnd();
  return CommandSend();
}

/*
 *   Radio module play status.
 */
bool T3BTuner::State(TunerState *status)
{
  CommandCreate(CMD_STREAM, STREAM_STATUS);
  return (CommandSend() && ResponseUint8(0, (uint8_t *)status));
}

/*
 *   Radio module play mode.
 */
bool T3BTuner::Mode(TunerMode *mode)
{
  CommandCreate(CMD_STREAM, STREAM_MODE);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)mode));
}

/*
 * Get DAB stationId, get FM frequency.
 */
bool T3BTuner::NowPlaying(uint32_t* programId)
{
  CommandCreate(CMD_STREAM, STREAM_NOW_PLAYING);
  return (CommandSend() && ResponseUint32(0, programId));
}

/*
 * Get signal strength
 * DAB: signalStrength=0..18, bitErrorRate=
 * FM: signalStrength=0..100
 */
bool T3BTuner::SignalStrength(uint8_t* signalStrength, uint16_t* bitErrorRate)
{
  CommandCreate(CMD_STREAM, STREAM_SIGNAL_STRENGTH);
  return (CommandSend() && ResponseUint8(0, signalStrength)
    && ResponseUint16(1, bitErrorRate));
}

/*
 *   Set stereo mode.
 */
bool T3BTuner::StereoModeSet(StereoMode stereoMode)
{
  CommandCreate(CMD_STREAM, STREAM_STEREO_MODE_SET, (uint8_t)stereoMode);
  return CommandSend();
}

/*
 *   Get stereo mode.
 */
bool T3BTuner::StereoModeGet(StereoMode* stereoMode)
{
  CommandCreate(CMD_STREAM, STREAM_STEREO_MODE_GET);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)stereoMode));
}

/*
 *   Get stereo type
 */
bool T3BTuner::StereoTypeGet(StereoType* stereotype)
{
  CommandCreate(CMD_STREAM, STREAM_STEREO_TYPE);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)stereotype));
}

/*
 *   Set volume.
 *   volumeLevel = 0..16
 */
bool T3BTuner::VolumeSet(uint8_t volume)
{
  uint8_t volumeValue = (volume > 16) ? 16 : volume;
  CommandCreate(CMD_STREAM, STREAM_VOLUME_SET, volumeValue);
  return CommandSend();
}

/*
 *   Get volume.
 *   return set volumeLevel: 0..16
 */
bool T3BTuner::VolumeGet(uint8_t *volume)
{
  CommandCreate(CMD_STREAM, STREAM_VOLUME_GET);
  return (CommandSend() && ResponseUint8(0, volume));
}

/*
 *   Get program type.
 */
bool T3BTuner::StationTypeGet(StationType* programType)
{
  CommandCreate(CMD_STREAM, STREAM_STATION_TYPE);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)programType));
}

/*
 * Get DAB station name.
 */
bool T3BTuner::DabStationName(uint32_t stationId, char* buffer, uint16_t size, bool longName)
{
  CommandCreateName(STREAM_DAB_STATION_NAME, stationId, longName);
  return (CommandSend() && ResponseText(buffer, size));
}

/*
 * Get DAB text event
 * return: 1=new text, 2=text is same, 3=no text
 * dabText: text
 */
bool T3BTuner::DabStationText(char* buffer, uint16_t size)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_STATION_TEXT);
  if (CommandSend())
  {
    if (responseSize == 1)
    {
      // No text received
      // Response[0] value => 0 = No text available, 1 = Station text is empty.
      return false;
    }

    ResponseText(buffer, size);
    bool changed = (strncmp(buffer, dabStationText, sizeof(dabStationText)) == 0);
    strncpy(dabStationText, buffer, sizeof(dabStationText));
    return changed;
  }

  return false;
}

/*
 *   Get sampling rate (DAB/FM).
 */
bool T3BTuner::SampleRateGet(SampleRate* sampleRate)
{
  CommandCreate(CMD_STREAM, STREAM_SAMPLE_RATE);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)sampleRate));
}

/*
 *   Get data rate (DAB)
 *   return data: data rate in kbps
 */
bool T3BTuner::DabDataRate(uint16_t* dataRate)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_DATA_RATE);
  return (CommandSend() && ResponseUint16(0, dataRate));
}

/*
 *   Get DAB signal quality
 *   return: 0..100
 *   0..19 = playback stop
 *   20..30 = the noise (short break) appears
 *   100 = the bit error rate is 0
 */
bool T3BTuner::DabSignalQuality(uint8_t* data)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_SIGNAL_QUALITY);
  return (CommandSend() && ResponseUint8(0, data));
}

/*
 *   Get DAB frequency for program index
 *   return: frequency index
 *   0=174.928MHz, 1=176.64, 2=178.352,...
 *
 *  // TODO: add conversion table for index2freqency
 */
bool T3BTuner::DabStationFrequency(uint32_t stationId, uint8_t* frequency)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_FREQUENCY, stationId);
  return (CommandSend() && ResponseUint8(0, frequency));
}

/*
 * Get DAB program ensemble name.
 */
bool T3BTuner::DabStationEnsembleName(uint32_t stationId, char* buffer, uint16_t size, bool longName)
{
  CommandCreateName(STREAM_DAB_ENSEMBLE_NAME, stationId, longName);
  return (CommandSend() && ResponseText(buffer, size));
}

/*
 * Number of DAB stations in database.
 */
bool T3BTuner::DabStationCount(uint32_t* count)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_STATION_COUNT);
  return (CommandSend() && ResponseUint32(0, count));
}

/*
 *   Test DAB program is active (on-air)
 *   return: 0=off-air, 1=on-air
 */
bool T3BTuner::DabStationOnAir(uint32_t stationId, bool* onAir)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_STATION_ON_AIR, stationId);
  bool result = CommandSend();
  *onAir = (bool)response[0];
  return result;
}

/*
 * Get DAB program service short name.
 */
bool T3BTuner::DabStationServiceName(uint32_t stationId, char* buffer, uint16_t size, bool longName)
{
  CommandCreateName(STREAM_DAB_STATION_SERVICE_NAME, stationId, longName);
  return (CommandSend() && ResponseText(buffer, size));
}

/*
 * Number of programs found in search process.
 */
bool T3BTuner::DabFoundStationsCount(uint8_t* count)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_FOUND_STATIONS_COUNT);
  return (CommandSend() && ResponseUint8(0, count));
}

/*
 * Get DAB program service component type (ASCTy)
 */
bool T3BTuner::DabStationType(uint32_t stationId, DabStreamType* type)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_STATION_TYPE, stationId);
  return (CommandSend() && ResponseUint8(0, (uint8_t *)type));
}

/*
 *   Set preset
 */
bool T3BTuner::MemorySet(MemoryType mode, MemoryId id, uint32_t programId)
{
  CommandStart(CMD_STREAM, STREAM_MEMORY_SET);
  CommandAppend((uint8_t)mode);
  CommandAppend((uint8_t)id);
  CommandAppend(programId);
  CommandEnd();
  return CommandSend();
}

/*
 *  Get preset
 */
bool T3BTuner::MemoryGet(MemoryType mode, MemoryId id, uint32_t* programId)
{
  CommandStart(CMD_STREAM, STREAM_MEMORY_GET);
  CommandAppend((uint8_t)mode);
  CommandAppend((uint8_t)id);
  CommandEnd();
  return (CommandSend() && ResponseUint32(0, programId));
}

/*
 * Get station info
 * return serviceId = service id of DAB program
 * return ensembleId = ensemble id of DAB program
 */
bool T3BTuner::DabStationInfo(uint32_t stationId, uint32_t* serviceId, uint16_t* ensembleId)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_STATION_INFO, stationId);
  return (CommandSend() && ResponseUint32(0, serviceId)
    && ResponseUint16(4, ensembleId));
}

/*
 *   Get DAB station sort order.
 */
bool T3BTuner::DabSortGet(DabSortOrder *sortOrder)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_SORT_GET);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)sortOrder));
}

/*
 *   Set DAB station sort order.
 */
bool T3BTuner::DabSortSet(DabSortOrder sortOrder)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_SORT_SET, (uint8_t)sortOrder);
  return CommandSend();
}

/*
 *   Get DAB DRC.
 */
bool T3BTuner::DabDrcGet(DabDrc* drc)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_DRC_GET);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)drc));
}

/*
 *   Set DAB DRC.
 */
bool T3BTuner::DabDrcSet(DabDrc drc)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_DRC_SET, (uint8_t)drc);
  return CommandSend();
}

/*
 *   Prune programs - delete inactive programs (!on-air)
 */
bool T3BTuner::DabRemoveOffAir(uint16_t* removedTotal, uint16_t* removedIndex)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_REMOVE_OFF_AIR);
  return (CommandSend() && ResponseUint16(0, removedTotal)
    && ResponseUint16(2, removedIndex));
}

/*
 * Get ECC
 * return ECC (Extended Country Code)
 * return countryId (Country identification)
 */
bool T3BTuner::DabExtendedCountryCode(uint8_t* ecc, uint8_t* countryId)
{
  CommandCreate(CMD_STREAM, STREAM_DAB_EXTENDED_COUNTRY_CODE);
  return (CommandSend() && ResponseUint8(0, ecc)
    && ResponseUint8(1, countryId));
}

/*
 *   Get FM RDS PI code
 */
bool T3BTuner::FmRdsPiCode(uint16_t *code)
{
  CommandCreate(CMD_STREAM, STREAM_FM_RDS_PI_CODE);
  return (CommandSend() && ResponseUint16(0, code));
}

/*
 *   Set FMstereoThdLevel
 *   RSSItresholdLevel = 0..10
 */
bool T3BTuner::FmStereoTresholdLevelSet(uint8_t level)
{
  CommandCreate(CMD_STREAM, STREAM_FM_STEREO_THRESHOLD_LEVEL_SET, level);
  return CommandSend();
}

/*
 *   Get FMstereoThdLevel
 *   data return = 0..10
 */
bool T3BTuner::FmStereoTresholdLevelGet(uint8_t* level)
{
  CommandCreate(CMD_STREAM, STREAM_FM_STEREO_THRESHOLD_LEVEL_GET);
  return (CommandSend() && ResponseUint8(0, level));
}

/*
 *   Get RDS raw data
 *   return: 1=new RDS data, 2=no new RDS data, 3=no RDS data
 */
bool T3BTuner::FmRdsRawData(uint16_t* blockA, uint16_t* blockB, uint16_t* blockC, uint16_t* blockD, 
                            uint16_t* blerA, uint16_t* blerB, uint16_t* blerC, uint16_t* blerD)
{
  CommandCreate(CMD_STREAM, STREAM_FM_RDS_DATA);
  if (CommandSend() && responseSize > 1)
  {
    return (ResponseUint16(0, blockA) && ResponseUint16(2, blockB) &&
      ResponseUint16(4, blockC) && ResponseUint16(6, blockD) &&
      ResponseUint16(8, blerA) && ResponseUint16(10, blerB) &&
      ResponseUint16(12, blerC) && ResponseUint16(14, blerD));
  }
  return false;
}

/*
 *   Set FMseekTreshold
 *   RSSItreshold = 0..100
 */
bool T3BTuner::FmSeekTresholdSet(uint8_t treshold)
{
  CommandCreate(CMD_STREAM, STREAM_FM_SEEK_TRESHOLD_SET, treshold);
  return CommandSend();
}

/*
 *   Get FMseekTreshold
 *   data return = 0..100
 */
bool T3BTuner::FmSeekTresholdGet(uint8_t* treshold)
{
  CommandCreate(CMD_STREAM, STREAM_FM_SEEK_TRESHOLD_GET);
  return (CommandSend() && ResponseUint8(0, treshold));
}

/*
 *   Set FMstereoTreshold
 *   RSSItreshold = 0..100
 */
bool T3BTuner::FmStereoTresholdSet(uint8_t treshold)
{
  CommandCreate(CMD_STREAM, STREAM_FM_STEREO_TRESHOLD_SET, treshold);
  return CommandSend();
}

/*
 *   Get FMstereoTreshold
 *   data return = 0..100
 */
bool T3BTuner::FmStereoTresholdGet(uint8_t* treshold)
{
  CommandCreate(CMD_STREAM, STREAM_FM_STEREO_TRESHOLD_GET);
  return (CommandSend() && ResponseUint8(0, treshold));
}

/*
 *   Get FM Exact station.
 */
bool T3BTuner::FmExactStationGet(FmExactStation* exact)
{
  CommandCreate(CMD_STREAM, STREAM_FM_EXACT_STATION);
  return (CommandSend() && ResponseUint8(0, (uint8_t*)exact));
}

// *************************
// ***** RTC ***************
// *************************

/*
 *  Set RTC clock
 *  year: 2017=17,2018=18, month: 1..12, day: 1..31, hour: 0..23, minute: 0..59, second: 0..59 
 */
bool T3BTuner::ClockSet(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
  CommandStart(CMD_RTC, RTC_SET);
  CommandAppend(second);
  CommandAppend(minute);
  CommandAppend(hour);
  CommandAppend(day);
  CommandAppend((uint8_t)0x00);
  CommandAppend(month);
  CommandAppend(year);
  CommandEnd();
  return CommandSend();
}

/*
 *  Get RTC ckock
 *  year: 2017=17,2018=18, month: 1..12, day: 1..31, hour: 0..23, minute: 0..59, second: 0..59 
 */
bool T3BTuner::ClockGet(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second)
{
  CommandCreate(CMD_RTC, RTC_GET);
  bool result = CommandSend();
  result &= ResponseUint8(0, second) & ResponseUint8(1, minute);
  result &= ResponseUint8(2, hour) & ResponseUint8(3, day);
  result &= ResponseUint8(5, month) & ResponseUint8(6, year);
  return result;
}

/*
 *  Set RTC sync clock from stream enable
 */
bool T3BTuner::ClockSyncSet(bool enable)
{
  CommandCreate(CMD_RTC, RTC_SYNC, (uint8_t)enable);
  return CommandSend();
}

/*
 *  Get RTC sync clock status
 */
bool T3BTuner::ClockSyncGet(bool *enabled)
{
  CommandCreate(CMD_RTC, RTC_SYNC_STATUS);
  bool result = CommandSend();
  *enabled = response[0];
  return result;
}

/*
 *  Get RTC clock status
 */
bool T3BTuner::ClockStatusGet(ClockStatus *status)
{
  CommandCreate(CMD_RTC, RTC_STATUS_CLOCK);
  return (CommandSend() && ResponseUint8(0, (uint8_t *)status));
}

// *************************
// ***** EVENTS ************
// *************************

/*
 *   Enabled / Disable event notifications.
 */
bool T3BTuner::EventEnable(bool enable)
{
  uint8_t value = (enable) ? 0x7F : 0x00;
  CommandCreate(CMD_EVENTS, EVENTS_NOTIFY, value);
  return CommandSend();
}

bool T3BTuner::EventReceived()
{
  return (bool)serial->available();
}

/*
 *   Read event
 *   RETURN EVENT TYP: 1=scan finish, 2=got new DAB program text, 3=DAB reconfiguration, 4=DAB channel list order change, 5=RDS group, 6=Got new FM radio text, 7=Return the scanning frequency /FM/
 */
int8_t T3BTuner::EventRead()
{
  uint8_t eventData[16];
  uint8_t dabReturn[6];
  uint8_t isPacketCompleted = 0;
  uint16_t byteIndex = 0;
  uint16_t dataIndex = 0;
  uint8_t serialData = 0;
  uint8_t eventDataSize = 128;
  unsigned long endMillis = millis() + 200; // timeout for answer from module = 200ms
  while (millis() < endMillis && dataIndex < DAB_MAX_DATA_LENGTH) {
    if (serial->available() > 0) {
      serialData = serial->read();
      if (serialData == 0xFE) {
        byteIndex = 0;
        dataIndex = 0;
      }
      if (eventDataSize && dataIndex < eventDataSize) {
        eventData[dataIndex++] = serialData;
      }
      if (byteIndex <= 5) {
        dabReturn[byteIndex] = serialData;
      }
      if (byteIndex == 5) {
        eventDataSize = (((long)dabReturn[4] << 8) + (long)dabReturn[5]);
      }
      if ((int16_t)(byteIndex - eventDataSize) >= 5 && serialData == 0xFD) {
        isPacketCompleted = 1;
        break;
      }
      byteIndex++;
    }
  }
  while (serial->available() > 0) {
    serial->read();
  }
  if (isPacketCompleted == 1 && dabReturn[1] == 0x07) {
    return dabReturn[2] + 1;
  }
  else {
    return 0;
  }
}

// *************************
// ***** PRIVATE FUNCTIONS *
// *************************

void T3BTuner::SerialBegin(uint32_t baud)
{
  switch (serialType)
  {
  case SerialType::Hardware:
    static_cast<HardwareSerial*>(serial)->begin(baud);
    break;
  case SerialType::Software:
    static_cast<SoftwareSerial*>(serial)->begin(baud);
    break;
  }
}

void T3BTuner::CommandStart(uint8_t type, uint8_t subType)
{
  command[0] = COMMAND_START;
  command[1] = type;
  command[2] = subType;
  command[3] = COMMAND_EMPTY;
  command[4] = COMMAND_EMPTY;
  command[5] = COMMAND_EMPTY;
  commandSize = 6;
}

void T3BTuner::CommandAppend(uint8_t data)
{
  // TODO Check if index is too big for data!

  command[commandSize] = data;
  command[COMMAND_SIZE_INDEX] = command[COMMAND_SIZE_INDEX] + 1;
  commandSize++;
}

void T3BTuner::CommandAppend(uint32_t data)
{
  CommandAppend((uint8_t)((data >> 24) & 0xFF));
  CommandAppend((uint8_t)((data >> 16) & 0xFF));
  CommandAppend((uint8_t)((data >> 8) & 0xFF));
  CommandAppend((uint8_t)((data >> 0) & 0xFF));
}

void T3BTuner::CommandEnd()
{
  command[commandSize++] = COMMAND_END;
}

void T3BTuner::CommandCreate(uint8_t type, uint8_t command)
{
  CommandStart(type, command);
  CommandEnd();
}

void T3BTuner::CommandCreate(uint8_t type, uint8_t subType, uint8_t param)
{
  CommandStart(type, subType);
  CommandAppend(param);
  CommandEnd();
}

void T3BTuner::CommandCreate(uint8_t type, uint8_t subType, uint32_t param)
{
  CommandStart(type, subType);
  CommandAppend(param);
  CommandEnd();
}

void T3BTuner::CommandCreatePlay(uint8_t playType, uint32_t param)
{
  CommandStart(CMD_STREAM, STREAM_PLAY);
  CommandAppend(playType);
  CommandAppend(param);
  CommandEnd();
}

void T3BTuner::CommandCreateName(uint8_t subType, uint32_t program, bool longName)
{
  CommandStart(CMD_STREAM, subType);
  CommandAppend(program);
  CommandAppend((uint8_t)longName);
  CommandEnd();
}

/*
 *  Send command to DAB module and wait for answer
 */
bool T3BTuner::CommandSend()
{
  uint8_t dabReturn[6];
  uint8_t isPacketCompleted = 0;
  uint16_t byteIndex = 0;
  uint16_t dataIndex = 0;
  uint8_t serialData = 0;
  responseSize = 0;
  while (serial->available() > 0) {
    serial->read();
  }
  while (byteIndex < 255) {
    if (command[byteIndex++] == 0xFD) break;
  }
  serial->write(command, byteIndex);
  serial->flush();
  byteIndex = 0;
  unsigned long endMillis = millis() + 200; // timeout for answer from module = 200ms
  while (millis() < endMillis && dataIndex < DAB_MAX_DATA_LENGTH) {
    if (serial->available() > 0) {
      serialData = serial->read();
      if (serialData == 0xFE) {
        byteIndex = 0;
        dataIndex = 0;
      }
      if (responseSize && dataIndex < responseSize) {
        response[dataIndex++] = serialData;
      }
      if (byteIndex <= 5) {
        dabReturn[byteIndex] = serialData;
      }
      if (byteIndex == 5) {
        responseSize = (((long)dabReturn[4] << 8) + (long)dabReturn[5]);
      }
      if ((int16_t)(byteIndex - responseSize) >= 5 && serialData == 0xFD) {
        isPacketCompleted = 1;
        break;
      }
      byteIndex++;
    }
  }
  return (isPacketCompleted == 1 && !(dabReturn[1] == 0x00 && dabReturn[2] == 0x02));
}

bool T3BTuner::ResponseText(char* buffer, uint16_t size)
{
  uint16_t j = 0;
  for (uint16_t i = 0; i < responseSize; i = i + 2)
  {
    if (size <= j) return false;
    buffer[j++] = Uint16ToChar(response[i], response[i + 1]);
  }
  return true;
}

bool T3BTuner::ResponseUint8(uint8_t index, uint8_t* resp)
{
  if (responseSize > index)
  {
    *resp = response[index];
    return true;
  }
  return false;
}

bool T3BTuner::ResponseUint16(uint8_t index, uint16_t* resp)
{
  if (responseSize > (index + 1))
  {
    *resp = (uint16_t)response[index + 1];
    *resp |= (uint16_t)response[index] << 8;
    return true;
  }
  return false;
}

bool T3BTuner::ResponseUint32(uint8_t index, uint32_t* resp)
{
  if (responseSize > (index + 3))
  {
    *resp = (uint32_t)response[index + 3];
    *resp |= (uint32_t)response[index + 2] << 8;
    *resp |= (uint32_t)response[index + 1] << 16;
    *resp |= (uint32_t)response[index + 0] << 24;
    return true;
  }
  return false;
}

/*
 * Convert uint16_t (2 * uint8_t) from Tuner to a char.
 */
char T3BTuner::Uint16ToChar(uint8_t byte1, uint8_t byte0)
{
  if (byte1 == 0x00)
  {
    if (byte0 < 128)
    {
      return byte0;
    }

    switch (byte0)
    {
    case 0x8A: return 'S';
    case 0x8C: return 'S';
    case 0x8D: return 'T';
    case 0x8E: return 'Z';
    case 0x8F: return 'Z';
    case 0x9A: return 's';
    case 0x9D: return 't';
    case 0x9E: return 'z';
    case 0xC0: return 'A';
    case 0xC1: return 'A';
    case 0xC2: return 'A';
    case 0xC3: return 'A';
    case 0xC4: return 'A';
    case 0xC5: return 'A';
    case 0xC7: return 'C';
    case 0xC8: return 'E';
    case 0xC9: return 'E';
    case 0xCA: return 'E';
    case 0xCB: return 'E';
    case 0xCC: return 'I';
    case 0xCD: return 'I';
    case 0xCE: return 'I';
    case 0xCF: return 'I';
    case 0xD0: return 'D';
    case 0xD1: return 'N';
    case 0xD2: return 'O';
    case 0xD3: return 'O';
    case 0xD4: return 'O';
    case 0xD5: return 'O';
    case 0xD6: return 'O';
    case 0xD8: return 'O';
    case 0xD9: return 'U';
    case 0xDA: return 'U';
    case 0xDB: return 'U';
    case 0xDC: return 'U';
    case 0xDD: return 'Y';
    case 0xE0: return 'a';
    case 0xE1: return 'a';
    case 0xE2: return 'a';
    case 0xE3: return 'a';
    case 0xE4: return 'a';
    case 0xE5: return 'a';
    case 0xE7: return 'c';
    case 0xE8: return 'e';
    case 0xE9: return 'e';
    case 0xEA: return 'e';
    case 0xEB: return 'e';
    case 0xEC: return 'i';
    case 0xED: return 'i';
    case 0xEE: return 'i';
    case 0xEF: return 'i';
    case 0xF1: return 'n';
    case 0xF2: return 'o';
    case 0xF3: return 'o';
    case 0xF4: return 'o';
    case 0xF5: return 'o';
    case 0xF6: return 'o';
    case 0xF9: return 'u';
    case 0xFA: return 'u';
    case 0xFB: return 'u';
    case 0xFC: return 'u';
    case 0xFD: return 'y';
    case 0xFF: return 'y';
    }
  }
  else if (byte1 == 0x01)
  {
    switch (byte0)
    {
    case 0x1B: return 'e'; // ě
    case 0x48: return 'n'; // ň
    case 0x59: return 'r'; // ř
    case 0x0D: return 'c'; // č
    case 0x7E: return 'z'; // ž
    case 0x0C: return 'C'; // Č
    }
  }

  return 0x00;
}