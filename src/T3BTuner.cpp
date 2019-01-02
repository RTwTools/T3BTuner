/*
 *  DABDUINO.cpp - Library for DABDUINO - DAB/DAB+ digital radio shield for Arduino.
 *  Created by Tomas Urbanek, Montyho technology Ltd., Januar 2, 2017.
 *  www.dabduino.com
 *  @license  BSD (see license.txt)
 */

#include "T3BTuner.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

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


T3BTuner::T3BTuner(Stream* stream, StreamType streamType, uint8_t resetPin, uint8_t dacMutePin, uint8_t spiCsPin) :
  stream(stream),
  streamType(streamType),
  resetPin(resetPin),
  dacMutePin(dacMutePin),
  spiCsPin(spiCsPin)
{
}

void T3BTuner::commandAppend(uint8_t data)
{
  // TODO Check if index is too big for data!

  command[commandSize] = data;
  command[COMMAND_SIZE_INDEX] = command[COMMAND_SIZE_INDEX] + 1;
  commandSize++;
}

void T3BTuner::commandAppend(uint32_t data)
{
  commandAppend((uint8_t)((data >> 24) & 0xFF));
  commandAppend((uint8_t)((data >> 16) & 0xFF));
  commandAppend((uint8_t)((data >> 8) & 0xFF));
  commandAppend((uint8_t)((data >> 0) & 0xFF));
}

void T3BTuner::commandStart(uint8_t type, uint8_t subType)
{
  command[0] = COMMAND_START;
  command[1] = type;
  command[2] = subType;
  command[3] = COMMAND_EMPTY;
  command[4] = COMMAND_EMPTY;
  command[5] = COMMAND_EMPTY;
  commandSize = 6;
}

void T3BTuner::commandEnd()
{
  command[commandSize++] = COMMAND_END;
}

void T3BTuner::commandCreate(uint8_t type, uint8_t command)
{
  commandStart(type, command);
  commandEnd();
}

void T3BTuner::commandCreate(uint8_t type, uint8_t subType, uint8_t param)
{
  commandStart(type, subType);
  commandAppend(param);
  commandEnd();
}

void T3BTuner::commandCreate(uint8_t type, uint8_t subType, uint32_t param)
{
  commandStart(type, subType);
  commandAppend(param);
  commandEnd();
}

void T3BTuner::commandCreatePlay(uint8_t playType, uint32_t param)
{
  commandStart(CMD_STREAM, STREAM_PLAY);
  commandAppend(playType);
  commandAppend(param);
  commandEnd();
}

void T3BTuner::commandCreateName(uint8_t subType, uint32_t program, bool longName)
{
  commandStart(CMD_STREAM, subType);
  commandAppend(program);
  commandAppend((uint8_t)longName);
  commandEnd();
}

/*
 * Convert two byte char from DAB to one byte char. Add next chars...
 */
char T3BTuner::charToAscii(uint8_t byte1, uint8_t byte0) {

  if (byte1 == 0x00) {

    if (byte0 == 0x0) {
      return (byte0);
    }

    if (byte0 < 128) {
      return (byte0);
    }

    switch (byte0)
    {
    case 0x8A: return (0x53); break;
    case 0x8C: return (0x53); break;
    case 0x8D: return (0x54); break;
    case 0x8E: return (0x5a); break;
    case 0x8F: return (0x5a); break;
    case 0x9A: return (0x73); break;
    case 0x9D: return (0x74); break;
    case 0x9E: return (0x7a); break;
    case 0xC0: return (0x41); break;
    case 0xC1: return (0x41); break;
    case 0xC2: return (0x41); break;
    case 0xC3: return (0x41); break;
    case 0xC4: return (0x41); break;
    case 0xC5: return (0x41); break;
    case 0xC7: return (0x43); break;
    case 0xC8: return (0x45); break;
    case 0xC9: return (0x45); break;
    case 0xCA: return (0x45); break;
    case 0xCB: return (0x45); break;
    case 0xCC: return (0x49); break;
    case 0xCD: return (0x49); break;
    case 0xCE: return (0x49); break;
    case 0xCF: return (0x49); break;
    case 0xD0: return (0x44); break;
    case 0xD1: return (0x4e); break;
    case 0xD2: return (0x4f); break;
    case 0xD3: return (0x4f); break;
    case 0xD4: return (0x4f); break;
    case 0xD5: return (0x4f); break;
    case 0xD6: return (0x4f); break;
    case 0xD8: return (0x4f); break;
    case 0xD9: return (0x55); break;
    case 0xDA: return (0x55); break;
    case 0xDB: return (0x55); break;
    case 0xDC: return (0x55); break;
    case 0xDD: return (0x59); break;
    case 0xE0: return (0x61); break;
    case 0xE1: return (0x61); break;
    case 0xE2: return (0x61); break;
    case 0xE3: return (0x61); break;
    case 0xE4: return (0x61); break;
    case 0xE5: return (0x61); break;
    case 0xE7: return (0x63); break;
    case 0xE8: return (0x65); break;
    case 0xE9: return (0x65); break;
    case 0xEA: return (0x65); break;
    case 0xEB: return (0x65); break;
    case 0xEC: return (0x69); break;
    case 0xED: return (0x69); break;
    case 0xEE: return (0x69); break;
    case 0xEF: return (0x69); break;
    case 0xF1: return (0x6e); break;
    case 0xF2: return (0x6f); break;
    case 0xF3: return (0x6f); break;
    case 0xF4: return (0x6f); break;
    case 0xF5: return (0x6f); break;
    case 0xF6: return (0x6f); break;
    case 0xF9: return (0x75); break;
    case 0xFA: return (0x75); break;
    case 0xFB: return (0x75); break;
    case 0xFC: return (0x75); break;
    case 0xFD: return (0x79); break;
    case 0xFF: return (0x79); break;
    }
  }

  if (byte1 == 0x01) {
    switch (byte0)
    {
    case 0x1B: return (0x65); break; // ě > e
    case 0x48: return (0x6e); break; // ň > n
    case 0x59: return (0x72); break; // ř > r
    case 0x0D: return (0x63); break; // č > c
    case 0x7E: return (0x7A); break; // ž > z
    case 0x0C: return (0x43); break; // Č > C
    }
  }

  return  (0x20);
}

void T3BTuner::init() {

  // DAC MUTE
  if (dacMutePin != UNUSED_PIN)
  {
    pinMode(dacMutePin, OUTPUT);
    digitalWrite(dacMutePin, HIGH);
  }

  // SPI CS
  if (spiCsPin != UNUSED_PIN)
  {
    pinMode(spiCsPin, OUTPUT);
    digitalWrite(spiCsPin, LOW);
  }

  // DAB module SERIAL
  streamBegin(57600);
  stream->setTimeout(50);

  // DAB module RESET
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  delay(1000);

  while (!Ready()) {
    delay(100);
  }
}

void T3BTuner::streamBegin(uint32_t baud)
{
  switch (streamType)
  {
  case StreamType::HardwareSerial:
    static_cast<HardwareSerial*>(stream)->begin(baud);
    break;
  case StreamType::SoftwareSerial:
    static_cast<SoftwareSerial*>(stream)->begin(baud);
    break;
  }
}

int8_t T3BTuner::isEvent() {
  return stream->available();
}

/*
 *   Read event
 *   RETURN EVENT TYP: 1=scan finish, 2=got new DAB program text, 3=DAB reconfiguration, 4=DAB channel list order change, 5=RDS group, 6=Got new FM radio text, 7=Return the scanning frequency /FM/
 */
int8_t T3BTuner::readEvent()
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
    if (stream->available() > 0) {
      serialData = stream->read();
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
  while (stream->available() > 0) {
    stream->read();
  }
  if (isPacketCompleted == 1 && dabReturn[1] == 0x07) {
    return dabReturn[2] + 1;
  } else {
    return 0;
  }
}

/*
 *  Send command to DAB module and wait for answer
 */
bool T3BTuner::commandSend() {

  uint8_t dabReturn[6];
  uint8_t isPacketCompleted = 0;
  uint16_t byteIndex = 0;
  uint16_t dataIndex = 0;
  uint8_t serialData = 0;
  responseSize = 0;
  while (stream->available() > 0) {
    stream->read();
  }
  while (byteIndex < 255) {
    if (command[byteIndex++] == 0xFD) break;
  }
  stream->write(command, byteIndex);
  stream->flush();
  byteIndex = 0;
  unsigned long endMillis = millis() + 200; // timeout for answer from module = 200ms
  while (millis() < endMillis && dataIndex < DAB_MAX_DATA_LENGTH) {
    if (stream->available() > 0) {
      serialData = stream->read();
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

// *************************
// ***** SYSTEM ************
// *************************

/*
 *   Test for DAB module is ready for communication
 */
bool T3BTuner::Ready()
{
  commandCreate(CMD_SYSTEM, SYSTEM_READY);
  return commandSend();
}

/*
 *   Reset module.
 *   FullReset => Reset module database & module.
 */
bool T3BTuner::Reset(bool fullReset)
{
  commandCreate(CMD_SYSTEM, SYSTEM_RESET, (uint8_t)fullReset);
  bool result = commandSend();
  if (result) init();  
  return result;
}

/*
 *   Set audio output channels (SPDIV, CINCH /I2S DAC/)
 *   CINCH for analog output, SPDIV for optical digital output
 */
bool T3BTuner::AudioOutput(bool spdiv, bool cinch)
{
  uint8_t param = (uint8_t)spdiv | ((uint8_t)cinch << 0x1);
  commandCreate(CMD_SYSTEM, SYSTEM_AUDIO_OUTPUT, param);
  return commandSend();
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
  commandCreatePlay(STREAM_PLAY_DAB, stationId);
  return commandSend();
}

/*
 *   Play FM program
 *   frequency = 87500..108000 (MHz)
 */
bool T3BTuner::PlayFm(uint32_t frequency)
{
  commandCreatePlay(STREAM_PLAY_FM, frequency);
  return commandSend();
}

/* 
 *   Play Beep.
 */
bool T3BTuner::PlayBeep()
{
  commandCreatePlay(STREAM_PLAY_BEEP, 0x00);
  return commandSend();
}

/*
 *   Stop.
 */
bool T3BTuner::Stop()
{
  commandCreate(CMD_STREAM, STREAM_STOP);
  return commandSend();
}

/*
 * Seek FM program.
 */
bool T3BTuner::FmSearch(bool searchForward)
{
  commandCreate(CMD_STREAM, STREAM_FM_SEARCH, (uint8_t)searchForward);
  return commandSend();
}

/*
 * Search DAB bands for programs.
 */
bool T3BTuner::DabSearch(DabBand band)
{
  commandStart(CMD_STREAM, STREAM_DAB_SEARCH);
  switch (band)
  {
  case DabBand::Band3:
    commandAppend((uint8_t)0);
    commandAppend((uint8_t)40);
    break;
  case DabBand::ChinaBand:
    commandAppend((uint8_t)41);
    commandAppend((uint8_t)71);
    break;
  case DabBand::LBand:
    commandAppend((uint8_t)72);
    commandAppend((uint8_t)94);
    break;
  }
  commandEnd();
  return commandSend();
}

/*
 *   Radio module play status.
 */
bool T3BTuner::State(TunerState *status)
{
  commandCreate(CMD_STREAM, STREAM_STATUS);
  bool result = commandSend();
  *status = (TunerState)response[0];
  return result;
}

/*
 *   Radio module play mode.
 */
bool T3BTuner::Mode(TunerMode *mode)
{
  commandCreate(CMD_STREAM, STREAM_MODE);
  bool result = commandSend();
  *mode = (TunerMode)response[0];
  return result;
}

/*
 * Get DAB stationId, get FM frequency.
 */
bool T3BTuner::NowPlaying(uint32_t* programId)
{
  commandCreate(CMD_STREAM, STREAM_NOW_PLAYING);
  bool result = commandSend();
  *programId = ((uint32_t)response[3]);
  *programId |= ((uint32_t)response[2] << 8);
  *programId |= ((uint32_t)response[1] << 16);
  *programId |= ((uint32_t)response[0] << 24);
  return result;
}

/*
 * Get signal strength
 * DAB: signalStrength=0..18, bitErrorRate=
 * FM: signalStrength=0..100
 */
bool T3BTuner::SignalStrength(uint8_t* signalStrength, uint16_t* bitErrorRate)
{
  commandCreate(CMD_STREAM, STREAM_SIGNAL_STRENGTH);
  bool result = commandSend();
  *signalStrength = response[0];
  *bitErrorRate = (uint16_t)response[2];
  *bitErrorRate |= (uint16_t)response[1] << 8;
  return result;
}

/*
 *   Set stereo mode.
 */
bool T3BTuner::StereoModeSet(StereoMode stereoMode)
{
  commandCreate(CMD_STREAM, STREAM_STEREO_MODE_SET, (uint8_t)stereoMode);
  return commandSend();
}

/*
 *   Get stereo mode.
 */
bool T3BTuner::StereoModeGet(StereoMode* stereoMode)
{
  commandCreate(CMD_STREAM, STREAM_STEREO_MODE_GET);
  bool result = commandSend();
  *stereoMode = (StereoMode)response[0];
  return result;
}

/*
 *   Get stereo type
 */
bool T3BTuner::StereoTypeGet(StereoType* stereotype)
{
  commandCreate(CMD_STREAM, STREAM_STEREO_TYPE);
  bool result = commandSend();
  *stereotype = (StereoType)response[0];
  return result;
}

/*
 *   Set volume.
 *   volumeLevel = 0..16
 */
bool T3BTuner::VolumeSet(uint8_t volume)
{
  uint8_t volumeValue = (volume > 16) ? 16 : volume;
  commandCreate(CMD_STREAM, STREAM_VOLUME_SET, volumeValue);
  return commandSend();
}

/*
 *   Get volume.
 *   return set volumeLevel: 0..16
 */
bool T3BTuner::VolumeGet(uint8_t *volume)
{
  commandCreate(CMD_STREAM, STREAM_VOLUME_GET);
  bool result = commandSend();
  *volume = response[0];
  return result;
}

/*
 *   Get program type.
 */
bool T3BTuner::StationTypeGet(StationType* programType)
{
  commandCreate(CMD_STREAM, STREAM_STATION_TYPE);
  bool result = commandSend();
  *programType = (StationType)response[0];
  return result;
}

/*
 * Get DAB station name.
 */
bool T3BTuner::DabStationName(uint32_t stationId, char* name, bool longName)
{
  commandCreateName(STREAM_DAB_STATION_NAME, stationId, longName);
  bool result = commandSend();
  uint8_t j = 0;
  for (uint32_t i = 0; i < responseSize; i = i + 2)
  {
    name[j++] = charToAscii(response[i], response[i + 1]);
  }
  return result;
}

/*
 * Get DAB text event
 * return: 1=new text, 2=text is same, 3=no text
 * dabText: text
 */
bool T3BTuner::DabStationText(char* text)
{
  commandCreate(CMD_STREAM, STREAM_DAB_STATION_TEXT);
  if (commandSend())
  {
    if (responseSize == 1)
    {
      // No text received
      // Response[0] value => 0 = No text available, 1 = Station text is empty.
      return false;
    }

    // Get text.
    int32_t j = 0;
    for (uint32_t i = 0; i < responseSize; i = i + 2)
    {
      text[j++] = charToAscii(response[i], response[i + 1]);
    }

    bool changed = (strncmp(text, dabProgramText, sizeof(dabProgramText)) == 0);
    strncpy(dabProgramText, text, sizeof(dabProgramText));
    return changed;
  }

  return false;
}

/*
 *   Get sampling rate (DAB/FM).
 */
bool T3BTuner::SampleRateGet(SampleRate* sampleRate)
{
  commandCreate(CMD_STREAM, STREAM_SAMPLE_RATE);
  bool result = commandSend();
  *sampleRate = (SampleRate)response[0];
  return result;
}

/*
 *   Get data rate (DAB)
 *   return data: data rate in kbps
 */
bool T3BTuner::DabDataRate(uint16_t* dataRate)
{
  commandCreate(CMD_STREAM, STREAM_DAB_DATA_RATE);
  bool result = commandSend();
  *dataRate = (uint16_t)response[1];
  *dataRate |= (uint16_t)response[0] << 8;
  return result;
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
  commandCreate(CMD_STREAM, STREAM_DAB_SIGNAL_QUALITY);
  bool result = commandSend();
  *data = response[0];
  return result;
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
  commandCreate(CMD_STREAM, STREAM_DAB_FREQUENCY, stationId);
  bool result = commandSend();
  *frequency = response[0];
  return result;
}

/*
 * Get DAB program ensemble name.
 */
bool T3BTuner::DabStationEnsembleName(uint32_t stationId, char* name, bool longName)
{
  commandCreateName(STREAM_DAB_ENSEMBLE_NAME, stationId, longName);
  bool result = commandSend();
  uint8_t j = 0;
  for (uint32_t i = 0; i < responseSize; i = i + 2)
  {
    name[j++] = charToAscii(response[i], response[i + 1]);
  }
  return result;
}

/*
 * Number of DAB stations in database.
 */
bool T3BTuner::DabStationCount(uint32_t* count)
{
  commandCreate(CMD_STREAM, STREAM_DAB_STATION_COUNT);
  bool result = commandSend();
  *count = ((uint32_t)response[3]);
  *count |= ((uint32_t)response[2] << 8);
  *count |= ((uint32_t)response[1] << 16);
  *count |= ((uint32_t)response[0] << 24);
  return result;
}

/*
 *   Test DAB program is active (on-air)
 *   return: 0=off-air, 1=on-air
 */
bool T3BTuner::DabStationOnAir(uint32_t stationId, bool* onAir)
{
  commandCreate(CMD_STREAM, STREAM_DAB_STATION_ON_AIR, stationId);
  bool result = commandSend();
  *onAir = (bool)response[0];
  return result;
}

/*
 * Get DAB program service short name
 */
bool T3BTuner::DabStationServiceName(uint32_t stationId, char* name, bool longName)
{
  commandCreateName(STREAM_DAB_STATION_SERVICE_NAME, stationId, longName);
  bool result = commandSend();
  uint8_t j = 0;
  for (uint32_t i = 0; i < responseSize; i = i + 2)
  {
    name[j++] = charToAscii(response[i], response[i + 1]);
  }
  return result;
}

/*
 * Number of programs found in search process.
 */
bool T3BTuner::DabFoundStationsCount(uint8_t* count)
{
  commandCreate(CMD_STREAM, STREAM_DAB_FOUND_STATIONS_COUNT);
  bool result = commandSend();
  *count = response[0];
  return result;
}

/*
 * Get DAB program service component type (ASCTy)
 */
bool T3BTuner::DabStationType(uint32_t stationId, DabStreamType* type)
{
  commandCreate(CMD_STREAM, STREAM_DAB_STATION_TYPE, stationId);
  bool result = commandSend();
  *type = (DabStreamType)response[0];
  return result;
}

/*
 *   Set preset
 */
bool T3BTuner::MemorySet(MemoryType mode, MemoryId id, uint32_t programId)
{
  commandStart(CMD_STREAM, STREAM_MEMORY_SET);
  commandAppend((uint8_t)mode);
  commandAppend((uint8_t)id);
  commandAppend(programId);
  commandEnd();
  return commandSend();
}

/*
 *  Get preset
 */
bool T3BTuner::MemoryGet(MemoryType mode, MemoryId id, uint32_t* programId)
{
  commandStart(CMD_STREAM, STREAM_MEMORY_GET);
  commandAppend((uint8_t)mode);
  commandAppend((uint8_t)id);
  commandEnd();
  bool result = commandSend();
  *programId = ((uint32_t)response[3]);
  *programId |= ((uint32_t)response[2] << 8);
  *programId |= ((uint32_t)response[1] << 16);
  *programId |= ((uint32_t)response[0] << 24);
  return result;
}

/*
 * Get station info
 * return serviceId = service id of DAB program
 * return ensembleId = ensemble id of DAB program
 */
bool T3BTuner::DabStationInfo(uint32_t stationId, uint32_t* serviceId, uint16_t* ensembleId)
{
  commandCreate(CMD_STREAM, STREAM_DAB_STATION_INFO, stationId);
  bool result = commandSend();
  *serviceId = ((uint32_t)response[3]);
  *serviceId |= ((uint32_t)response[2] << 8);
  *serviceId |= ((uint32_t)response[1] << 16);
  *serviceId |= ((uint32_t)response[0] << 24);
  *ensembleId = (((uint16_t)response[4] << 8) + (uint16_t)response[5]);
  return result;
}

/*
 *   Get DAB station sort order.
 */
bool T3BTuner::DabSortGet(DabSortOrder *sortOrder)
{
  commandCreate(CMD_STREAM, STREAM_DAB_SORT_GET);
  bool result = commandSend();
  *sortOrder = (DabSortOrder)response[0];
  return result;
}

/*
 *   Set DAB station sort order.
 */
bool T3BTuner::DabSortSet(DabSortOrder sortOrder)
{
  commandCreate(CMD_STREAM, STREAM_DAB_SORT_SET, (uint8_t)sortOrder);
  return commandSend();
}

/*
 *   Get DAB DRC.
 */
bool T3BTuner::DabDrcGet(DabDrc* drc)
{
  commandCreate(CMD_STREAM, STREAM_DAB_DRC_GET);
  bool result = commandSend();
  *drc = (DabDrc)response[0];
  return result;
}

/*
 *   Set DAB DRC.
 */
bool T3BTuner::DabDrcSet(DabDrc drc)
{
  commandCreate(CMD_STREAM, STREAM_DAB_DRC_SET, (uint8_t)drc);
  return commandSend();
}

/*
 *   Prune programs - delete inactive programs (!on-air)
 */
bool T3BTuner::DabRemoveOffAir(uint16_t *removedTotal, uint16_t *removedIndex)
{
  commandCreate(CMD_STREAM, STREAM_DAB_REMOVE_OFF_AIR);
  bool result = commandSend();
  *removedTotal = (((uint16_t)response[0] << 8) + (uint16_t)response[1]);
  *removedIndex = (((uint16_t)response[2] << 8) + (uint16_t)response[3]);
  return result;
}

/*
 * Get ECC
 * return ECC (Extended Country Code)
 * return countryId (Country identification)
 */
bool T3BTuner::DabExtendedCountryCode(uint8_t* ecc, uint8_t* countryId)
{
  commandCreate(CMD_STREAM, STREAM_DAB_EXTENDED_COUNTRY_CODE);
  bool result = commandSend();
  *ecc = response[0];
  *countryId = response[1];
  return result;
}

/*
 *   Get FM RDS PI code
 */
bool T3BTuner::FmRdsPiCode(uint16_t *code)
{
  commandCreate(CMD_STREAM, STREAM_FM_RDS_PI_CODE);
  bool result = commandSend();
  *code = (uint16_t)response[1];
  *code |= (uint16_t)response[0] << 8;
  return result;
}

/*
 *   Set FMstereoThdLevel
 *   RSSItresholdLevel = 0..10
 */
bool T3BTuner::FmStereoTresholdLevelSet(uint8_t level)
{
  commandCreate(CMD_STREAM, STREAM_FM_STEREO_THRESHOLD_LEVEL_SET, level);
  return commandSend();
}

/*
 *   Get FMstereoThdLevel
 *   data return = 0..10
 */
bool T3BTuner::FmStereoTresholdLevelGet(uint8_t* level)
{
  commandCreate(CMD_STREAM, STREAM_FM_STEREO_THRESHOLD_LEVEL_GET);
  bool result = commandSend();
  *level = response[0];
  return result;
}

/*
 *   Get RDS raw data
 *   return: 1=new RDS data, 2=no new RDS data, 3=no RDS data
 */
bool T3BTuner::FmRdsRawData(uint16_t* blockA, uint16_t* blockB, uint16_t* blockC, uint16_t* blockD, 
                            uint16_t* blerA, uint16_t* blerB, uint16_t* blerC, uint16_t* blerD)
{
  commandCreate(CMD_STREAM, STREAM_FM_RDS_DATA);
  if (commandSend() && responseSize > 1)
  {
    *blockA = (((uint16_t)response[0] << 8) + (uint16_t)response[1]);
    *blockB = (((uint16_t)response[2] << 8) + (uint16_t)response[3]);
    *blockC = (((uint16_t)response[4] << 8) + (uint16_t)response[5]);
    *blockD = (((uint16_t)response[6] << 8) + (uint16_t)response[7]);
    *blerA = (((uint16_t)response[8] << 8) + (uint16_t)response[9]);
    *blerB = (((uint16_t)response[10] << 8) + (uint16_t)response[11]);
    *blerC = (((uint16_t)response[12] << 8) + (uint16_t)response[13]);
    *blerD = (((uint16_t)response[14] << 8) + (uint16_t)response[15]);
    return true;
  }
  
  return false;
}

/*
 *   Set FMseekTreshold
 *   RSSItreshold = 0..100
 */
bool T3BTuner::FmSeekTresholdSet(uint8_t treshold)
{
  commandCreate(CMD_STREAM, STREAM_FM_SEEK_TRESHOLD_SET, treshold);
  return commandSend();
}

/*
 *   Get FMseekTreshold
 *   data return = 0..100
 */
bool T3BTuner::FmSeekTresholdGet(uint8_t* treshold)
{
  commandCreate(CMD_STREAM, STREAM_FM_SEEK_TRESHOLD_GET);
  bool result = commandSend();
  *treshold = response[0];
  return result;
}

/*
 *   Set FMstereoTreshold
 *   RSSItreshold = 0..100
 */
bool T3BTuner::FmStereoTresholdSet(uint8_t treshold)
{
  commandCreate(CMD_STREAM, STREAM_FM_STEREO_TRESHOLD_SET, treshold);
  return commandSend();
}

/*
 *   Get FMstereoTreshold
 *   data return = 0..100
 */
bool T3BTuner::FmStereoTresholdGet(uint8_t* treshold)
{
  commandCreate(CMD_STREAM, STREAM_FM_STEREO_TRESHOLD_GET);
  bool result = commandSend();
  *treshold = response[0];
  return result;
}

/*
 *   Get FM Exact station.
 */
bool T3BTuner::FmExactStationGet(FmExactStation* exact)
{
  commandCreate(CMD_STREAM, STREAM_FM_EXACT_STATION);
  bool result = commandSend();
  *exact = (FmExactStation)response[0];
  return result;
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
  commandStart(CMD_RTC, RTC_SET);
  commandAppend(second);
  commandAppend(minute);
  commandAppend(hour);
  commandAppend(day);
  commandAppend((uint8_t)0x00);
  commandAppend(month);
  commandAppend(year);
  commandEnd();
  return commandSend();
}

/*
 *  Get RTC ckock
 *  year: 2017=17,2018=18, month: 1..12, day: 1..31, hour: 0..23, minute: 0..59, second: 0..59 
 */
bool T3BTuner::ClockGet(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second)
{
  commandCreate(CMD_RTC, RTC_GET);
  bool result = commandSend();
  *second = response[0];
  *minute = response[1];
  *hour = response[2];
  *day = response[3];
  *month = response[5];
  *year = response[6];
  return result;
}

/*
 *  Set RTC sync clock from stream enable
 */
bool T3BTuner::ClockSyncStatusSet(bool enable)
{
  commandCreate(CMD_RTC, RTC_SYNC, (uint8_t)enable);
  return commandSend();
}

/*
 *  Get RTC sync clock status
 */
bool T3BTuner::ClockSyncStatusGet(bool *enabled)
{
  commandCreate(CMD_RTC, RTC_SYNC_STATUS);
  bool result = commandSend();
  *enabled = response[0];
  return result;
}

/*
 *  Get RTC clock status
 */
bool T3BTuner::ClockStatusGet(ClockStatus *status)
{
  commandCreate(CMD_RTC, RTC_STATUS_CLOCK);
  bool result = commandSend();
  *status = (ClockStatus)response[0];
  return result;
}

// *************************
// ***** NOTIFY ************
// *************************

/*
 *   Enabled / Disable event notifications.
 */
bool T3BTuner::Notifications(bool enable)
{
  uint8_t value = (enable) ? 0x7F : 0x00;
  commandCreate(CMD_EVENTS, EVENTS_NOTIFY, value);
  return commandSend();
}