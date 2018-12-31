/*
 * DABDUINO.h - Library for DABDUINO - DAB/DAB+ digital radio shield for Arduino.
 * Created by Tomas Urbanek, Montyho technology Ltd., Januar 2, 2017.
 * www.dabduino.com
 * @license  BSD (see license.txt)
 */

#ifndef T3BTUNER_H
#define T3BTUNER_H

#include <inttypes.h>
#include <stream.h>

#define DAB_MAX_TEXT_LENGTH 128
#define UNUSED_PIN 255
#define DAB_MAX_DATA_LENGTH 2 * DAB_MAX_TEXT_LENGTH

#define COMMAND_MAX_SIZE 20

enum StreamType
{
  STREAM_HARDWARE,
  STREAM_SOFTWARE
};

enum DabSystemReset
{
  DabSystemResetModule = 0x00,
  DabSystemResetFull = 0x01
};

class T3BTuner
{
public:

  T3BTuner(Stream* stream, StreamType streamType, uint8_t resetPin, uint8_t dacMutePin = UNUSED_PIN, uint8_t spiCsPin = UNUSED_PIN);

  char charToAscii(uint8_t byte0, uint8_t byte1);

  void init();

  int8_t isEvent();
  int8_t readEvent();
  int8_t sendCommand(uint8_t dabCommand[], uint8_t dabData[], uint32_t *dabDataSize);
  
  // *************************
  // ***** SYSTEM ************
  // *************************

  bool Ready();
  bool Reset(DabSystemReset reset = DabSystemResetModule);
  bool AudioOutput(bool spdiv = true, bool cinch = true);

  // *************************
  // ***** STREAM ************
  // *************************

  int8_t playDAB(uint32_t programIndex);
  int8_t playFM(uint32_t frequency);
  int8_t playBEEP();
  int8_t playSTOP();
  int8_t searchDAB(uint32_t band = 1);
  int8_t searchFM(uint32_t seekDirection);
  int8_t playStatus(uint32_t *data);
  int8_t playMode(uint32_t *data);
  int8_t getPlayIndex(uint32_t *data);
  int8_t getSignalStrength(uint32_t *signalStrength, uint32_t *bitErrorRate);
  int8_t setStereoMode(bool stereo = true);
  int8_t getStereoMode(uint32_t *data);
  int8_t getStereoType(uint32_t *data);
  int8_t setVolume(uint32_t volumeLevel);
  int8_t getVolume(uint32_t *data);
  int8_t getProgramType(uint32_t *data);
  int8_t getProgramShortName(uint32_t programIndex, char text[]);
  int8_t getProgramLongName(uint32_t programIndex, char text[]);
  int8_t getProgramText(char text[]);
  int8_t getSamplingRate(uint32_t *data);
  int8_t getDataRate(uint32_t *data);
  int8_t getSignalQuality(uint32_t *data);
  int8_t getFrequency(uint32_t programIndex, uint32_t *data);
  int8_t getEnsembleShortName(uint32_t programIndex, char text[]);
  int8_t getEnsembleLongName(uint32_t programIndex, char text[]);
  int8_t getProgramIndex(uint32_t *data);
  int8_t isProgramOnAir(uint32_t programIndex);
  int8_t getServiceShortName(uint32_t programIndex, char text[]);
  int8_t getServiceLongName(uint32_t programIndex, char text[]);
  int8_t getSearchIndex(uint32_t *data);
  int8_t getServCompType(uint32_t programIndex, uint32_t *data);
  int8_t setPreset(uint32_t programIndex, uint32_t presetIndex, uint32_t presetMode);
  int8_t getPreset(uint32_t presetIndex, uint32_t presetMode, uint32_t *data);
  int8_t getProgramInfo(uint32_t programIndex, uint32_t *serviceId, uint32_t *ensembleId);
  int8_t getProgramSorter(uint32_t *data);
  int8_t setProgramSorter(uint32_t sortMethod);
  int8_t getDRC(uint32_t *data);
  int8_t setDRC(uint32_t setDRC);
  int8_t prunePrograms(uint32_t *prunedTotalPrograms, uint32_t *prunedProgramIndex);
  int8_t getECC(uint32_t *ECC, uint32_t *countryId);
  int8_t getRdsPIcode(uint32_t *data);
  int8_t setFMstereoThdLevel(uint32_t RSSItresholdLevel);
  int8_t getFMstereoThdLevel(uint32_t *data);
  int8_t getRDSrawData(uint32_t *RDSblockA, uint32_t *RDSblockB, uint32_t *RDSblockC, uint32_t *RDSblockD, uint32_t *BlerA, uint32_t *BlerB, uint32_t *BlerC, uint32_t *BlerD);
  int8_t setFMseekTreshold(uint32_t RSSItreshold);
  int8_t getFMseekTreshold(uint32_t *data);
  int8_t setFMstereoTreshold(uint32_t RSSIstereoTreshold);
  int8_t getFMstereoTreshold(uint32_t *data);
  int8_t getFMexactStation(uint32_t *data);

  // *************************
  // ***** RTC ***************
  // *************************

  int8_t setRTCclock(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second);
  int8_t getRTCclock(uint32_t *year, uint32_t *month, uint32_t *week, uint32_t *day, uint32_t *hour, uint32_t *minute, uint32_t *second);
  int8_t RTCsyncEnable();
  int8_t RTCsyncDisable();
  int8_t getRTCsyncStatus(uint32_t *data);
  int8_t getRTCclockStatus(uint32_t *data);

  // ********************************************
  // ***** MOT (Multimedia Object Transfer) *****
  // ********************************************



  // *************************
  // ***** NOTIFY ************
  // *************************

  int8_t eventNotificationEnable();
  int8_t eventNotificationDisable();

private:
  void commandAppend(uint8_t data);
  void commandAppend(uint32_t data);
  void commandStart(uint8_t type, uint8_t command, uint8_t option);
  void commandEnd();
  void commandCreate(uint8_t type, uint8_t command, uint8_t option);
  void commandCreate(uint8_t type, uint8_t command, uint8_t option, uint8_t param);
  bool commandSend();

  void streamBegin(uint32_t baud);

  uint8_t command[COMMAND_MAX_SIZE];
  uint8_t commandSize = 0;
  uint8_t response[DAB_MAX_DATA_LENGTH];
  uint32_t responseSize = 0;

  Stream * stream;
  StreamType streamType;
  uint8_t resetPin;
  uint8_t dacMutePin;
  uint8_t spiCsPin;
};

#endif
