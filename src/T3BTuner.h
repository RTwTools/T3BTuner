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

enum class StreamType
{
  HardwareSerial,
  SoftwareSerial
};

enum class DabBand
{
  Band3 = 0x01,
  ChinaBand = 0x02,
  LBand = 0x03
};

enum class TunerState
{
  Playing,
  Searching,
  Tuning,
  Stopped,
  Sorting,
  Reconfiguring
};

enum class TunerMode
{
  Dab,
  Fm,
  Beep,
  None
};

enum class StereoMode
{
  Mono,
  Stereo
};

enum class StereoType
{
  Stereo,
  JoinStereo,
  DualChannel,
  SingleChannel
};

enum class StationType
{
  NotAvailable,
  News,
  CurrentAffairs,
  Information,
  Sport,
  Education,
  Drama,
  Arts,
  Science,
  Talk,
  PopMusic,
  RockMusic,
  EasyListening,
  LightClassical,
  ClassicalMusic,
  OtherMusic,
  Weather,
  Finance,
  Childrens,
  Factual,
  Religion,
  PhoneIn,
  Travel,
  Leisure,
  JazzBlues,
  CountryMusic,
  NationalMusic,
  OldiesMusic,
  FolkMusic,
  Documentary,
  Undefined,
  Undefined2,
};

enum class SampleRate
{
  Khz32 = 0x01, // TODO: 32 and 24 turned?
  Khz24,
  Khz48
};

enum class DabStreamType
{
  Dab,
  DabPlus,
  PacketData,
  StreamData
};

enum class DabSortOrder
{
  EnsembleId,
  ServiceName,
  ActiveInactive
};

enum class MemoryType
{
  Dab,
  Fm
};

enum class MemoryId
{
  Memory01,
  Memory02,
  Memory03,
  Memory04,
  Memory05,
  Memory06,
  Memory07,
  Memory08,
  Memory09,
  Memory10
};

enum class DabDrc
{
  Off,
  Low,
  High
};

enum class FmExactStation
{
  NotExact,
  Exact,
  NoInformationYet = 0xFE
};

enum class ClockStatus
{
  Unset,
  Set
};

class T3BTuner
{
public:
  T3BTuner(Stream* stream, StreamType streamType, uint8_t resetPin,
           uint8_t dacMutePin = UNUSED_PIN, uint8_t spiCsPin = UNUSED_PIN);
  char charToAscii(uint8_t byte0, uint8_t byte1);
  void init();
  int8_t isEvent();
  int8_t readEvent();
  
  // *************************
  // ***** SYSTEM ************
  // *************************
  bool Ready();
  bool Reset(bool fullReset = false);
  bool AudioOutput(bool spdiv = true, bool cinch = true);

  // *************************
  // ***** STREAM ************
  // *************************
  bool PlayDab(uint32_t stationId);
  bool PlayFm(uint32_t frequency);
  bool PlayBeep();
  bool Stop();
  bool FmSearch(bool searchForward = true);
  bool DabSearch(DabBand band = DabBand::Band3);
  bool State(TunerState* status);
  bool Mode(TunerMode* mode);
  bool NowPlaying(uint32_t* programId);
  bool SignalStrength(uint8_t* signalStrength, uint16_t* bitErrorRate);
  bool StereoModeSet(StereoMode stereoMode = StereoMode::Stereo);
  bool StereoModeGet(StereoMode* stereoMode);
  bool StereoTypeGet(StereoType* stereotype);
  bool VolumeSet(uint8_t volume);
  bool VolumeGet(uint8_t *volume);
  bool StationTypeGet(StationType *programType);
  bool DabStationName(uint32_t stationId, char* name, bool longName = true);
  bool DabStationText(char* text);
  bool SampleRateGet(SampleRate *sampleRate);
  bool DabDataRate(uint16_t *data);
  bool DabSignalQuality(uint8_t *data);
  bool DabStationFrequency(uint32_t stationId, uint8_t* frequency);
  bool DabStationEnsembleName(uint32_t stationId, char* name, bool longName = true);
  bool DabStationCount(uint32_t* count);
  bool DabStationOnAir(uint32_t stationId, bool* onAir);
  bool DabStationServiceName(uint32_t stationId, char* name, bool longName = true);
  bool DabFoundStationsCount(uint8_t* count);
  bool DabStationType(uint32_t stationId, DabStreamType* type);
  bool MemorySet(MemoryType mode, MemoryId id, uint32_t programId);
  bool MemoryGet(MemoryType mode, MemoryId id, uint32_t* programId);
  bool DabStationInfo(uint32_t stationId, uint32_t* serviceId, uint16_t* ensembleId);
  bool DabSortGet(DabSortOrder *data);
  bool DabSortSet(DabSortOrder sortMethod);
  bool DabDrcGet(DabDrc* drc);
  bool DabDrcSet(DabDrc drc);
  bool DabRemoveOffAir(uint16_t* removedTotal, uint16_t* removedIndex);
  bool DabExtendedCountryCode(uint8_t* ecc, uint8_t* countryId);
  bool FmRdsPiCode(uint16_t* code);
  bool FmStereoTresholdLevelSet(uint8_t level);
  bool FmStereoTresholdLevelGet(uint8_t* level);
  bool FmRdsRawData(uint16_t* blockA, uint16_t* blockB, uint16_t* blockC, uint16_t* blockD,
                    uint16_t* blerA, uint16_t* blerB, uint16_t* blerC, uint16_t* blerD);
  bool FmSeekTresholdSet(uint8_t treshold);
  bool FmSeekTresholdGet(uint8_t* treshold);
  bool FmStereoTresholdSet(uint8_t treshold);
  bool FmStereoTresholdGet(uint8_t* data);
  bool FmExactStationGet(FmExactStation* exactStation);

  // *************************
  // ***** RTC ***************
  // *************************

  bool ClockSet(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
  bool ClockGet(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second);
  bool ClockSyncStatusSet(bool enable);
  bool ClockSyncStatusGet(bool *enabled);
  bool ClockStatusGet(ClockStatus *status);

  // *************************
  // ***** NOTIFY ************
  // *************************

  bool Notifications(bool enable);

private:
  void commandAppend(uint8_t data);
  void commandAppend(uint32_t data);
  void commandStart(uint8_t type, uint8_t command);
  void commandEnd();
  void commandCreate(uint8_t type, uint8_t subType);
  void commandCreate(uint8_t type, uint8_t command, uint8_t param);
  void commandCreate(uint8_t type, uint8_t subType, uint32_t param);
  void commandCreatePlay(uint8_t playType, uint32_t param);
  void commandCreateName(uint8_t subType, uint32_t program, bool longName);
  bool commandSend();

  void streamBegin(uint32_t baud);

  uint8_t command[COMMAND_MAX_SIZE];
  uint8_t commandSize = 0;
  uint8_t response[DAB_MAX_DATA_LENGTH];
  uint32_t responseSize = 0;
  char dabProgramText[DAB_MAX_TEXT_LENGTH];

  Stream * stream;
  StreamType streamType;
  uint8_t resetPin;
  uint8_t dacMutePin;
  uint8_t spiCsPin;
};

#endif
