#ifndef T3BTUNER_H
#define T3BTUNER_H

#include <inttypes.h>
#include "CommandBuilder.h"
#include "ISerialStream.h"

#define DAB_MAX_TEXT_LENGTH 128
#define HEADER_SIZE 6
#define UNUSED_PIN 255
#define DAB_MAX_DATA_LENGTH 2 * DAB_MAX_TEXT_LENGTH

#define COMMAND_MAX_SIZE 20

enum class DabBand : uint8_t
{
  BandIII = 0x01,
  ChinaBand = 0x02,
  LBand = 0x03
};

enum class TunerState : uint8_t
{
  Playing,
  Searching,
  Tuning,
  Stopped,
  Sorting,
  Reconfiguring
};

enum class TunerMode : uint8_t
{
  Dab = 0x00,
  Fm = 0x01,
  Beep = 0x03,
  Am = 0x04,
  None = 0xFF
};

enum class StereoMode : uint8_t
{
  Mono,
  Stereo
};

enum class StereoType : uint8_t
{
  Stereo,
  JointStereo,
  DualChannel,
  SingleChannel
};

enum class StationType : uint8_t
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

enum class SampleRate : uint8_t
{
  Khz32 = 0x01,
  Khz24,
  Khz48
};

enum class DabStreamType : uint8_t
{
  Dab,
  DabPlus,
  PacketData,
  StreamData
};

enum class DabSortOrder : uint8_t
{
  EnsembleId,
  ServiceName,
  ActiveInactive
};

enum class MemoryType : uint8_t
{
  Dab,
  Fm
};

enum class MemoryId : uint8_t
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

enum class DabDrc : uint8_t
{
  Off,
  Low,
  High
};

enum class FmExactStation : uint8_t
{
  NotExact,
  Exact,
  NoInformationYet = 0xFE
};

enum class ClockStatus : uint8_t
{
  Unset,
  Set
};

enum class EventType : uint8_t
{
  FinishedScan,
  DabStationText,
  DabReconfigured,
  DabSorted,
  FmRdsGroup,
  FmStationText,
  ScanFrequency
};

class T3BTuner
{
public:
  T3BTuner(ISerialStream* serial, uint8_t resetPin, uint8_t mutePin = UNUSED_PIN,
           uint8_t spiCsPin = UNUSED_PIN);
  void Init();
  
  // *************************
  // ***** SYSTEM ************
  // *************************
  bool Ready();
  bool Reset(bool fullReset = false);
  bool AudioOutput(bool spdif = true, bool cinch = true);

  // *************************
  // ***** STREAM ************
  // *************************
  bool PlayDab(uint32_t stationId);
  bool PlayFm(uint32_t frequency);
  bool PlayBeep();
  bool Stop();
  bool FmSearch(bool searchForward = true);
  bool DabSearch(DabBand band = DabBand::BandIII);
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
  bool DabStationName(uint32_t stationId, char* buffer, uint16_t size, bool longName = true);
  bool DabStationText(char* buffer, uint16_t size);
  bool SampleRateGet(SampleRate *sampleRate);
  bool DabDataRate(uint16_t *dataRate);
  bool DabSignalQuality(uint8_t *signalQuality);
  bool DabStationFrequency(uint32_t stationId, uint8_t* frequency);
  bool DabStationEnsembleName(uint32_t stationId, char* buffer, uint16_t size);
  bool DabStationCount(uint32_t* count);
  bool DabStationOnAir(uint32_t stationId, bool* onAir);
  bool DabStationServiceName(uint32_t stationId, char* buffer, uint16_t size);
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
  bool FmStereoThresholdLevelSet(uint8_t level);
  bool FmStereoThresholdLevelGet(uint8_t* level);
  bool FmRdsRawData(uint16_t* blockA, uint16_t* blockB, uint16_t* blockC, uint16_t* blockD,
                    uint16_t* blerA, uint16_t* blerB, uint16_t* blerC, uint16_t* blerD);
  bool FmSeekThresholdSet(uint8_t threshold);
  bool FmSeekThresholdGet(uint8_t* threshold);
  bool FmStereoThresholdSet(uint8_t threshold);
  bool FmStereoThresholdGet(uint8_t* threshold);
  bool FmExactStationGet(FmExactStation* exactStation);

  // *************************
  // ***** RTC ***************
  // *************************
  bool ClockSet(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
  bool ClockGet(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second);
  bool ClockSyncSet(bool enable);
  bool ClockSyncGet(bool *enabled);
  bool ClockStatusGet(ClockStatus *status);

  // *************************
  // ***** EVENTS ************
  // *************************
  bool EventEnable(bool enable);
  bool EventReceived();
  bool EventRead(EventType* type);

private:
  // *************************
  // ***** PRIVATE FUNCTIONS *
  // *************************
  bool CommandSend(Command& command);
  bool ResponseReceive();
  bool ResponseText(char* buffer, uint16_t size);
  bool ResponseUint8(uint8_t index, uint8_t * resp);
  bool ResponseUint16(uint8_t index, uint16_t * resp);
  bool ResponseUint32(uint8_t index, uint32_t * resp);
  char Uint16ToChar(uint8_t byte0, uint8_t byte1);

  CommandBuilder commandBuilder;
  uint8_t response[DAB_MAX_DATA_LENGTH];
  uint8_t responseHeader[HEADER_SIZE];
  uint16_t responseSize = 0;
  char dabStationText[DAB_MAX_TEXT_LENGTH];

  ISerialStream * serial;
  uint8_t pinReset;
  uint8_t pinMute;
  uint8_t pinSpiCs;
};

#endif
