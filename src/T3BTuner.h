#ifndef T3BTuner_h
#define T3BTuner_h

#include "CommandBuilder.h"
#include "ISerialStream.h"
#include <stdint.h>

static const uint16_t T3BTunerMaxTextSize = 128U;
static const uint16_t T3BTunerMaxDataSize = 2 * T3BTunerMaxTextSize;
static const uint8_t T3BTunerHeaderSize = 6U;

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
    T3BTuner(ISerialStream* const serial, uint8_t const resetPin, uint8_t const mutePin);
    T3BTuner(ISerialStream* const serial, uint8_t const resetPin, uint8_t const mutePin, uint8_t const spiCsPin);
    void Init();

    // *************************
    // ***** SYSTEM ************
    // *************************
    bool Ready();
    bool Reset(bool const fullReset = false);
    bool AudioOutput(bool const spdif = true, bool const cinch = true);

    // *************************
    // ***** STREAM ************
    // *************************
    bool PlayDab(uint32_t const stationId);
    bool PlayFm(uint32_t const frequency);
    bool PlayBeep();
    bool Stop();
    bool FmSearch(bool const searchForward = true);
    bool DabSearch(DabBand const band = DabBand::BandIII);
    bool State(TunerState* const status);
    bool Mode(TunerMode* const mode);
    bool NowPlaying(uint32_t* const programId);
    bool SignalStrength(uint8_t* const signalStrength, uint16_t* const bitErrorRate);
    bool StereoModeSet(StereoMode const stereoMode = StereoMode::Stereo);
    bool StereoModeGet(StereoMode* const stereoMode);
    bool StereoTypeGet(StereoType* const stereotype);
    bool VolumeSet(uint8_t const volume);
    bool VolumeGet(uint8_t* const volume);
    bool StationTypeGet(StationType* const programType);
    bool DabStationName(uint32_t const stationId, char* const buffer, uint16_t const size, bool const longName = true);
    bool DabStationText(char* const buffer, uint16_t const size);
    bool SampleRateGet(SampleRate* const sampleRate);
    bool DabDataRate(uint16_t* const dataRate);
    bool DabSignalQuality(uint8_t* const signalQuality);
    bool DabStationFrequency(uint32_t const stationId, uint8_t* const frequency);
    bool DabStationEnsembleName(uint32_t const stationId, char* const buffer, uint16_t const size);
    bool DabStationCount(uint32_t* const count);
    bool DabStationOnAir(uint32_t const stationId, bool* const onAir);
    bool DabStationServiceName(uint32_t const stationId, char* const buffer, uint16_t const size);
    bool DabFoundStationsCount(uint8_t* const count);
    bool DabStationType(uint32_t const stationId, DabStreamType* const type);
    bool MemorySet(MemoryType const mode, MemoryId const id, uint32_t const programId);
    bool MemoryGet(MemoryType const mode, MemoryId const id, uint32_t* const programId);
    bool DabStationInfo(uint32_t const stationId, uint32_t* const serviceId, uint16_t* const ensembleId);
    bool DabSortGet(DabSortOrder* const data);
    bool DabSortSet(DabSortOrder const sortMethod);
    bool DabDrcGet(DabDrc* const drc);
    bool DabDrcSet(DabDrc const drc);
    bool DabRemoveOffAir(uint16_t* const removedTotal, uint16_t* const removedIndex);
    bool DabExtendedCountryCode(uint8_t* const ecc, uint8_t* const countryId);
    bool FmRdsPiCode(uint16_t* const code);
    bool FmStereoThresholdLevelSet(uint8_t const level);
    bool FmStereoThresholdLevelGet(uint8_t* const level);
    bool FmRdsRawData(uint16_t* const blockA, uint16_t* const blockB, uint16_t* const blockC, uint16_t* const blockD, uint16_t* const blerA,
                      uint16_t* const blerB, uint16_t* const blerC, uint16_t* const blerD);
    bool FmSeekThresholdSet(uint8_t const threshold);
    bool FmSeekThresholdGet(uint8_t* const threshold);
    bool FmStereoThresholdSet(uint8_t const threshold);
    bool FmStereoThresholdGet(uint8_t* const threshold);
    bool FmExactStationGet(FmExactStation* const exactStation);

    // *************************
    // ***** RTC ***************
    // *************************
    bool ClockSet(uint8_t const year, uint8_t const month, uint8_t const day, uint8_t const hour, uint8_t const minute, uint8_t const second);
    bool ClockGet(uint8_t* const year, uint8_t* const month, uint8_t* const day, uint8_t* const hour, uint8_t* const minute,
                  uint8_t* const second);
    bool ClockSyncSet(bool const enable);
    bool ClockSyncGet(bool* const enabled);
    bool ClockStatusGet(ClockStatus* const status);

    // *************************
    // ***** EVENTS ************
    // *************************
    bool EventEnable(bool const enable);
    bool EventReceived();
    bool EventRead(EventType* const type);

  private:
    // *************************
    // ***** PRIVATE FUNCTIONS *
    // *************************
    bool CommandSend(Command const& command);
    bool ResponseReceive();
    bool ResponseText(char* const buffer, uint16_t const size);
    bool ResponseUint8(uint8_t const index, uint8_t* const resp);
    bool ResponseUint16(uint8_t const index, uint16_t* const resp);
    bool ResponseUint32(uint8_t const index, uint32_t* const resp);
    char Uint16ToChar(uint8_t const byte0, uint8_t const byte1);

    CommandBuilder commandBuilder;
    uint8_t response[T3BTunerMaxDataSize];
    uint8_t responseHeader[T3BTunerHeaderSize];
    uint16_t responseSize = 0U;
    char dabStationText[T3BTunerMaxTextSize];

    ISerialStream* serial;
    uint8_t pinReset;
    uint8_t pinMute;
    uint8_t pinSpiCs;
};

#endif // T3BTuner_h
