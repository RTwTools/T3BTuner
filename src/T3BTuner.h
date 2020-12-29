#ifndef T3BTuner_h
#define T3BTuner_h

#include "CommandBuilder.h"
#include "ISerialStream.h"
#include <stdint.h>

static uint16_t const T3BTunerMaxTextSize = 128U;
static uint16_t const T3BTunerMaxDataSize = 2 * T3BTunerMaxTextSize;
static uint8_t const T3BTunerHeaderSize = 6U;

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
    T3BTuner(ISerialStream* const serial, uint8_t const resetPin, uint8_t const mutePin,
             uint8_t const spiCsPin);
    void init();

    // *************************
    // ***** SYSTEM ************
    // *************************
    bool ready();
    bool reset(bool const fullReset = false);
    bool audioOutput(bool const spdif = true, bool const cinch = true);

    // *************************
    // ***** STREAM ************
    // *************************
    bool playDab(uint32_t const stationId);
    bool playFm(uint32_t const frequency);
    bool playBeep();
    bool stop();
    bool fmSearch(bool const searchForward = true);
    bool dabSearch(DabBand const band = DabBand::BandIII);
    bool state(TunerState* const status);
    bool mode(TunerMode* const mode);
    bool nowPlaying(uint32_t* const programId);
    bool signalStrength(uint8_t* const signalStrength, uint16_t* const bitErrorRate);
    bool stereoModeSet(StereoMode const stereoMode = StereoMode::Stereo);
    bool stereoModeGet(StereoMode* const stereoMode);
    bool stereoTypeGet(StereoType* const stereotype);
    bool volumeSet(uint8_t const volume);
    bool volumeGet(uint8_t* const volume);
    bool stationTypeGet(StationType* const programType);
    bool dabStationName(uint32_t const stationId, char* const buffer, uint16_t const size,
                        bool const longName = true);
    bool dabStationText(char* const buffer, uint16_t const size);
    bool sampleRateGet(SampleRate* const sampleRate);
    bool dabDataRate(uint16_t* const dataRate);
    bool dabSignalQuality(uint8_t* const signalQuality);
    bool dabStationFrequency(uint32_t const stationId, uint8_t* const frequency);
    bool dabStationEnsembleName(uint32_t const stationId, char* const buffer, uint16_t const size);
    bool dabStationCount(uint32_t* const count);
    bool dabStationOnAir(uint32_t const stationId, bool* const onAir);
    bool dabStationServiceName(uint32_t const stationId, char* const buffer, uint16_t const size);
    bool dabFoundStationsCount(uint8_t* const count);
    bool dabStationType(uint32_t const stationId, DabStreamType* const type);
    bool memorySet(MemoryType const mode, MemoryId const id, uint32_t const programId);
    bool memoryGet(MemoryType const mode, MemoryId const id, uint32_t* const programId);
    bool dabStationInfo(uint32_t const stationId, uint32_t* const serviceId, uint16_t* const ensembleId);
    bool dabSortGet(DabSortOrder* const data);
    bool dabSortSet(DabSortOrder const sortMethod);
    bool dabDrcGet(DabDrc* const drc);
    bool dabDrcSet(DabDrc const drc);
    bool dabRemoveOffAir(uint16_t* const removedTotal, uint16_t* const removedIndex);
    bool dabExtendedCountryCode(uint8_t* const ecc, uint8_t* const countryId);
    bool fmRdsPiCode(uint16_t* const code);
    bool fmStereoThresholdLevelSet(uint8_t const level);
    bool fmStereoThresholdLevelGet(uint8_t* const level);
    bool fmRdsRawData(uint16_t* const blockA, uint16_t* const blockB, uint16_t* const blockC,
                      uint16_t* const blockD, uint16_t* const blerA, uint16_t* const blerB,
                      uint16_t* const blerC, uint16_t* const blerD);
    bool fmSeekThresholdSet(uint8_t const threshold);
    bool fmSeekThresholdGet(uint8_t* const threshold);
    bool fmStereoThresholdSet(uint8_t const threshold);
    bool fmStereoThresholdGet(uint8_t* const threshold);
    bool fmExactStationGet(FmExactStation* const exactStation);

    // *************************
    // ***** RTC ***************
    // *************************
    bool clockSet(uint8_t const year, uint8_t const month, uint8_t const day, uint8_t const hour,
                  uint8_t const minute, uint8_t const second);
    bool clockGet(uint8_t* const year, uint8_t* const month, uint8_t* const day, uint8_t* const hour,
                  uint8_t* const minute, uint8_t* const second);
    bool clockSyncSet(bool const enable);
    bool clockSyncGet(bool* const enabled);
    bool clockStatusGet(ClockStatus* const status);

    // *************************
    // ***** EVENTS ************
    // *************************
    bool eventEnable(bool const enable);
    bool eventReceived();
    bool eventRead(EventType* const type);

  private:
    // *************************
    // ***** PRIVATE FUNCTIONS *
    // *************************
    bool commandSend(Command const& command);
    bool responseReceive();
    bool responseText(char* const buffer, uint16_t const size);
    bool responseUint8(uint8_t const index, uint8_t* const resp);
    bool responseUint16(uint8_t const index, uint16_t* const resp);
    bool responseUint32(uint8_t const index, uint32_t* const resp);
    char uint16ToChar(uint8_t const byte0, uint8_t const byte1);

    CommandBuilder commandBuilder;
    uint8_t response[T3BTunerMaxDataSize];
    uint8_t responseHeader[T3BTunerHeaderSize];
    uint16_t responseSize = 0U;
    char stationText[T3BTunerMaxTextSize];

    ISerialStream* serial;
    uint8_t pinReset;
    uint8_t pinMute;
    uint8_t pinSpiCs;
};

#endif // T3BTuner_h
