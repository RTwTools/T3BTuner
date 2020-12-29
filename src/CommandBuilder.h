#ifndef CommandBuilder_h
#define CommandBuilder_h

#include "Command.h"
#include <stdint.h>

enum class CommandType : uint8_t
{
    System = 0x00,
    Stream = 0x01,
    Rtc = 0x02,
    Notification = 0x07
};

enum class CmdSystemId : uint8_t
{
    Ready = 0x00,
    Reset = 0x01,
    AudioOutput = 0x06
};

enum class CmdStreamId : uint8_t
{
    Play = 0x00,
    Stop = 0x01,
    SearchFm = 0x02,
    SearchDab = 0x03,
    SearchStop = 0x04,
    Status = 0x05,
    Mode = 0x06,
    NowPlaying = 0x07,
    SignalStrength = 0x08,
    StereoModeSet = 0x09,
    StereoModeGet = 0x0A,
    StereoType = 0x0B,
    VolumeSet = 0x0C,
    VolumeGet = 0x0D,
    StationType = 0x0E,
    DabStationName = 0x0F,
    DabStationText = 0x10,
    SampleRate = 0x11,
    DabDataRate = 0x12,
    DabSignalQuality = 0x13,
    DabFrequency = 0x14,
    DabEnsembleName = 0x15,
    DabStationCount = 0x16,
    DabStationOnAir = 0x17,
    DabStationServiceName = 0x1A,
    DabFoundStationsCount = 0x1B,
    DabStationType = 0x1E,
    MemorySet = 0x21,
    MemoryGet = 0x22,
    DabStationInfo = 0x23,
    DabSortGet = 0x24,
    DabSortSet = 0x25,
    DabDrcGet = 0x26,
    DabDrcSet = 0x27,
    DabRemoveOffAir = 0x2B,
    DabExtendedCountryCode = 0x2D,
    FmRdsPiCode = 0x2E,
    FmStereoThresholdLevelSet = 0x30,
    FmStereoThresholdLevelGet = 0x31,
    FmRdsData = 0x32,
    FmSeekThresholdSet = 0x35,
    FmSeekThresholdGet = 0x36,
    FmStereoThresholdSet = 0x37,
    FmStereoThresholdGet = 0x38,
    FmExactStation = 0x39
};

enum class CmdRtcId : uint8_t
{
    Set = 0x00,
    Get = 0x01,
    Sync = 0x02,
    SyncStatus = 0x03,
    StatusClock = 0x04
};

enum class CmdNotificationId : uint8_t
{
    Notification = 0x00
};

class CommandBuilder
{
  public:
    CommandBuilder& createSystem(CmdSystemId const id);
    CommandBuilder& createStream(CmdStreamId const id);
    CommandBuilder& createRtc(CmdRtcId const id);
    CommandBuilder& createNotification(CmdNotificationId const id);

    CommandBuilder& append(uint8_t const value);
    CommandBuilder& append(uint16_t const value);
    CommandBuilder& append(uint32_t const value);

    Command& build();

  private:
    CommandBuilder& create(CommandType const type, uint8_t const id);
    Command command;
};

#endif // CommandBuilder_h
