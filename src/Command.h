#ifndef COMMAND_H
#define COMMAND_H

#include <inttypes.h>

#define COMMAND_MAX_SIZE 20

enum class CommandType : uint8_t
{
  System = 0x00,
  Stream = 0x01,
  Rtc = 0x02,
  Notification = 0x07
};

enum class CommandId : uint8_t
{
  SystemReady = 0x00,
  SystemReset = 0x01,
  SystemAudioOutput = 0x06,

  StreamPlay = 0x00,
  StreamStop = 0x01,
  StreamSearchFm = 0x02,
  StreamSearchDab = 0x03,
  StreamSearchStop = 0x04,
  StreamStatus = 0x05,
  StreamMode = 0x06,
  StreamNowPlaying = 0x07,
  StreamSignalStrength = 0x08,
  StreamStereoModeSet = 0x09,
  StreamStereoModeGet = 0x0A,
  StreamStereoType = 0x0B,
  StreamVolumeSet = 0x0C,
  StreamVolumeGet = 0x0D,
  StreamStationType = 0x0E,
  StreamDabStationName = 0x0F,
  StreamDabStationText = 0x10,
  StreamSampleRate = 0x11,
  StreamDabDataRate = 0x12,
  StreamDabSignalQuality = 0x13,
  StreamDabFrequency = 0x14,
  StreamDabEnsembleName = 0x15,
  StreamDabStationCount = 0x16,
  StreamDabStationOnAir = 0x17,
  StreamDabStationServiceName = 0x1A,
  StreamDabFoundStationsCount = 0x1B,
  StreamDabStationType = 0x1E,
  StreamMemorySet = 0x21,
  StreamMemoryGet = 0x22,
  StreamDabStationInfo = 0x23,
  StreamDabSortGet = 0x24,
  StreamDabSortSet = 0x25,
  StreamDabDrcGet = 0x26,
  StreamDabDrcSet = 0x27,
  StreamDabRemoveOffAir = 0x2B,
  StreamDabExtendedCountryCode = 0x2D,
  StreamFmRdsPiCode = 0x2E,
  StreamFmStereoThresholdLevelSet = 0x30,
  StreamFmStereoThresholdLevelGet = 0x31,
  StreamFmRdsData = 0x32,
  StreamFmSeekThresholdSet = 0x35,
  StreamFmSeekThresholdGet = 0x36,
  StreamFmStereoThresholdSet = 0x37,
  StreamFmStereoThresholdGet = 0x38,
  StreamFmExactStation = 0x39,

  RtcSet = 0x00,
  RtcGet = 0x01,
  RtcSync = 0x02,
  RtcSyncStatus = 0x03,
  RtcStatusClock = 0x04,

  Notification = 0x00
};

class Command
{
public:
  const uint8_t* GetData();
  uint8_t GetSize();
  void Start(CommandType type, CommandId id);
  bool Append(uint8_t value);
  bool Append(uint16_t value);
  bool Append(uint32_t value);
  bool End();

private:
  uint8_t data[COMMAND_MAX_SIZE];
  uint8_t size = 0;
};

#endif
