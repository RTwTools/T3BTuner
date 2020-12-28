#include "T3BTuner.h"
#include "Gpio.h"
#include "SystemAdapt.h"
#include <string.h>

static const uint16_t TunerSerialBaudrate = 57600U;
static const uint8_t HeaderSizeIndex = 5U;
static const uint8_t ResponseAck = 0x00;
static const uint8_t ResponseAckNack = 0x02;
static const uint8_t UnusedPin = 255;

T3BTuner::T3BTuner(ISerialStream* const serial, uint8_t const resetPin, uint8_t const mutePin) :
    T3BTuner(serial, resetPin, mutePin, UnusedPin)
{
}

T3BTuner::T3BTuner(ISerialStream* const serial, uint8_t const resetPin, uint8_t const mutePin,
                   uint8_t const spiCsPin) :
    serial(serial),
    pinReset(resetPin),
    pinMute(mutePin),
    pinSpiCs(spiCsPin)
{
}

void T3BTuner::Init()
{
    if (pinMute != UnusedPin)
    {
        GpioModeSet(pinMute, GpioMode::Output);
        GpioWrite(pinMute, GpioState::High);
    }

    if (pinSpiCs != UnusedPin)
    {
        GpioModeSet(pinSpiCs, GpioMode::Output);
        GpioWrite(pinSpiCs, GpioState::Low);
    }

    serial->begin(TunerSerialBaudrate);
    serial->setTimeout(50U);

    GpioModeSet(pinReset, GpioMode::Output);
    GpioWrite(pinReset, GpioState::Low);
    SystemDelay(100U);
    GpioWrite(pinReset, GpioState::High);
    SystemDelay(1000U);

    while (!Ready())
    {
        SystemDelay(500U);
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
    Command command = commandBuilder.CreateSystem(CmdSystemId::Ready).Build();
    return CommandSend(command);
}

/*
 *   Reset module.
 *   FullReset => Reset module database & module.
 */
bool T3BTuner::Reset(bool const fullReset)
{
    Command command =
        commandBuilder.CreateSystem(CmdSystemId::Reset).Append(static_cast<uint8_t>(fullReset)).Build();

    if (CommandSend(command))
    {
        Init();
        return true;
    }
    return false;
}

/*
 *   Set audio output channels (SPDIF, CINCH /I2S DAC/)
 *   CINCH for analog output, SPDIF for optical digital output
 */
bool T3BTuner::AudioOutput(bool const spdif, bool const cinch)
{
    uint8_t param = static_cast<uint8_t>(spdif) | (static_cast<uint8_t>(cinch << 0x1));
    Command command = commandBuilder.CreateSystem(CmdSystemId::AudioOutput).Append(param).Build();

    return CommandSend(command);
}

// *************************
// ***** STREAM ************
// *************************

/*
 *   Play DAB program
 *   programIndex = 1..9999999 (see programs index)
 */
bool T3BTuner::PlayDab(uint32_t const stationId)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::Play)
                          .Append(static_cast<uint8_t>(TunerMode::Dab))
                          .Append(stationId)
                          .Build();

    return CommandSend(command);
}

/*
 *   Play FM program
 *   frequency = 87500..108000 (MHz)
 */
bool T3BTuner::PlayFm(uint32_t const frequency)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::Play)
                          .Append(static_cast<uint8_t>(TunerMode::Fm))
                          .Append(frequency)
                          .Build();

    return CommandSend(command);
}

/*
 *   Play Beep.
 */
bool T3BTuner::PlayBeep()
{
    Command command = commandBuilder.CreateStream(CmdStreamId::Play)
                          .Append(static_cast<uint8_t>(TunerMode::Beep))
                          .Append(static_cast<uint32_t>(0x00))
                          .Build();

    return CommandSend(command);
}

/*
 *   Stop.
 */
bool T3BTuner::Stop()
{
    Command command = commandBuilder.CreateStream(CmdStreamId::Stop).Build();
    return CommandSend(command);
}

/*
 * Seek FM program.
 */
bool T3BTuner::FmSearch(bool const searchForward)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::SearchFm)
                          .Append(static_cast<uint8_t>(searchForward))
                          .Build();

    return CommandSend(command);
}

/*
 * Search DAB bands for programs.
 */
bool T3BTuner::DabSearch(DabBand const band)
{
    commandBuilder.CreateStream(CmdStreamId::SearchDab);

    switch (band)
    {
        case DabBand::BandIII:
            commandBuilder.Append(static_cast<uint8_t>(0U));
            commandBuilder.Append(static_cast<uint8_t>(40U));
            break;
        case DabBand::ChinaBand:
            commandBuilder.Append(static_cast<uint8_t>(41U));
            commandBuilder.Append(static_cast<uint8_t>(71U));
            break;
        case DabBand::LBand:
            commandBuilder.Append(static_cast<uint8_t>(72U));
            commandBuilder.Append(static_cast<uint8_t>(94U));
            break;
    }

    Command command = commandBuilder.Build();
    return CommandSend(command);
}

/*
 *   Radio module play status.
 */
bool T3BTuner::State(TunerState* const status)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::Status).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(status)));
}

/*
 *   Radio module play mode.
 */
bool T3BTuner::Mode(TunerMode* const mode)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::Mode).Build();
    return (CommandSend(command) && ResponseUint8(0, reinterpret_cast<uint8_t*>(mode)));
}

/*
 * Get DAB stationId, get FM frequency.
 */
bool T3BTuner::NowPlaying(uint32_t* const programId)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::NowPlaying).Build();
    return (CommandSend(command) && ResponseUint32(0U, programId));
}

/*
 * Get signal strength
 * DAB: signalStrength=0..18, bitErrorRate=
 * FM: signalStrength=0..100
 */
bool T3BTuner::SignalStrength(uint8_t* const signalStrength, uint16_t* const bitErrorRate)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::SignalStrength).Build();
    return (CommandSend(command) && ResponseUint8(0U, signalStrength) && ResponseUint16(1U, bitErrorRate));
}

/*
 *   Set stereo mode.
 */
bool T3BTuner::StereoModeSet(StereoMode const stereoMode)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::StereoModeSet)
                          .Append(static_cast<uint8_t>(stereoMode))
                          .Build();

    return CommandSend(command);
}

/*
 *   Get stereo mode.
 */
bool T3BTuner::StereoModeGet(StereoMode* const stereoMode)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::StereoModeGet).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(stereoMode)));
}

/*
 *   Get stereo type
 */
bool T3BTuner::StereoTypeGet(StereoType* const stereotype)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::StereoType).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(stereotype)));
}

/*
 *   Set volume.
 *   volumeLevel = 0..16
 */
bool T3BTuner::VolumeSet(uint8_t const volume)
{
    uint8_t volumeValue = (volume > 16U) ? 16U : volume;
    Command command = commandBuilder.CreateStream(CmdStreamId::VolumeSet).Append(volumeValue).Build();

    return CommandSend(command);
}

/*
 *   Get volume.
 *   return set volumeLevel: 0..16
 */
bool T3BTuner::VolumeGet(uint8_t* const volume)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::VolumeGet).Build();
    return (CommandSend(command) && ResponseUint8(0U, volume));
}

/*
 *   Get program type.
 */
bool T3BTuner::StationTypeGet(StationType* const programType)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::StationType).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(programType)));
}

/*
 * Get DAB station name.
 */
bool T3BTuner::DabStationName(uint32_t const stationId, char* const buffer, uint16_t const size,
                              bool const longName)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabStationName)
                          .Append(stationId)
                          .Append(static_cast<uint8_t>(longName))
                          .Build();

    return (CommandSend(command) && ResponseText(buffer, size));
}

/*
 * Get DAB text event
 * return: 1=new text, 2=text is same, 3=no text
 * dabText: text
 */
bool T3BTuner::DabStationText(char* const buffer, uint16_t const size)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabStationText).Build();
    if (CommandSend(command))
    {
        if (responseSize == 1)
        {
            // No text received
            // Response[0] value => 0 = No text available, 1 = Station text is empty.
            return false;
        }

        ResponseText(buffer, size);
        bool changed = (strncmp(buffer, dabStationText, sizeof(dabStationText)) != 0);
        strncpy(dabStationText, buffer, sizeof(dabStationText));
        return changed;
    }

    return false;
}

/*
 *   Get sampling rate (DAB/FM).
 */
bool T3BTuner::SampleRateGet(SampleRate* const sampleRate)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::SampleRate).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(sampleRate)));
}

/*
 *   Get data rate (DAB)
 *   return data: data rate in kbps
 */
bool T3BTuner::DabDataRate(uint16_t* const dataRate)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabDataRate).Build();
    return (CommandSend(command) && ResponseUint16(0U, dataRate));
}

/*
 *   Get DAB signal quality
 *   return: 0..100
 *   0..19 = playback stop
 *   20..30 = the noise (short break) appears
 *   100 = the bit error rate is 0
 */
bool T3BTuner::DabSignalQuality(uint8_t* const signalQuality)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabSignalQuality).Build();
    return (CommandSend(command) && ResponseUint8(0U, signalQuality));
}

/*
 *   Get DAB frequency for program index
 *   return: frequency index
 *   0=174.928MHz, 1=176.64, 2=178.352,...
 *
 *  // TODO: add conversion table for index2freqency
 */
bool T3BTuner::DabStationFrequency(uint32_t const stationId, uint8_t* const frequency)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabFrequency).Append(stationId).Build();

    return (CommandSend(command) && ResponseUint8(0U, frequency));
}

/*
 * Get DAB program ensemble name.
 */
bool T3BTuner::DabStationEnsembleName(uint32_t const stationId, char* const buffer, uint16_t const size)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabEnsembleName)
                          .Append(stationId)
                          .Append(static_cast<uint8_t>(false))
                          .Build();

    return (CommandSend(command) && ResponseText(buffer, size));
}

/*
 * Number of DAB stations in database.
 */
bool T3BTuner::DabStationCount(uint32_t* const count)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabStationCount).Build();
    return (CommandSend(command) && ResponseUint32(0U, count));
}

/*
 *   Test DAB program is active (on-air)
 *   return: 0=off-air, 1=on-air
 */
bool T3BTuner::DabStationOnAir(uint32_t const stationId, bool* const onAir)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabStationOnAir).Append(stationId).Build();

    bool result = CommandSend(command);
    *onAir = static_cast<bool>(response[0U]);
    return result;
}

/*
 * Get DAB program service short name.
 */
bool T3BTuner::DabStationServiceName(uint32_t const stationId, char* const buffer, uint16_t const size)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabStationServiceName)
                          .Append(stationId)
                          .Append(static_cast<uint8_t>(false))
                          .Build();

    return (CommandSend(command) && ResponseText(buffer, size));
}

/*
 * Number of programs found in search process.
 */
bool T3BTuner::DabFoundStationsCount(uint8_t* const count)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabFoundStationsCount).Build();
    return (CommandSend(command) && ResponseUint8(0U, count));
}

/*
 * Get DAB program service component type (ASCTy)
 */
bool T3BTuner::DabStationType(uint32_t const stationId, DabStreamType* const type)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabStationType).Append(stationId).Build();

    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(type)));
}

/*
 *   Set preset
 */
bool T3BTuner::MemorySet(MemoryType const mode, MemoryId const id, uint32_t const programId)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::MemorySet)
                          .Append(static_cast<uint8_t>(mode))
                          .Append(static_cast<uint8_t>(id))
                          .Append(programId)
                          .Build();

    return CommandSend(command);
}

/*
 *  Get preset
 */
bool T3BTuner::MemoryGet(MemoryType const mode, MemoryId const id, uint32_t* const programId)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::MemoryGet)
                          .Append(static_cast<uint8_t>(mode))
                          .Append(static_cast<uint8_t>(id))
                          .Build();

    return (CommandSend(command) && ResponseUint32(0U, programId));
}

/*
 * Get station info
 * return serviceId = service id of DAB program
 * return ensembleId = ensemble id of DAB program
 */
bool T3BTuner::DabStationInfo(uint32_t const stationId, uint32_t* const serviceId, uint16_t* const ensembleId)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabStationInfo).Append(stationId).Build();

    return (CommandSend(command) && ResponseUint32(0U, serviceId) && ResponseUint16(4U, ensembleId));
}

/*
 *   Get DAB station sort order.
 */
bool T3BTuner::DabSortGet(DabSortOrder* const sortOrder)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabSortGet).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(sortOrder)));
}

/*
 *   Set DAB station sort order.
 */
bool T3BTuner::DabSortSet(DabSortOrder const sortOrder)
{
    Command command =
        commandBuilder.CreateStream(CmdStreamId::DabSortSet).Append(static_cast<uint8_t>(sortOrder)).Build();

    return CommandSend(command);
}

/*
 *   Get DAB DRC.
 */
bool T3BTuner::DabDrcGet(DabDrc* const drc)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabDrcGet).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(drc)));
}

/*
 *   Set DAB DRC.
 */
bool T3BTuner::DabDrcSet(DabDrc const drc)
{
    Command command =
        commandBuilder.CreateStream(CmdStreamId::DabDrcSet).Append(static_cast<uint8_t>(drc)).Build();

    return CommandSend(command);
}

/*
 *   Prune programs - delete inactive programs (!on-air)
 */
bool T3BTuner::DabRemoveOffAir(uint16_t* const removedTotal, uint16_t* const removedIndex)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabRemoveOffAir).Build();
    return (CommandSend(command) && ResponseUint16(0U, removedTotal) && ResponseUint16(2U, removedIndex));
}

/*
 * Get ECC
 * return ECC (Extended Country Code)
 * return countryId (Country identification)
 */
bool T3BTuner::DabExtendedCountryCode(uint8_t* const ecc, uint8_t* const countryId)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::DabExtendedCountryCode).Build();
    return (CommandSend(command) && ResponseUint8(0U, ecc) && ResponseUint8(1U, countryId));
}

/*
 *   Get FM RDS PI code
 */
bool T3BTuner::FmRdsPiCode(uint16_t* const code)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::FmRdsPiCode).Build();
    return (CommandSend(command) && ResponseUint16(0U, code));
}

/*
 *   Set FMstereoThdLevel
 *   RSSIthresholdLevel = 0..10
 */
bool T3BTuner::FmStereoThresholdLevelSet(uint8_t const level)
{
    Command command =
        commandBuilder.CreateStream(CmdStreamId::FmStereoThresholdLevelSet).Append(level).Build();

    return CommandSend(command);
}

/*
 *   Get FMstereoThdLevel
 *   data return = 0..10
 */
bool T3BTuner::FmStereoThresholdLevelGet(uint8_t* const level)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::FmStereoThresholdLevelGet).Build();
    return (CommandSend(command) && ResponseUint8(0U, level));
}

/*
 *   Get RDS raw data
 *   return: 1=new RDS data, 2=no new RDS data, 3=no RDS data
 */
bool T3BTuner::FmRdsRawData(uint16_t* const blockA, uint16_t* const blockB, uint16_t* const blockC,
                            uint16_t* const blockD, uint16_t* const blerA, uint16_t* const blerB,
                            uint16_t* const blerC, uint16_t* const blerD)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::FmRdsData).Build();
    if (CommandSend(command) && responseSize > 1U)
    {
        return (ResponseUint16(0, blockA) && ResponseUint16(2, blockB) && ResponseUint16(4U, blockC) &&
                ResponseUint16(6U, blockD) && ResponseUint16(8U, blerA) && ResponseUint16(10U, blerB) &&
                ResponseUint16(12U, blerC) && ResponseUint16(14U, blerD));
    }
    return false;
}

/*
 *   Set FMseekThreshold
 *   RSSIthreshold = 0..100
 */
bool T3BTuner::FmSeekThresholdSet(uint8_t const threshold)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::FmSeekThresholdSet).Append(threshold).Build();

    return CommandSend(command);
}

/*
 *   Get FMseekThreshold
 *   data return = 0..100
 */
bool T3BTuner::FmSeekThresholdGet(uint8_t* const threshold)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::FmSeekThresholdGet).Build();
    return (CommandSend(command) && ResponseUint8(0U, threshold));
}

/*
 *   Set FMstereoThreshold
 *   RSSIthreshold = 0..100
 */
bool T3BTuner::FmStereoThresholdSet(uint8_t const threshold)
{
    Command command =
        commandBuilder.CreateStream(CmdStreamId::FmStereoThresholdSet).Append(threshold).Build();

    return CommandSend(command);
}

/*
 *   Get FMstereoThreshold
 *   data return = 0..100
 */
bool T3BTuner::FmStereoThresholdGet(uint8_t* const threshold)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::FmStereoThresholdGet).Build();
    return (CommandSend(command) && ResponseUint8(0U, threshold));
}

/*
 *   Get FM Exact station.
 */
bool T3BTuner::FmExactStationGet(FmExactStation* const exact)
{
    Command command = commandBuilder.CreateStream(CmdStreamId::FmExactStation).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(exact)));
}

// *************************
// ***** RTC ***************
// *************************

/*
 *  Set RTC clock
 *  year: 2017=17,2018=18, month: 1..12, day: 1..31, hour: 0..23, minute: 0..59, second: 0..59
 */
bool T3BTuner::ClockSet(uint8_t const year, uint8_t const month, uint8_t const day, uint8_t const hour,
                        uint8_t const minute, uint8_t const second)
{
    Command command = commandBuilder.CreateRtc(CmdRtcId::Set)
                          .Append(second)
                          .Append(minute)
                          .Append(hour)
                          .Append(day)
                          .Append(static_cast<uint8_t>(0x00))
                          .Append(month)
                          .Append(year)
                          .Build();

    return CommandSend(command);
}

/*
 *  Get RTC ckock
 *  year: 2017=17,2018=18, month: 1..12, day: 1..31, hour: 0..23, minute: 0..59, second: 0..59
 */
bool T3BTuner::ClockGet(uint8_t* const year, uint8_t* const month, uint8_t* const day, uint8_t* const hour,
                        uint8_t* const minute, uint8_t* const second)
{
    Command command = commandBuilder.CreateRtc(CmdRtcId::Get).Build();
    bool result = CommandSend(command);
    result &= ResponseUint8(0U, second) & ResponseUint8(1U, minute);
    result &= ResponseUint8(2U, hour) & ResponseUint8(3U, day);
    result &= ResponseUint8(5U, month) & ResponseUint8(6U, year);
    return result;
}

/*
 *  Set RTC sync clock from stream enable
 */
bool T3BTuner::ClockSyncSet(bool const enable)
{
    Command command = commandBuilder.CreateRtc(CmdRtcId::Sync).Append(static_cast<uint8_t>(enable)).Build();

    return CommandSend(command);
}

/*
 *  Get RTC sync clock status
 */
bool T3BTuner::ClockSyncGet(bool* const enabled)
{
    Command command = commandBuilder.CreateRtc(CmdRtcId::SyncStatus).Build();
    bool result = CommandSend(command);
    *enabled = response[0U];
    return result;
}

/*
 *  Get RTC clock status
 */
bool T3BTuner::ClockStatusGet(ClockStatus* const status)
{
    Command command = commandBuilder.CreateRtc(CmdRtcId::StatusClock).Build();
    return (CommandSend(command) && ResponseUint8(0U, reinterpret_cast<uint8_t*>(status)));
}

// *************************
// ***** EVENTS ************
// *************************

/*
 *   Enabled / Disable event notifications.
 */
bool T3BTuner::EventEnable(bool const enable)
{
    uint16_t value = (enable) ? 0x7F : 0x00;
    Command command =
        commandBuilder.CreateNotification(CmdNotificationId::Notification).Append(value).Build();

    return CommandSend(command);
}

bool T3BTuner::EventReceived()
{
    return (bool)serial->available();
}

/*
 *   Read event
 */
bool T3BTuner::EventRead(EventType* const type)
{
    bool result = (ResponseReceive() && ((CommandType)responseHeader[1U] == CommandType::Notification));
    *type = (EventType)responseHeader[2U];
    return result;
}

// *************************
// ***** PRIVATE FUNCTIONS *
// *************************

/*
 *  Send command to DAB module and wait for answer
 */
bool T3BTuner::CommandSend(Command const& command)
{
    while (serial->available())
    {
        serial->read();
    }
    serial->write(command.data, command.size);
    serial->flush();
    return (ResponseReceive() &&
            !(responseHeader[1U] == ResponseAck && responseHeader[2U] == ResponseAckNack));
}

bool T3BTuner::ResponseReceive()
{
    uint16_t index = 0U;
    uint8_t data = 0U;
    uint32_t endMillis = SystemMillis() + 200U; // timeout for answer from module = 200ms
    responseSize = 0U;

    while (SystemMillis() < endMillis && index < T3BTunerMaxDataSize)
    {
        if (serial->available())
        {
            data = serial->read();
            if (data == CommandStartValue)
            {
                index = 0U;
            }

            if (index < T3BTunerHeaderSize)
            {
                responseHeader[index] = data;

                if (index == HeaderSizeIndex)
                {
                    responseSize = static_cast<uint16_t>(responseHeader[5U]);
                    responseSize |= static_cast<uint16_t>(responseHeader[4U]) << 8U;
                }
            }
            else if ((index - T3BTunerHeaderSize) < responseSize)
            {
                response[index - T3BTunerHeaderSize] = data;
            }

            if (data == CommandEndValue)
            {
                if ((index - T3BTunerHeaderSize - responseSize) == 0U)
                {
                    return true;
                }
            }
            index++;
        }
    }
    return false;
}

bool T3BTuner::ResponseText(char* const buffer, uint16_t const size)
{
    uint16_t j = 0U;
    for (uint16_t i = 0U; i < responseSize; i = i + 2U)
    {
        if (size <= j)
            return false;
        buffer[j++] = Uint16ToChar(response[i], response[i + 1U]);
    }
    return true;
}

bool T3BTuner::ResponseUint8(uint8_t const index, uint8_t* const resp)
{
    if (responseSize > index)
    {
        *resp = response[index];
        return true;
    }
    return false;
}

bool T3BTuner::ResponseUint16(uint8_t const index, uint16_t* const resp)
{
    if (responseSize > (index + 1U))
    {
        *resp = static_cast<uint16_t>(response[index + 1U]);
        *resp |= static_cast<uint16_t>(response[index]) << 8U;
        return true;
    }
    return false;
}

bool T3BTuner::ResponseUint32(uint8_t const index, uint32_t* const resp)
{
    if (responseSize > (index + 3U))
    {
        *resp = static_cast<uint32_t>(response[index + 3U]);
        *resp |= static_cast<uint32_t>(response[index + 2U]) << 8U;
        *resp |= static_cast<uint32_t>(response[index + 1U]) << 16U;
        *resp |= static_cast<uint32_t>(response[index + 0U]) << 24U;
        return true;
    }
    return false;
}

/*
 * Convert uint16_t (2 * uint8_t) from Tuner to a char.
 */
char T3BTuner::Uint16ToChar(uint8_t const byte1, uint8_t const byte0)
{
    if (byte1 == 0x00)
    {
        if (byte0 < 128U)
        {
            return byte0;
        }

        switch (byte0)
        {
            case 0x8A:
                return 'S';
            case 0x8C:
                return 'S';
            case 0x8D:
                return 'T';
            case 0x8E:
                return 'Z';
            case 0x8F:
                return 'Z';
            case 0x9A:
                return 's';
            case 0x9D:
                return 't';
            case 0x9E:
                return 'z';
            case 0xC0:
                return 'A';
            case 0xC1:
                return 'A';
            case 0xC2:
                return 'A';
            case 0xC3:
                return 'A';
            case 0xC4:
                return 'A';
            case 0xC5:
                return 'A';
            case 0xC7:
                return 'C';
            case 0xC8:
                return 'E';
            case 0xC9:
                return 'E';
            case 0xCA:
                return 'E';
            case 0xCB:
                return 'E';
            case 0xCC:
                return 'I';
            case 0xCD:
                return 'I';
            case 0xCE:
                return 'I';
            case 0xCF:
                return 'I';
            case 0xD0:
                return 'D';
            case 0xD1:
                return 'N';
            case 0xD2:
                return 'O';
            case 0xD3:
                return 'O';
            case 0xD4:
                return 'O';
            case 0xD5:
                return 'O';
            case 0xD6:
                return 'O';
            case 0xD8:
                return 'O';
            case 0xD9:
                return 'U';
            case 0xDA:
                return 'U';
            case 0xDB:
                return 'U';
            case 0xDC:
                return 'U';
            case 0xDD:
                return 'Y';
            case 0xE0:
                return 'a';
            case 0xE1:
                return 'a';
            case 0xE2:
                return 'a';
            case 0xE3:
                return 'a';
            case 0xE4:
                return 'a';
            case 0xE5:
                return 'a';
            case 0xE7:
                return 'c';
            case 0xE8:
                return 'e';
            case 0xE9:
                return 'e';
            case 0xEA:
                return 'e';
            case 0xEB:
                return 'e';
            case 0xEC:
                return 'i';
            case 0xED:
                return 'i';
            case 0xEE:
                return 'i';
            case 0xEF:
                return 'i';
            case 0xF1:
                return 'n';
            case 0xF2:
                return 'o';
            case 0xF3:
                return 'o';
            case 0xF4:
                return 'o';
            case 0xF5:
                return 'o';
            case 0xF6:
                return 'o';
            case 0xF9:
                return 'u';
            case 0xFA:
                return 'u';
            case 0xFB:
                return 'u';
            case 0xFC:
                return 'u';
            case 0xFD:
                return 'y';
            case 0xFF:
                return 'y';
        }
    }
    else if (byte1 == 0x01)
    {
        switch (byte0)
        {
            case 0x1B:
                return 'e'; // ě
            case 0x48:
                return 'n'; // ň
            case 0x59:
                return 'r'; // ř
            case 0x0D:
                return 'c'; // č
            case 0x7E:
                return 'z'; // ž
            case 0x0C:
                return 'C'; // Č
        }
    }

    return 0x00;
}
