#include "T3BTuner.h"
#include "Gpio.h"
#include "SystemAdapt.h"
#include <string.h>

static const uint16_t TunerSerialBaudrate = 57600U;
static const uint8_t HeaderSizeIndex = 5U;
static const uint8_t ResponseAck = 0x00;
static const uint8_t ResponseAckNack = 0x02;
static const uint8_t UnusedPin = 255;

T3BTuner::T3BTuner(ISerialStream& stream, uint8_t const resetPin, uint8_t const mutePin) :
    T3BTuner(stream, resetPin, mutePin, UnusedPin)
{
}

T3BTuner::T3BTuner(ISerialStream& stream, uint8_t const resetPin, uint8_t const mutePin,
                   uint8_t const spiCsPin) :
    stream(stream),
    pinReset(resetPin),
    pinMute(mutePin),
    pinSpiCs(spiCsPin)
{
}

void T3BTuner::init()
{
    if (pinMute != UnusedPin)
    {
        gpioModeSet(pinMute, GpioMode::Output);
        gpioWrite(pinMute, GpioState::High);
    }

    if (pinSpiCs != UnusedPin)
    {
        gpioModeSet(pinSpiCs, GpioMode::Output);
        gpioWrite(pinSpiCs, GpioState::Low);
    }

    stream.begin(TunerSerialBaudrate);
    stream.setTimeout(50U);

    gpioModeSet(pinReset, GpioMode::Output);
    gpioWrite(pinReset, GpioState::Low);
    systemDelay(100U);
    gpioWrite(pinReset, GpioState::High);
    systemDelay(1000U);

    while (!ready())
    {
        systemDelay(500U);
    }
}

// *************************
// ***** SYSTEM ************
// *************************

/*
 *   Test for DAB module is ready for communication
 */
bool T3BTuner::ready()
{
    Command command = commandBuilder.createSystem(CmdSystemId::Ready).build();
    return commandSend(command);
}

/*
 *   Reset module.
 *   FullReset => Reset module database & module.
 */
bool T3BTuner::reset(bool const fullReset)
{
    Command command =
        commandBuilder.createSystem(CmdSystemId::Reset).append(static_cast<uint8_t>(fullReset)).build();

    if (commandSend(command))
    {
        init();
        return true;
    }
    return false;
}

/*
 *   Set audio output channels (SPDIF, CINCH /I2S DAC/)
 *   CINCH for analog output, SPDIF for optical digital output
 */
bool T3BTuner::audioOutput(bool const spdif, bool const cinch)
{
    uint8_t param = static_cast<uint8_t>(spdif) | (static_cast<uint8_t>(cinch << 0x1));
    Command command = commandBuilder.createSystem(CmdSystemId::AudioOutput).append(param).build();

    return commandSend(command);
}

// *************************
// ***** STREAM ************
// *************************

/*
 *   Play DAB program
 *   programIndex = 1..9999999 (see programs index)
 */
bool T3BTuner::playDab(uint32_t const stationId)
{
    Command command = commandBuilder.createStream(CmdStreamId::Play)
                          .append(static_cast<uint8_t>(TunerMode::Dab))
                          .append(stationId)
                          .build();

    return commandSend(command);
}

/*
 *   Play FM program
 *   frequency = 87500..108000 (MHz)
 */
bool T3BTuner::playFm(uint32_t const frequency)
{
    Command command = commandBuilder.createStream(CmdStreamId::Play)
                          .append(static_cast<uint8_t>(TunerMode::Fm))
                          .append(frequency)
                          .build();

    return commandSend(command);
}

/*
 *   Play Beep.
 */
bool T3BTuner::playBeep()
{
    Command command = commandBuilder.createStream(CmdStreamId::Play)
                          .append(static_cast<uint8_t>(TunerMode::Beep))
                          .append(static_cast<uint32_t>(0x00))
                          .build();

    return commandSend(command);
}

/*
 *   Stop.
 */
bool T3BTuner::stop()
{
    Command command = commandBuilder.createStream(CmdStreamId::Stop).build();
    return commandSend(command);
}

/*
 * Seek FM program.
 */
bool T3BTuner::fmSearch(bool const searchForward)
{
    Command command = commandBuilder.createStream(CmdStreamId::SearchFm)
                          .append(static_cast<uint8_t>(searchForward))
                          .build();

    return commandSend(command);
}

/*
 * Search DAB bands for programs.
 */
bool T3BTuner::dabSearch(DabBand const band)
{
    commandBuilder.createStream(CmdStreamId::SearchDab);

    switch (band)
    {
        case DabBand::BandIII:
            commandBuilder.append(static_cast<uint8_t>(0U));
            commandBuilder.append(static_cast<uint8_t>(40U));
            break;
        case DabBand::ChinaBand:
            commandBuilder.append(static_cast<uint8_t>(41U));
            commandBuilder.append(static_cast<uint8_t>(71U));
            break;
        case DabBand::LBand:
            commandBuilder.append(static_cast<uint8_t>(72U));
            commandBuilder.append(static_cast<uint8_t>(94U));
            break;
    }

    Command command = commandBuilder.build();
    return commandSend(command);
}

/*
 *   Radio module play status.
 */
bool T3BTuner::state(TunerState* const status)
{
    Command command = commandBuilder.createStream(CmdStreamId::Status).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(status)));
}

/*
 *   Radio module play mode.
 */
bool T3BTuner::mode(TunerMode* const mode)
{
    Command command = commandBuilder.createStream(CmdStreamId::Mode).build();
    return (commandSend(command) && responseUint8(0, reinterpret_cast<uint8_t*>(mode)));
}

/*
 * Get DAB stationId, get FM frequency.
 */
bool T3BTuner::nowPlaying(uint32_t* const programId)
{
    Command command = commandBuilder.createStream(CmdStreamId::NowPlaying).build();
    return (commandSend(command) && responseUint32(0U, programId));
}

/*
 * Get signal strength
 * DAB: signalStrength=0..18, bitErrorRate=
 * FM: signalStrength=0..100
 */
bool T3BTuner::signalStrength(uint8_t* const signalStrength, uint16_t* const bitErrorRate)
{
    Command command = commandBuilder.createStream(CmdStreamId::SignalStrength).build();
    return (commandSend(command) && responseUint8(0U, signalStrength) && responseUint16(1U, bitErrorRate));
}

/*
 *   Set stereo mode.
 */
bool T3BTuner::stereoModeSet(StereoMode const stereoMode)
{
    Command command = commandBuilder.createStream(CmdStreamId::StereoModeSet)
                          .append(static_cast<uint8_t>(stereoMode))
                          .build();

    return commandSend(command);
}

/*
 *   Get stereo mode.
 */
bool T3BTuner::stereoModeGet(StereoMode* const stereoMode)
{
    Command command = commandBuilder.createStream(CmdStreamId::StereoModeGet).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(stereoMode)));
}

/*
 *   Get stereo type
 */
bool T3BTuner::stereoTypeGet(StereoType* const stereotype)
{
    Command command = commandBuilder.createStream(CmdStreamId::StereoType).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(stereotype)));
}

/*
 *   Set volume.
 *   volumeLevel = 0..16
 */
bool T3BTuner::volumeSet(uint8_t const volume)
{
    uint8_t volumeValue = (volume > 16U) ? 16U : volume;
    Command command = commandBuilder.createStream(CmdStreamId::VolumeSet).append(volumeValue).build();

    return commandSend(command);
}

/*
 *   Get volume.
 *   return set volumeLevel: 0..16
 */
bool T3BTuner::volumeGet(uint8_t* const volume)
{
    Command command = commandBuilder.createStream(CmdStreamId::VolumeGet).build();
    return (commandSend(command) && responseUint8(0U, volume));
}

/*
 *   Get program type.
 */
bool T3BTuner::stationTypeGet(StationType* const programType)
{
    Command command = commandBuilder.createStream(CmdStreamId::StationType).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(programType)));
}

/*
 * Get DAB station name.
 */
bool T3BTuner::dabStationName(uint32_t const stationId, char* const buffer, uint16_t const size,
                              bool const longName)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabStationName)
                          .append(stationId)
                          .append(static_cast<uint8_t>(longName))
                          .build();

    return (commandSend(command) && responseText(buffer, size));
}

/*
 * Get DAB text event
 * return: 1=new text, 2=text is same, 3=no text
 * dabText: text
 */
bool T3BTuner::dabStationText(char* const buffer, uint16_t const size)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabStationText).build();
    if (commandSend(command))
    {
        if (responseSize == 1)
        {
            // No text received
            // Response[0] value => 0 = No text available, 1 = Station text is empty.
            return false;
        }

        responseText(buffer, size);
        bool changed = (strncmp(buffer, stationText, sizeof(stationText)) != 0);
        strncpy(stationText, buffer, sizeof(stationText));
        return changed;
    }

    return false;
}

/*
 *   Get sampling rate (DAB/FM).
 */
bool T3BTuner::sampleRateGet(SampleRate* const sampleRate)
{
    Command command = commandBuilder.createStream(CmdStreamId::SampleRate).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(sampleRate)));
}

/*
 *   Get data rate (DAB)
 *   return data: data rate in kbps
 */
bool T3BTuner::dabDataRate(uint16_t* const dataRate)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabDataRate).build();
    return (commandSend(command) && responseUint16(0U, dataRate));
}

/*
 *   Get DAB signal quality
 *   return: 0..100
 *   0..19 = playback stop
 *   20..30 = the noise (short break) appears
 *   100 = the bit error rate is 0
 */
bool T3BTuner::dabSignalQuality(uint8_t* const signalQuality)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabSignalQuality).build();
    return (commandSend(command) && responseUint8(0U, signalQuality));
}

/*
 *   Get DAB frequency for program index
 *   return: frequency index
 *   0=174.928MHz, 1=176.64, 2=178.352,...
 *
 *  // TODO: add conversion table for index2freqency
 */
bool T3BTuner::dabStationFrequency(uint32_t const stationId, uint8_t* const frequency)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabFrequency).append(stationId).build();

    return (commandSend(command) && responseUint8(0U, frequency));
}

/*
 * Get DAB program ensemble name.
 */
bool T3BTuner::dabStationEnsembleName(uint32_t const stationId, char* const buffer, uint16_t const size)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabEnsembleName)
                          .append(stationId)
                          .append(static_cast<uint8_t>(false))
                          .build();

    return (commandSend(command) && responseText(buffer, size));
}

/*
 * Number of DAB stations in database.
 */
bool T3BTuner::dabStationCount(uint32_t* const count)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabStationCount).build();
    return (commandSend(command) && responseUint32(0U, count));
}

/*
 *   Test DAB program is active (on-air)
 *   return: 0=off-air, 1=on-air
 */
bool T3BTuner::dabStationOnAir(uint32_t const stationId, bool* const onAir)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabStationOnAir).append(stationId).build();

    bool result = commandSend(command);
    *onAir = static_cast<bool>(response[0U]);
    return result;
}

/*
 * Get DAB program service short name.
 */
bool T3BTuner::dabStationServiceName(uint32_t const stationId, char* const buffer, uint16_t const size)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabStationServiceName)
                          .append(stationId)
                          .append(static_cast<uint8_t>(false))
                          .build();

    return (commandSend(command) && responseText(buffer, size));
}

/*
 * Number of programs found in search process.
 */
bool T3BTuner::dabFoundStationsCount(uint8_t* const count)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabFoundStationsCount).build();
    return (commandSend(command) && responseUint8(0U, count));
}

/*
 * Get DAB program service component type (ASCTy)
 */
bool T3BTuner::dabStationType(uint32_t const stationId, DabStreamType* const type)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabStationType).append(stationId).build();

    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(type)));
}

/*
 *   Set preset
 */
bool T3BTuner::memorySet(MemoryType const mode, MemoryId const id, uint32_t const programId)
{
    Command command = commandBuilder.createStream(CmdStreamId::MemorySet)
                          .append(static_cast<uint8_t>(mode))
                          .append(static_cast<uint8_t>(id))
                          .append(programId)
                          .build();

    return commandSend(command);
}

/*
 *  Get preset
 */
bool T3BTuner::memoryGet(MemoryType const mode, MemoryId const id, uint32_t* const programId)
{
    Command command = commandBuilder.createStream(CmdStreamId::MemoryGet)
                          .append(static_cast<uint8_t>(mode))
                          .append(static_cast<uint8_t>(id))
                          .build();

    return (commandSend(command) && responseUint32(0U, programId));
}

/*
 * Get station info
 * return serviceId = service id of DAB program
 * return ensembleId = ensemble id of DAB program
 */
bool T3BTuner::dabStationInfo(uint32_t const stationId, uint32_t* const serviceId, uint16_t* const ensembleId)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabStationInfo).append(stationId).build();

    return (commandSend(command) && responseUint32(0U, serviceId) && responseUint16(4U, ensembleId));
}

/*
 *   Get DAB station sort order.
 */
bool T3BTuner::dabSortGet(DabSortOrder* const sortOrder)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabSortGet).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(sortOrder)));
}

/*
 *   Set DAB station sort order.
 */
bool T3BTuner::dabSortSet(DabSortOrder const sortOrder)
{
    Command command =
        commandBuilder.createStream(CmdStreamId::DabSortSet).append(static_cast<uint8_t>(sortOrder)).build();

    return commandSend(command);
}

/*
 *   Get DAB DRC.
 */
bool T3BTuner::dabDrcGet(DabDrc* const drc)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabDrcGet).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(drc)));
}

/*
 *   Set DAB DRC.
 */
bool T3BTuner::dabDrcSet(DabDrc const drc)
{
    Command command =
        commandBuilder.createStream(CmdStreamId::DabDrcSet).append(static_cast<uint8_t>(drc)).build();

    return commandSend(command);
}

/*
 *   Prune programs - delete inactive programs (!on-air)
 */
bool T3BTuner::dabRemoveOffAir(uint16_t* const removedTotal, uint16_t* const removedIndex)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabRemoveOffAir).build();
    return (commandSend(command) && responseUint16(0U, removedTotal) && responseUint16(2U, removedIndex));
}

/*
 * Get ECC
 * return ECC (Extended Country Code)
 * return countryId (Country identification)
 */
bool T3BTuner::dabExtendedCountryCode(uint8_t* const ecc, uint8_t* const countryId)
{
    Command command = commandBuilder.createStream(CmdStreamId::DabExtendedCountryCode).build();
    return (commandSend(command) && responseUint8(0U, ecc) && responseUint8(1U, countryId));
}

/*
 *   Get FM RDS PI code
 */
bool T3BTuner::fmRdsPiCode(uint16_t* const code)
{
    Command command = commandBuilder.createStream(CmdStreamId::FmRdsPiCode).build();
    return (commandSend(command) && responseUint16(0U, code));
}

/*
 *   Set FMstereoThdLevel
 *   RSSIthresholdLevel = 0..10
 */
bool T3BTuner::fmStereoThresholdLevelSet(uint8_t const level)
{
    Command command =
        commandBuilder.createStream(CmdStreamId::FmStereoThresholdLevelSet).append(level).build();

    return commandSend(command);
}

/*
 *   Get FMstereoThdLevel
 *   data return = 0..10
 */
bool T3BTuner::fmStereoThresholdLevelGet(uint8_t* const level)
{
    Command command = commandBuilder.createStream(CmdStreamId::FmStereoThresholdLevelGet).build();
    return (commandSend(command) && responseUint8(0U, level));
}

/*
 *   Get RDS raw data
 *   return: 1=new RDS data, 2=no new RDS data, 3=no RDS data
 */
bool T3BTuner::fmRdsRawData(uint16_t* const blockA, uint16_t* const blockB, uint16_t* const blockC,
                            uint16_t* const blockD, uint16_t* const blerA, uint16_t* const blerB,
                            uint16_t* const blerC, uint16_t* const blerD)
{
    Command command = commandBuilder.createStream(CmdStreamId::FmRdsData).build();
    if (commandSend(command) && responseSize > 1U)
    {
        return (responseUint16(0, blockA) && responseUint16(2, blockB) && responseUint16(4U, blockC) &&
                responseUint16(6U, blockD) && responseUint16(8U, blerA) && responseUint16(10U, blerB) &&
                responseUint16(12U, blerC) && responseUint16(14U, blerD));
    }
    return false;
}

/*
 *   Set FMseekThreshold
 *   RSSIthreshold = 0..100
 */
bool T3BTuner::fmSeekThresholdSet(uint8_t const threshold)
{
    Command command = commandBuilder.createStream(CmdStreamId::FmSeekThresholdSet).append(threshold).build();

    return commandSend(command);
}

/*
 *   Get FMseekThreshold
 *   data return = 0..100
 */
bool T3BTuner::fmSeekThresholdGet(uint8_t* const threshold)
{
    Command command = commandBuilder.createStream(CmdStreamId::FmSeekThresholdGet).build();
    return (commandSend(command) && responseUint8(0U, threshold));
}

/*
 *   Set FMstereoThreshold
 *   RSSIthreshold = 0..100
 */
bool T3BTuner::fmStereoThresholdSet(uint8_t const threshold)
{
    Command command =
        commandBuilder.createStream(CmdStreamId::FmStereoThresholdSet).append(threshold).build();

    return commandSend(command);
}

/*
 *   Get FMstereoThreshold
 *   data return = 0..100
 */
bool T3BTuner::fmStereoThresholdGet(uint8_t* const threshold)
{
    Command command = commandBuilder.createStream(CmdStreamId::FmStereoThresholdGet).build();
    return (commandSend(command) && responseUint8(0U, threshold));
}

/*
 *   Get FM Exact station.
 */
bool T3BTuner::fmExactStationGet(FmExactStation* const exact)
{
    Command command = commandBuilder.createStream(CmdStreamId::FmExactStation).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(exact)));
}

// *************************
// ***** RTC ***************
// *************************

/*
 *  Set RTC clock
 *  year: 2017=17,2018=18, month: 1..12, day: 1..31, hour: 0..23, minute: 0..59, second: 0..59
 */
bool T3BTuner::clockSet(uint8_t const year, uint8_t const month, uint8_t const day, uint8_t const hour,
                        uint8_t const minute, uint8_t const second)
{
    Command command = commandBuilder.createRtc(CmdRtcId::Set)
                          .append(second)
                          .append(minute)
                          .append(hour)
                          .append(day)
                          .append(static_cast<uint8_t>(0x00))
                          .append(month)
                          .append(year)
                          .build();

    return commandSend(command);
}

/*
 *  Get RTC ckock
 *  year: 2017=17,2018=18, month: 1..12, day: 1..31, hour: 0..23, minute: 0..59, second: 0..59
 */
bool T3BTuner::clockGet(uint8_t* const year, uint8_t* const month, uint8_t* const day, uint8_t* const hour,
                        uint8_t* const minute, uint8_t* const second)
{
    Command command = commandBuilder.createRtc(CmdRtcId::Get).build();
    bool result = commandSend(command);
    result &= responseUint8(0U, second) & responseUint8(1U, minute);
    result &= responseUint8(2U, hour) & responseUint8(3U, day);
    result &= responseUint8(5U, month) & responseUint8(6U, year);
    return result;
}

/*
 *  Set RTC sync clock from stream enable
 */
bool T3BTuner::clockSyncSet(bool const enable)
{
    Command command = commandBuilder.createRtc(CmdRtcId::Sync).append(static_cast<uint8_t>(enable)).build();

    return commandSend(command);
}

/*
 *  Get RTC sync clock status
 */
bool T3BTuner::clockSyncGet(bool* const enabled)
{
    Command command = commandBuilder.createRtc(CmdRtcId::SyncStatus).build();
    bool result = commandSend(command);
    *enabled = response[0U];
    return result;
}

/*
 *  Get RTC clock status
 */
bool T3BTuner::clockStatusGet(ClockStatus* const status)
{
    Command command = commandBuilder.createRtc(CmdRtcId::StatusClock).build();
    return (commandSend(command) && responseUint8(0U, reinterpret_cast<uint8_t*>(status)));
}

// *************************
// ***** EVENTS ************
// *************************

/*
 *   Enabled / Disable event notifications.
 */
bool T3BTuner::eventEnable(bool const enable)
{
    uint16_t value = (enable) ? 0x7F : 0x00;
    Command command =
        commandBuilder.createNotification(CmdNotificationId::Notification).append(value).build();

    return commandSend(command);
}

bool T3BTuner::eventReceived()
{
    return (bool)stream.available();
}

/*
 *   Read event
 */
bool T3BTuner::eventRead(EventType* const type)
{
    bool result = (responseReceive() && ((CommandType)responseHeader[1U] == CommandType::Notification));
    *type = (EventType)responseHeader[2U];
    return result;
}

// *************************
// ***** PRIVATE FUNCTIONS *
// *************************

/*
 *  Send command to DAB module and wait for answer
 */
bool T3BTuner::commandSend(Command const& command)
{
    while (stream.available())
    {
        stream.read();
    }
    stream.write(command.data, command.size);
    stream.flush();
    return (responseReceive() &&
            !(responseHeader[1U] == ResponseAck && responseHeader[2U] == ResponseAckNack));
}

bool T3BTuner::responseReceive()
{
    uint16_t index = 0U;
    uint8_t data = 0U;
    uint32_t endMillis = systemMillis() + 200U; // timeout for answer from module = 200ms
    responseSize = 0U;

    while (systemMillis() < endMillis && index < T3BTunerMaxDataSize)
    {
        if (stream.available())
        {
            data = stream.read();
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

bool T3BTuner::responseText(char* const buffer, uint16_t const size)
{
    uint16_t j = 0U;
    for (uint16_t i = 0U; i < responseSize; i = i + 2U)
    {
        if (size <= j)
            return false;
        buffer[j++] = uint16ToChar(response[i], response[i + 1U]);
        discardTrailingSpaces(buffer);
    }
    return true;
}

bool T3BTuner::responseUint8(uint8_t const index, uint8_t* const resp)
{
    if (responseSize > index)
    {
        *resp = response[index];
        return true;
    }
    return false;
}

bool T3BTuner::responseUint16(uint8_t const index, uint16_t* const resp)
{
    if (responseSize > (index + 1U))
    {
        *resp = static_cast<uint16_t>(response[index + 1U]);
        *resp |= static_cast<uint16_t>(response[index]) << 8U;
        return true;
    }
    return false;
}

bool T3BTuner::responseUint32(uint8_t const index, uint32_t* const resp)
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
char T3BTuner::uint16ToChar(uint8_t const byte1, uint8_t const byte0)
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

void T3BTuner::discardTrailingSpaces(char* const text)
{
    int16_t index = strlen(text) - 1;

    while (index >= 0U && text[index] == ' ')
    {
        index--;
    }

    text[++index] = '\0';
}
