#include "CommandBuilder.h"
#include <string.h>

static const uint8_t CommandStartId = 0U;
static const uint8_t CommandTypeId = 1U;
static const uint8_t CommandIdentifierId = 2U;
static const uint8_t CommandSizeId = 5U;
static const uint8_t CommandInitSize = 6U;

CommandBuilder& CommandBuilder::Create(CommandType const type, uint8_t const id)
{
    memset(command.data, 0U, sizeof(command.data));

    command.data[CommandStartId] = CommandStartValue;
    command.data[CommandTypeId] = static_cast<uint8_t>(type);
    command.data[CommandIdentifierId] = id;

    command.size = CommandInitSize;

    return *this;
}

CommandBuilder& CommandBuilder::CreateSystem(CmdSystemId const id)
{
    return Create(CommandType::System, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::CreateStream(CmdStreamId const id)
{
    return Create(CommandType::Stream, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::CreateRtc(CmdRtcId const id)
{
    return Create(CommandType::Rtc, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::CreateNotification(CmdNotificationId const id)
{
    return Create(CommandType::Notification, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::Append(uint8_t const value)
{
    if ((command.size + 1) < CommandMaxSize)
    {
        command.data[command.size++] = value;
        command.data[CommandSizeId]++;
    }

    return *this;
}

CommandBuilder& CommandBuilder::Append(uint16_t const value)
{
    Append(static_cast<uint8_t>(((value >> 8U) & 0xFF)));
    Append(static_cast<uint8_t>(((value >> 0U) & 0xFF)));

    return *this;
}

CommandBuilder& CommandBuilder::Append(uint32_t const value)
{
    Append(static_cast<uint8_t>(((value >> 24U) & 0xFF)));
    Append(static_cast<uint8_t>(((value >> 16U) & 0xFF)));
    Append(static_cast<uint8_t>(((value >> 8U) & 0xFF)));
    Append(static_cast<uint8_t>(((value >> 0U) & 0xFF)));

    return *this;
}

Command& CommandBuilder::Build()
{
    if (command.size < CommandMaxSize)
    {
        command.data[command.size++] = CommandEndValue;
    }

    return command;
}
