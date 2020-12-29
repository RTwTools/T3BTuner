#include "CommandBuilder.h"
#include <string.h>

static uint8_t const CommandStartId = 0U;
static uint8_t const CommandTypeId = 1U;
static uint8_t const CommandIdentifierId = 2U;
static uint8_t const CommandSizeId = 5U;
static uint8_t const CommandInitSize = 6U;

CommandBuilder& CommandBuilder::create(CommandType const type, uint8_t const id)
{
    memset(command.data, 0U, sizeof(command.data));

    command.data[CommandStartId] = CommandStartValue;
    command.data[CommandTypeId] = static_cast<uint8_t>(type);
    command.data[CommandIdentifierId] = id;

    command.size = CommandInitSize;

    return *this;
}

CommandBuilder& CommandBuilder::createSystem(CmdSystemId const id)
{
    return create(CommandType::System, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::createStream(CmdStreamId const id)
{
    return create(CommandType::Stream, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::createRtc(CmdRtcId const id)
{
    return create(CommandType::Rtc, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::createNotification(CmdNotificationId const id)
{
    return create(CommandType::Notification, static_cast<uint8_t>(id));
}

CommandBuilder& CommandBuilder::append(uint8_t const value)
{
    if ((command.size + 1) < CommandMaxSize)
    {
        command.data[command.size++] = value;
        command.data[CommandSizeId]++;
    }

    return *this;
}

CommandBuilder& CommandBuilder::append(uint16_t const value)
{
    append(static_cast<uint8_t>(((value >> 8U) & 0xFF)));
    append(static_cast<uint8_t>(((value >> 0U) & 0xFF)));

    return *this;
}

CommandBuilder& CommandBuilder::append(uint32_t const value)
{
    append(static_cast<uint8_t>(((value >> 24U) & 0xFF)));
    append(static_cast<uint8_t>(((value >> 16U) & 0xFF)));
    append(static_cast<uint8_t>(((value >> 8U) & 0xFF)));
    append(static_cast<uint8_t>(((value >> 0U) & 0xFF)));

    return *this;
}

Command& CommandBuilder::build()
{
    if (command.size < CommandMaxSize)
    {
        command.data[command.size++] = CommandEndValue;
    }

    return command;
}
