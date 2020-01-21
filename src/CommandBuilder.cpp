#include <string.h>
#include "CommandBuilder.h"

#define COMMAND_ID_START 0
#define COMMAND_ID_TYPE 1
#define COMMAND_ID_ID 2
#define COMMAND_ID_SIZE 5

#define COMMAND_VALUE_START 0xFE
#define COMMAND_VALUE_END 0xFD
#define COMMAND_VALUE_INIT_SIZE 6

CommandBuilder& CommandBuilder::Create(CommandType type, uint8_t id)
{
  memset(command.data, '\0', sizeof(command.data));

  command.data[COMMAND_ID_START] = COMMAND_VALUE_START;
  command.data[COMMAND_ID_TYPE] = (uint8_t)type;
  command.data[COMMAND_ID_ID] = id;

  command.size = COMMAND_VALUE_INIT_SIZE;

  return *this;
}

CommandBuilder& CommandBuilder::CreateSystem(CmdSystemId id)
{
  return Create(CommandType::System, (uint8_t)id);
}

CommandBuilder& CommandBuilder::CreateStream(CmdStreamId id)
{
  return Create(CommandType::Stream, (uint8_t)id);
}

CommandBuilder& CommandBuilder::CreateRtc(CmdRtcId id)
{
  return Create(CommandType::Rtc, (uint8_t)id);
}

CommandBuilder& CommandBuilder::CreateNotification(CmdNotificationId id)
{
  return Create(CommandType::Notification, (uint8_t)id);
}

CommandBuilder& CommandBuilder::Append(uint8_t value)
{
  if ((command.size + 1) < COMMAND_MAX_SIZE)
  {
    command.data[command.size++] = value;
    command.data[COMMAND_ID_SIZE]++;
  }
  
  return *this;
}

CommandBuilder& CommandBuilder::Append(uint16_t value)
{
  Append((uint8_t)((value >> 8) & 0xFF));
  Append((uint8_t)((value >> 0) & 0xFF));

  return *this;
}

CommandBuilder& CommandBuilder::Append(uint32_t value)
{
  Append((uint8_t)((value >> 24) & 0xFF));
  Append((uint8_t)((value >> 16) & 0xFF));
  Append((uint8_t)((value >> 8) & 0xFF));
  Append((uint8_t)((value >> 0) & 0xFF));
  
  return *this;
}

Command& CommandBuilder::Build()
{
  if (command.size < COMMAND_MAX_SIZE)
  {
    command.data[command.size++] = COMMAND_VALUE_END;
  }
  
  return command;
}
