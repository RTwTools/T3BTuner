#include <string.h>
#include "Command.h"

#define COMMAND_ID_START 0
#define COMMAND_ID_TYPE 1
#define COMMAND_ID_ID 2
#define COMMAND_ID_SIZE 5

#define COMMAND_VALUE_START 0xFE
#define COMMAND_VALUE_END 0xFD
#define COMMAND_VALUE_INIT_SIZE 6

const uint8_t* Command::GetData()
{
  return data;
}

uint8_t Command::GetSize()
{
  return size;
}

void Command::Start(CommandType type, CommandId id)
{
  memset(data, '\0', sizeof(data));

  data[COMMAND_ID_START] = COMMAND_VALUE_START;
  data[COMMAND_ID_TYPE] = (uint8_t)type;
  data[COMMAND_ID_ID] = (uint8_t)id;

  size = COMMAND_VALUE_INIT_SIZE;
}

bool Command::Append(uint8_t value)
{
  bool result = ((size + 1) < COMMAND_MAX_SIZE);

  if (result)
  {
    data[size++] = value;
    data[COMMAND_ID_SIZE]++;
  }
  
  return result;
}

bool Command::Append(uint16_t value)
{
  bool result = true;

  result |= Append((uint8_t)((value >> 8) & 0xFF));
  result |= Append((uint8_t)((value >> 0) & 0xFF));

  return result;
}

bool Command::Append(uint32_t value)
{
  bool result = true;

  result |= Append((uint8_t)((value >> 24) & 0xFF));
  result |= Append((uint8_t)((value >> 16) & 0xFF));
  result |= Append((uint8_t)((value >> 8) & 0xFF));
  result |= Append((uint8_t)((value >> 0) & 0xFF));
  
  return result;
}

bool Command::End()
{
  bool result = (size < COMMAND_MAX_SIZE);

  if (result)
  {
    data[size++] = COMMAND_VALUE_END;
  }
  
  return result;
}