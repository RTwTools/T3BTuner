#include "Command.h"

#define LENGTH_ID 5
#define HEADER_SIZE 6
#define VALUE_START 0xFE
#define VALUE_EMPTY 0x00
#define VALUE_END 0xFD

void Command::Start(CommandType type, CommandId id)
{
  data[0] = VALUE_START;
  data[1] = (uint8_t)type;
  data[2] = (uint8_t)id;
  data[3] = VALUE_EMPTY;
  data[4] = VALUE_EMPTY;
  data[5] = VALUE_EMPTY;
  size = HEADER_SIZE;
}

void Command::Append(uint8_t param)
{
  // TODO Check if index is too big for data!

  data[size] = param;
  data[LENGTH_ID]++;
  size++;
}

void Command::Append(uint16_t param)
{
  Append((uint8_t)((param >> 8) & 0xFF));
  Append((uint8_t)((param >> 0) & 0xFF));
}

void Command::Append(uint32_t param)
{
  Append((uint8_t)((param >> 24) & 0xFF));
  Append((uint8_t)((param >> 16) & 0xFF));
  Append((uint8_t)((param >> 8) & 0xFF));
  Append((uint8_t)((param >> 0) & 0xFF));
}

void Command::End()
{
  data[size++] = VALUE_END;
}