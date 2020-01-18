#include <string.h>
#include "Command.h"
#include "gtest/gtest.h"

#define COMMAND_ID_TYPE 1
#define COMMAND_ID_ID 2
#define COMMAND_ID_SIZE 5

#define COMMAND_VALUE_START 0xFE
#define COMMAND_VALUE_EMPTY 0x00
#define COMMAND_VALUE_END 0xFD
#define COMMAND_VALUE_INIT_SIZE 6
#define COMMAND_VALUE_INIT_PAYLOAD_SIZE 0
#define COMMAND_PACKAGE_SIZE 7

#define RESULT_OK 0

TEST(CommandTests, Start_Ok)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;
  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)type,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY
  };

  Command command;
  command.Start(type, id);

  EXPECT_EQ(RESULT_OK, memcmp(command.GetData(), expected, sizeof(expected)));
}

TEST(CommandTests, PayloadUint8_Ok)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;
  uint8_t payload = 0x54;

  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)type,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)sizeof(payload),
    (uint8_t)payload,
    (uint8_t)COMMAND_VALUE_END
  };

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.Append(payload));
  EXPECT_TRUE(command.End());
  EXPECT_EQ(RESULT_OK, memcmp(command.GetData(), expected, sizeof(expected)));
  EXPECT_EQ(sizeof(payload) + COMMAND_PACKAGE_SIZE, command.GetSize());
}

TEST(CommandTests, PayloadUint16_Ok)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;
  uint16_t payload = 0x5468;

  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)type,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)sizeof(payload),
    (uint8_t)((payload >> 8) & 0xFF),
    (uint8_t)((payload >> 0) & 0xFF),
    (uint8_t)COMMAND_VALUE_END
  };

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.Append(payload));
  EXPECT_TRUE(command.End());
  EXPECT_EQ(RESULT_OK, memcmp(command.GetData(), expected, sizeof(expected)));
  EXPECT_EQ(sizeof(payload) + COMMAND_PACKAGE_SIZE, command.GetSize());
}

TEST(CommandTests, PayloadUint32_Ok)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;
  uint32_t payload = 0x54689123;

  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)type,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)sizeof(payload),
    (uint8_t)((payload >> 24) & 0xFF),
    (uint8_t)((payload >> 16) & 0xFF),
    (uint8_t)((payload >> 8) & 0xFF),
    (uint8_t)((payload >> 0) & 0xFF),
    (uint8_t)COMMAND_VALUE_END
  };

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.Append(payload));
  EXPECT_TRUE(command.End());
  EXPECT_EQ(RESULT_OK, memcmp(command.GetData(), expected, sizeof(expected)));
  EXPECT_EQ(sizeof(payload) + COMMAND_PACKAGE_SIZE, command.GetSize());
}

TEST(CommandTests, Payload_False_CommandAlreadyFull)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.Append((uint32_t)0x00)); // Id  6 -  9
  EXPECT_TRUE(command.Append((uint32_t)0x00)); // Id 10 - 13
  EXPECT_TRUE(command.Append((uint32_t)0x00)); // Id 14 - 17
  EXPECT_TRUE(command.Append((uint8_t)0x00)); //  Id 18

  // Id 19 is reserved for end of command char
  EXPECT_FALSE(command.Append((uint8_t)0x00));
}

TEST(CommandTests, End_Ok)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;
  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)type,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_END
  };

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.End());
  EXPECT_EQ(RESULT_OK, memcmp(command.GetData(), expected, sizeof(expected)));
}

TEST(CommandTests, End_False_CommandAlreadyFull)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.Append((uint32_t)0x00)); // Id  6 -  9
  EXPECT_TRUE(command.Append((uint32_t)0x00)); // Id 10 - 13
  EXPECT_TRUE(command.Append((uint32_t)0x00)); // Id 14 - 17
  EXPECT_TRUE(command.Append((uint8_t)0x00)); //  Id 18
  EXPECT_TRUE(command.End()); // Id 19

  // Id == 20 Command already full!
  EXPECT_FALSE(command.End()); 
}

TEST(CommandTests, End_ShouldntIncreasePayloadSize)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.End());
  EXPECT_EQ(COMMAND_VALUE_INIT_PAYLOAD_SIZE , command.GetData()[COMMAND_ID_SIZE]);
}

TEST(CommandTests, End_ShouldIncreaseCommandSize)
{
  CommandType type = CommandType::Stream;
  CommandId id = CommandId::StreamDabRemoveOffAir;

  Command command;
  command.Start(type, id);

  EXPECT_TRUE(command.End());
  EXPECT_EQ(COMMAND_VALUE_INIT_SIZE + 1 , command.GetSize());
}