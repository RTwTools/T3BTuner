#include <string.h>
#include "CommandBuilder.h"
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

TEST(CommandBuilderTests, Start_Ok)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;
  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)CommandType::Stream,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_END,
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

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id).Build();

  EXPECT_EQ(RESULT_OK, memcmp(command.data, expected, sizeof(expected)));
}

TEST(CommandBuilderTests, PayloadUint8_Ok)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;
  uint8_t payload = 0x54;

  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)CommandType::Stream,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)sizeof(payload),
    (uint8_t)payload,
    (uint8_t)COMMAND_VALUE_END
  };

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id)
    .Append(payload)
    .Build();

  EXPECT_EQ(RESULT_OK, memcmp(command.data, expected, sizeof(expected)));
  EXPECT_EQ(sizeof(payload) + COMMAND_PACKAGE_SIZE, command.size);
}

TEST(CommandBuilderTests, PayloadUint16_Ok)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;
  uint16_t payload = 0x5468;

  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)CommandType::Stream,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)sizeof(payload),
    (uint8_t)((payload >> 8) & 0xFF),
    (uint8_t)((payload >> 0) & 0xFF),
    (uint8_t)COMMAND_VALUE_END
  };

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id)
    .Append(payload)
    .Build();

  EXPECT_EQ(RESULT_OK, memcmp(command.data, expected, sizeof(expected)));
  EXPECT_EQ(sizeof(payload) + COMMAND_PACKAGE_SIZE, command.size);
}

TEST(CommandBuilderTests, PayloadUint32_Ok)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;
  uint32_t payload = 0x54689123;

  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)CommandType::Stream,
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

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id)
    .Append(payload)
    .Build();

  EXPECT_EQ(RESULT_OK, memcmp(command.data, expected, sizeof(expected)));
  EXPECT_EQ(sizeof(payload) + COMMAND_PACKAGE_SIZE, command.size);
}

TEST(CommandBuilderTests, Payload_False_CommandAlreadyFull)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id)
    .Append((uint32_t)0x01020304) // Id  6 -  9
    .Append((uint32_t)0x05060708) // Id 10 - 13
    .Append((uint32_t)0x09101112) // Id 14 - 17
    .Append((uint8_t)0x13)        // Id 18
    .Append((uint8_t)0x14)        // Not included already full.
    .Build();

  uint8_t expected[] =
    {
      (uint8_t)COMMAND_VALUE_START,
      (uint8_t)CommandType::Stream,
      (uint8_t)id,
      (uint8_t)COMMAND_VALUE_EMPTY,
      (uint8_t)COMMAND_VALUE_EMPTY,
      (uint8_t)13,
      0x01,
      0x02,
      0x03,
      0x04,
      0x05,
      0x06,
      0x07,
      0x08,
      0x09,
      0x10,
      0x11,
      0x12,
      0x13,
      (uint8_t)COMMAND_VALUE_END
    };


  EXPECT_EQ(RESULT_OK, memcmp(command.data, expected, sizeof(expected)));
  EXPECT_EQ(COMMAND_MAX_SIZE, command.size);
}

TEST(CommandBuilderTests, End_Ok)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;
  uint8_t expected[] =
  {
    (uint8_t)COMMAND_VALUE_START,
    (uint8_t)CommandType::Stream,
    (uint8_t)id,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_EMPTY,
    (uint8_t)COMMAND_VALUE_END
  };

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id).Build();

  EXPECT_EQ(RESULT_OK, memcmp(command.data, expected, sizeof(expected)));
}

TEST(CommandBuilderTests, End_ShouldntIncreasePayloadSize)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id).Build();

  EXPECT_EQ(COMMAND_VALUE_INIT_PAYLOAD_SIZE , command.data[COMMAND_ID_SIZE]);
}

TEST(CommandBuilderTests, End_ShouldIncreaseCommandSize)
{
  CmdStreamId id = CmdStreamId::DabRemoveOffAir;

  CommandBuilder commandBuilder;
  Command command = commandBuilder.CreateStream(id).Build();

  EXPECT_EQ(COMMAND_VALUE_INIT_SIZE + 1 , command.size);
}
