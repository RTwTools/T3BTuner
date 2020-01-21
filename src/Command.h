#ifndef COMMAND_H
#define COMMAND_H

#include <inttypes.h>

#define COMMAND_MAX_SIZE 20

struct Command
{
  uint8_t data[COMMAND_MAX_SIZE];
  uint8_t size = 0;
};

#endif
