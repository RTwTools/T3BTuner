#ifndef Command_h
#define Command_h

#include <stdint.h>

static uint8_t const CommandMaxSize = 20U;
static uint8_t const CommandStartValue = 0xFE;
static uint8_t const CommandEndValue = 0xFD;

struct Command
{
    uint8_t data[CommandMaxSize];
    uint8_t size = 0U;
};

#endif // Command_h
