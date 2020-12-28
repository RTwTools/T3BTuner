#ifndef Command_h
#define Command_h

#include <stdint.h>

static const uint8_t CommandMaxSize = 20U;
static const uint8_t CommandStartValue = 0xFE;
static const uint8_t CommandEndValue = 0xFD;

struct Command
{
    uint8_t data[CommandMaxSize];
    uint8_t size = 0U;
};

#endif // Command_h
