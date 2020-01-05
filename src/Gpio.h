#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

enum class GpioMode : uint8_t
{
  Input,
  Output
};

enum class GpioState : uint8_t
{
  Low,
  High
};

void GpioModeSet(uint8_t pin, GpioMode mode);
GpioState GpioRead(uint8_t pin);
void GpioWrite(uint8_t pin, GpioState value);

#endif
