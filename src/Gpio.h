#ifndef Gpio_h
#define Gpio_h

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

void gpioModeSet(uint8_t const pin, GpioMode const mode);
GpioState gpioRead(uint8_t const pin);
void gpioWrite(uint8_t const pin, GpioState const value);

#endif // Gpio_h
