#include "Gpio.h"
#include <Arduino.h>

void gpioModeSet(uint8_t const pin, GpioMode const mode)
{
    if (mode == GpioMode::Input)
    {
        pinMode(pin, INPUT);
    }
    else
    {
        pinMode(pin, OUTPUT);
    }
}

GpioState gpioRead(uint8_t const pin)
{
    return (digitalRead(pin) == HIGH) ? GpioState::High : GpioState::Low;
}

void gpioWrite(uint8_t const pin, GpioState const value)
{
    if (value == GpioState::High)
    {
        digitalWrite(pin, HIGH);
    }
    else
    {
        digitalWrite(pin, LOW);
    }
}