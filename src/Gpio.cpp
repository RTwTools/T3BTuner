#include "Gpio.h"
#include <Arduino.h>

void GpioModeSet(uint8_t const pin, GpioMode const mode)
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

GpioState GpioRead(uint8_t const pin)
{
  return (digitalRead(pin) == HIGH)
   ? GpioState::High
   : GpioState::Low;
}

void GpioWrite(uint8_t const pin, GpioState const value)
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