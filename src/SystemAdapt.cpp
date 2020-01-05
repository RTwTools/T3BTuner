#include "SystemAdapt.h"
#include <Arduino.h>

void SystemDelay(uint32_t milliSeconds)
{
  delay(milliSeconds);
}

uint32_t SystemMillis()
{
  return millis();
}