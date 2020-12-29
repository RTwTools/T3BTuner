#include "SystemAdapt.h"
#include <Arduino.h>

void systemDelay(uint32_t const milliSeconds)
{
    delay(milliSeconds);
}

uint32_t systemMillis()
{
    return millis();
}