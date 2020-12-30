#include "Nextion.h"
#include "SerialStream.h"
#include "T3BTuner.h"
#include <SoftwareSerial.h>

// Nextion display is connected to the default HardwareSerial -> Serial
static uint8_t const PinMute = 2U;
static uint8_t const PinReset = 3U;
static uint8_t const PinTunerTx = 8U;
static uint8_t const PinTunerRx = 9U;

auto tunerSerial = new SoftwareSerial(PinTunerRx, PinTunerTx);
auto tunerStream = new SerialStream(tunerSerial);
auto nextionStream = new SerialStream(&Serial);

T3BTuner tuner(*tunerStream, PinReset, PinMute);
Nextion nextion(*nextionStream, tuner);

void setup()
{
    Serial.begin(57600);
    tuner.init();
    tuner.reset();
    tuner.dabSearch();
    tuner.audioOutput();
    tuner.volumeSet(8);
}

void loop()
{
    nextion.execute();
    delay(20);
}
