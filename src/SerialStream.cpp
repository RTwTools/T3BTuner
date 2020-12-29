#include "SerialStream.h"
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Stream.h>

SerialStream::SerialStream(HardwareSerial* const serial) : SerialStream(serial, SerialType::Hardware)
{
}

SerialStream::SerialStream(SoftwareSerial* const serial) : SerialStream(serial, SerialType::Software)
{
}

SerialStream::SerialStream(Stream* const serialStream, SerialType const type) :
    serialStream(serialStream),
    serialType(type)
{
}

void SerialStream::begin(uint32_t const baud)
{
    switch (serialType)
    {
        case SerialType::Hardware:
        {
            static_cast<HardwareSerial*>(serialStream)->begin(baud);
            break;
        }
        case SerialType::Software:
        {
            static_cast<SoftwareSerial*>(serialStream)->begin(baud);
            break;
        }
    }
}

void SerialStream::setTimeout(uint32_t const timeout)
{
    serialStream->setTimeout(timeout);
}

void SerialStream::flush()
{
    serialStream->flush();
}

uint32_t SerialStream::available()
{
    return serialStream->available();
}

uint32_t SerialStream::read()
{
    return serialStream->read();
}

uint32_t SerialStream::write(uint8_t const* const buffer, uint32_t const size)
{
    return serialStream->write(buffer, size);
}
