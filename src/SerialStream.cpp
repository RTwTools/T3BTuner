#include "SerialStream.h"

SerialStream::SerialStream(HardwareSerial* serial) : 
  SerialStream(serial, SerialType::Hardware) { }

SerialStream::SerialStream(SoftwareSerial* serial) :
  SerialStream(serial, SerialType::Software) { }

SerialStream::SerialStream(Stream* serialStream, SerialType type) :
  serialStream(serialStream),
  serialType(type)
{ }

void SerialStream::begin(uint32_t baud)
{
  switch (serialType)
  {
  case SerialType::Hardware:
    static_cast<HardwareSerial*>(serialStream)->begin(baud);
    break;
  case SerialType::Software:
    static_cast<SoftwareSerial*>(serialStream)->begin(baud);
    break;
  }
}

void SerialStream::setTimeout(uint32_t timeout)
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

uint32_t SerialStream::write(const uint8_t * buffer, uint32_t size)
{
  return serialStream->write(buffer, size);
}
