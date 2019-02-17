#ifndef SERIALSTREAM_H
#define SERIALSTREAM_H

#include <stdint.h>
#include <Stream.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "ISerialStream.h"

enum class SerialType : uint8_t
{
  Hardware,
  Software
};

class SerialStream : public ISerialStream
{
public:
  SerialStream(HardwareSerial* serial);
  SerialStream(SoftwareSerial* serial);
  void begin(uint32_t baud);
  void setTimeout(uint32_t timeout);
  void flush();
  uint32_t available();
  uint32_t read();
  uint32_t write(const uint8_t* buffer, uint32_t size);

private:
  SerialStream(Stream* serialStream, SerialType type);
  Stream * serialStream;
  SerialType serialType;
};

#endif

