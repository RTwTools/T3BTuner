#ifndef ISERIALSTREAM_H
#define ISERIALSTREAM_H

#include <stdint.h>

class ISerialStream
{
public:
  virtual void begin(uint32_t baud) = 0;
  virtual void setTimeout(uint32_t timeout) = 0;
  virtual void flush() = 0;
  virtual uint32_t available() = 0;
  virtual uint32_t read() = 0;
  virtual uint32_t write(const uint8_t* buffer, uint32_t size) = 0;
};

#endif
