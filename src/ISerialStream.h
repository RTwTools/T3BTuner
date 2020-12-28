#ifndef ISerialStream_h
#define ISerialStream_h

#include <stdint.h>

class ISerialStream
{
  public:
    virtual ~ISerialStream() = default;
    virtual void begin(uint32_t const baud) = 0;
    virtual void setTimeout(uint32_t const timeout) = 0;
    virtual void flush() = 0;
    virtual uint32_t available() = 0;
    virtual uint32_t read() = 0;
    virtual uint32_t write(uint8_t const* const buffer,
                           uint32_t const size) = 0;
};

#endif // ISerialStream_h
