#ifndef SerialStream_h
#define SerialStream_h

#include "ISerialStream.h"
#include <stdint.h>

class Stream;
class HardwareSerial;
class SoftwareSerial;

enum class SerialType : uint8_t
{
    Hardware,
    Software
};

class SerialStream : public ISerialStream
{
  public:
    SerialStream(HardwareSerial* const serial);
    SerialStream(SoftwareSerial* const serial);
    void begin(uint32_t const baud) override;
    void setTimeout(uint32_t const timeout) override;
    void flush() override;
    uint32_t available() override;
    uint32_t read() override;
    uint32_t write(uint8_t const* const buffer, uint32_t const size) override;

  private:
    SerialStream(Stream* const serialStream, SerialType const type);

    Stream* const serialStream;
    SerialType const serialType;
};

#endif // SerialStream_h
