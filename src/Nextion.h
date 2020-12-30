#ifndef Nextion_h
#define Nextion_h

#include "ISerialStream.h"
#include "T3BTuner.h"

class Nextion
{
  public:
    Nextion(ISerialStream& stream, T3BTuner& tuner);
    ~Nextion();
    void execute();

  private:
    ISerialStream& stream;
    T3BTuner& tuner;
    char* command;
    uint8_t index;

    char buffer[T3BTunerMaxTextSize];
    uint32_t dabStationId;
    uint32_t dabStationsCount;

    bool receive();
    void executeCommand();
    void printDabStationName();
    void setLabel(char const* label, char const* text);
    void write(const char* data);
    void write(const char* data, uint32_t const size);
};

#endif // Nextion_h