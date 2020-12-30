#include "Nextion.h"
#include <string.h>

static char const* LabelStation = "station";
static char const* LabelDebug = "debug";

static char const* InvalidCommand = "Invalid command!";

static char const* AssignmentStart = ".txt=\"";
static char const* AssignmentEnd = "\"\xFF\xFF\xFF";

static char const* next = "next";
static char const* prev = "prev";

static char const StartCommand = '$';
static char const EndCommand = ';';
static uint8_t const MaxCommandSize = 20U;
static uint8_t const Match = 0U;
static uint8_t const TimeoutMs = 200U;

Nextion::Nextion(ISerialStream& stream, T3BTuner& tuner) :
    stream{stream},
    tuner{tuner},
    command{new char[MaxCommandSize]},
    index{0}
{
    memset(command, 0U, MaxCommandSize);
}

Nextion::~Nextion()
{
    delete[] command;
}

void Nextion::execute()
{
    if (true == receive())
    {
        executeCommand();
    }
}

bool Nextion::receive()
{
    bool result = false;

    while (stream.available())
    {
        char const data = stream.read();

        if (data == StartCommand)
        {
            index = 0U;
            memset(command, 0U, MaxCommandSize);
        }
        else if (data == EndCommand)
        {
            result = true;
            break;
        }
        else
        {
            if ((index + 1U) < MaxCommandSize)
            {
                command[index++] = data;
            }
            else
            {
                // Command is too long, reset.
                index = 0U;
                setLabel(LabelDebug, "CommandTooLong");
                memset(command, 0U, MaxCommandSize);
            }
        }
    }

    return result;
}

void Nextion::executeCommand()
{
    if (strncmp(command, next, strlen(next)) == Match)
    {
        tuner.dabStationCount(&dabStationsCount);
        dabStationId = ((dabStationId + 1) == dabStationsCount) ? 0U : dabStationId + 1U;
        tuner.playDab(dabStationId);
        printDabStationName();
    }
    else if (strncmp(command, prev, strlen(prev)) == Match)
    {
        tuner.dabStationCount(&dabStationsCount);
        dabStationId = (dabStationId == 0U) ? dabStationsCount - 1U : dabStationId - 1U;
        tuner.playDab(dabStationId);
        printDabStationName();
    }
    else
    {
        setLabel(LabelDebug, InvalidCommand);
    }
}

void Nextion::printDabStationName()
{
    if (tuner.dabStationName(dabStationId, buffer, sizeof(buffer)))
    {
        setLabel(LabelStation, buffer);
    }    
}

void Nextion::setLabel(char const* label, char const* text)
{
    write(label);
    write(AssignmentStart);
    write(text);
    write(AssignmentEnd);
}

void Nextion::write(char const* data)
{
    write(data, strlen(data));
}

void Nextion::write(char const* data, uint32_t const size)
{
    stream.write(reinterpret_cast<uint8_t const*>(data), size);
}
