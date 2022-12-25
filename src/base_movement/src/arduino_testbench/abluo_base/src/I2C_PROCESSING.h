#ifndef I2C_PROCESSING_H
#define I2C_PROCESSING_H
#include <Arduino.h>
#include <Wire.h>

namespace LibWireConstants
{
    const byte numChars = 32;
    const byte numInputs = 4;
}

extern bool newData;
extern int payload[];
extern char receivedChars[];
extern char speedChars[];

static char *input;
static char *token;
extern void receiveEvent(int numChar);
// extern void requestEvent();
extern void parseInput();

#endif