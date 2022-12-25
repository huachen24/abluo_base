#ifndef I2C_PROCESSING_H
#define I2C_PROCESSING_H
#include <Arduino.h>
#include <Wire.h>

namespace LibWireConstants
{
    const byte numChars = 32;
    const byte numInputs = 4;
}

extern boolean newData;
extern int payload[];
extern char receivedChars[];

static char *input;
static char *token;
extern void receiveEvent(int numChar);
extern void parseInput();

#endif