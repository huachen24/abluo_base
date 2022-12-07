#include "I2C_PROCESSING.h"

using namespace LibWireConstants;

unsigned char rc;
char receivedChars[numChars];
char speedChars[numChars];
int payload[numInputs];
boolean newData = false;

void receiveEvent(int numChar) {
    static byte idx = 0;
    Wire.read();
    while (Wire.available()) {
      rc = Wire.read();
      if (rc != '\n') {
        receivedChars[idx] = rc;
        idx++;
      } else {
        receivedChars[idx] = '\0';
        idx = 0;
      }
    }
    newData = true;
}

void requestEvent()
{
  Wire.write(speedChars, 32);
}

void parseInput()
{
    input = receivedChars;
    newData = false;
    int counter = 0;
    while ((token = strtok_r(input, ",", &input)))
    {
        payload[counter] = atoi(token);
        counter++;
    }
}