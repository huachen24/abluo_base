#include "I2C_PROCESSING.h"

using namespace LibWireConstants;

unsigned char rc;
char receivedChars[numChars];
char speedChars[numChars];
// char padded_FL[8];
// char padded_FR[8];
// char padded_BL[8];
// char padded_BR[8];
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

// void requestEvent()
// {
//   dtostrf(posFL, 7, 0, padded_FL);
//   strcat(speedChars, padded_FL);
//   strcat(speedChars, ",");
//   dtostrf(posFR, 7, 0, padded_FR);
//   strcat(speedChars, padded_FR);
//   strcat(speedChars, ",");
//   dtostrf(posBL, 7, 0, padded_BL);
//   strcat(speedChars, padded_BL);
//   strcat(speedChars, ",");
//   dtostrf(posBR, 7, 0, padded_BR);
//   strcat(speedChars, padded_BR);
  
//   speedChars[0] = 0;
//   Wire.write(speedChars, 32);
//   posFL = 0;
//   posFR = 0;
//   posBL = 0;
//   posBR = 0;

// }

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