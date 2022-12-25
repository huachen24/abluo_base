/***
   Arduino Code: Command movement of 4 meccanum wheels and get encoder data
   @file abluo_base.ino
   @author Wang Huachen (huachenw24@gmail.com)
   @references: https://www.baldengineer.com/software-pwm-with-millis.html
   @version: 2.0 (6 December 2022)

  * Takes in I2C Input of the form <fl, fr, bl, br>
  * Updates Data Model to store MotorState
  * Executes Software PWM to control multiple motors handling Motors

*/
#include <util/atomic.h>
#include <Wire.h>

// Encoder input pins (only 1 needs to be interrupt)
#define ENC_FL_A 2
#define ENC_FL_B 4

#define ENC_FR_A 3
#define ENC_FR_B 5

#define ENC_BL_A 18
#define ENC_BL_B 16

#define ENC_BR_A 19
#define ENC_BR_B 17

// Tick count for 1 rev
#define ENC_FL_REV 550
#define ENC_FR_REV 550
#define ENC_BL_REV 550
#define ENC_BR_REV 550

// Constants
const int measure_freq = 100;       // freq for velocity measurement (hz)
const float twopi = 6.283185307; // 2pi

// Variables
volatile int posFL = 0;
volatile int posFR = 0;
volatile int posBL = 0;
volatile int posBR = 0;
long prevT = 0;
float ang_vel_FL = 0;
float ang_vel_FR = 0;
float ang_vel_BL = 0;
float ang_vel_BR = 0;
char padded_FL[8];
char padded_FR[8];
char padded_BL[8];
char padded_BR[8];
char speedChars[32];

// Digital Pins Encoders
const byte digitalEncoderPinCount = 8;
const byte digitalEncoderPins[digitalEncoderPinCount] = {ENC_FL_A, ENC_FL_B, ENC_FR_A, ENC_FR_B, ENC_BL_A, ENC_BL_B, ENC_BR_A, ENC_BR_B};

void readFL()
{
  int fl = digitalRead(ENC_FL_B);
  if (fl > 0)
  {
    posFL++;
  }
  else
  {
    posFL--;
  }
}

void readFR()
{
  int fr = digitalRead(ENC_FR_B);
  if (fr > 0)
  {
    posFR++;
  }
  else
  {
    posFR--;
  }
}

void readBL()
{
  int bl = digitalRead(ENC_BL_B);
  if (bl > 0)
  {
    posBL++;
  }
  else
  {
    posBL--;
  }
}

void readBR()
{
  int br = digitalRead(ENC_BR_B);
  if (br > 0)
  {
    posBR++;
  }
  else
  {
    posBR--;
  }
}

void requestEvent()
{
  long currT = millis();
  float deltaT = (float)(currT - prevT);

  ang_vel_FL = (float)(posFL * twopi / deltaT * 1000 / ENC_FL_REV);
  ang_vel_FR = (float)(posFR * twopi / deltaT * 1000 / ENC_FR_REV);
  ang_vel_BL = (float)(posBL * twopi / deltaT * 1000 / ENC_BL_REV);
  ang_vel_BR = (float)(posBR * twopi / deltaT * 1000 / ENC_BR_REV);

  speedChars[0] = 0;
  dtostrf(ang_vel_FL, 7, 3, padded_FL);
  strcat(speedChars, padded_FL);
  strcat(speedChars, ",");
  dtostrf(ang_vel_FR, 7, 3, padded_FR);
  strcat(speedChars, padded_FR);
  strcat(speedChars, ",");
  dtostrf(ang_vel_BL, 7, 3, padded_BL);
  strcat(speedChars, padded_BL);
  strcat(speedChars, ",");
  dtostrf(ang_vel_BR, 7, 3, padded_BR);
  strcat(speedChars, padded_BR);

  posFL = 0;
  posFR = 0;
  posBL = 0;
  posBR = 0;

  Wire.write(speedChars, 31);
  prevT = millis();
}

void setup()
{
  // Begin I2C and Setup Pins
  Wire.begin(0x65);
  Wire.setClock(400000);
  Wire.onRequest(requestEvent);
  for (int index = 0; index < digitalEncoderPinCount; index++)
  {
    pinMode(digitalEncoderPins[index], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), readFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), readFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), readBL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), readBR, RISING);
}

void loop()
{
//  long currT = millis();
//  float deltaT = (float)(currT - prevT);
//  if (deltaT > (1.0 / measure_freq)) {
//    prevT = currT;
//
//    int posb_FL = 0;
//    int posb_FR = 0;
//    int posb_BL = 0;
//    int posb_BR = 0;
//
//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
//    {
//      posb_FL = posFL;
//      posb_FR = posFR;
//      posb_BL = posBL;
//      posb_BR = posBR;
//    }
//    
//    ang_vel_FL = (float)(posb_FL * 1000 / deltaT / ENC_FL_REV * twopi);
//    ang_vel_FR = (float)(posb_FR * 1000 / deltaT / ENC_FR_REV * twopi);
//    ang_vel_BL = (float)(posb_BL * 1000 / deltaT / ENC_BL_REV * twopi);
//    ang_vel_BR = (float)(posb_BR * 1000 / deltaT / ENC_BR_REV * twopi);
//
//    speedChars[0] = 0;
//    dtostrf(ang_vel_FL, 7, 3, padded_FL);
//    strcat(speedChars, padded_FL);
//    strcat(speedChars, ",");
//    dtostrf(ang_vel_FR, 7, 3, padded_FR);
//    strcat(speedChars, padded_FR);
//    strcat(speedChars, ",");
//    dtostrf(ang_vel_BL, 7, 3, padded_BL);
//    strcat(speedChars, padded_BL);
//    strcat(speedChars, ",");
//    dtostrf(ang_vel_BR, 7, 3, padded_BR);
//    strcat(speedChars, padded_BR);
//
//  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
//    {
//      posFL = 0;
//      posFR = 0;
//      posBL = 0;
//      posBR = 0;
//    }
//  }
}
