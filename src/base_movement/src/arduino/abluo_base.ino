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
#include <avr/dtostrf.h>
#include "src/I2C_PROCESSING.h"
#include "src/PWM_PROCESSING.h"

// Front Left Wheel - DC Controller 1 Motor 1
#define FL_EN 54
#define FL_IN1 55
#define FL_IN2 56

// Front Right Wheel - DC Controller 1 Motor 2
#define FR_EN 57
#define FR_IN1 58
#define FR_IN2 59

// Back Left Wheel - DC Controller 2 Motor 1
#define BL_EN 60
#define BL_IN1 61
#define BL_IN2 62

// Back Right Wheel - DC Controller 2 Motor 2
#define BR_EN 63
#define BR_IN1 64
#define BR_IN2 65

#define ESTOP_INT 3

// Encoder input pins (only 1 needs to be interrupt)
#define ENC_FL_A 2
#define ENC_FL_B 3

#define ENC_FR_A 4
#define ENC_FR_B 5

#define ENC_BL_A 6
#define ENC_BL_B 7

#define ENC_BR_A 8
#define ENC_BR_B 9

// Tick count for 1 rev
#define ENC_FL_REV 8900
#define ENC_FR_REV 8900
#define ENC_BL_REV 8900
#define ENC_BR_REV 8900

// Constants
const int measure_interval = 20;       // interval for velocity measurement (ms)
const float rev_to_rad = 6.283185307; // 2pi

// Helpers
bool settingUp = true;
bool emergency = false;
typedef void (*functiontype)();
unsigned long last_interrupt_time = millis();
unsigned long currentMicros = micros();
unsigned long currentMillis = millis();
unsigned long motion_duration;
unsigned long motion_start_millis;

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
char final_FL[8];
char final_FR[8];
char final_BL[8];
char final_BR[8];

// Enums for Wheel Index and Direction
enum WHEELS_INDEX
{
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  BACK_LEFT = 2,
  BACK_RIGHT = 3
};

// Software PWM
// Frequency: (1us x 58 Count = 58 us Period / 17.2 kHz)
const byte motorMicroInterval = 1;
const byte motorPwmMax = 58;
const byte motorPinsCount = 4;
const byte motorPwmPins[motorPinsCount] = {FL_EN, FR_EN, BL_EN, BR_EN};
PWM_PROCESSING pwmMotorsController;

// Digital Pins Motors (Pins not used for Software PWM Generation)
const byte digitalMotorPinCount = 8;
const byte digitalMotorPins[digitalMotorPinCount] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, BL_IN1, BL_IN2, BR_IN1, BR_IN2};

// Digital Pins Encoders
const byte digitalEncoderPinCount = 8;
const byte digitalEncoderPins[digitalEncoderPinCount] = {ENC_FL_A, ENC_FL_B, ENC_FR_A, ENC_FR_B, ENC_BL_A, ENC_BL_B, ENC_BR_A, ENC_BR_B};

// Motor Data Structures
struct Motor
{
  int motorId;
  int status;
  int direction;
  int speed;
};
const byte motorCount = 4;
Motor myMotors[motorCount];

// WHEELS DIRECTION & BRAKE
// FRONT LEFT (MC1A)
void set_FL_Forward()
{
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, LOW);
}
void set_FL_Backward()
{
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, HIGH);
}
void brake_FL()
{
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
}

// FRONT RIGHT (MC1B)
void set_FR_Forward()
{
  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, LOW);
}
void set_FR_Backward()
{
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, HIGH);
}
void brake_FR()
{
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
}

// BACK LEFT (MC2A)
void set_BL_Forward()
{
  digitalWrite(BL_IN1, HIGH);
  digitalWrite(BL_IN2, LOW);
}
void set_BL_Backward()
{
  digitalWrite(BL_IN1, LOW);
  digitalWrite(BL_IN2, HIGH);
}
void brake_BL()
{
  digitalWrite(BL_IN1, LOW);
  digitalWrite(BL_IN2, LOW);
}

// BACK RIGHT (MC2B)
void set_BR_Forward()
{
  digitalWrite(BR_IN1, HIGH);
  digitalWrite(BR_IN2, LOW);
}
void set_BR_Backward()
{
  digitalWrite(BR_IN1, LOW);
  digitalWrite(BR_IN2, HIGH);
}
void brake_BR()
{
  digitalWrite(BR_IN1, LOW);
  digitalWrite(BR_IN2, LOW);
}

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

void setupMotors()
{
  for (int index = 1; index <= motorCount; index++)
  {
    myMotors->motorId = index;
    myMotors->status = 0;
    myMotors->direction = 0;
    myMotors->speed = 0;
  }
}

/**
 * @brief Updates Direction & PWM Duty Cycle for Motor
 *
 * @param motorIndex index of motor to be updated
 * @param status on or off status of the motor
 * @param direction desired direction of spin of the motor
 * @param speed desired pwm duty cycle
 * @param setForward function to set direction 1 of motor
 * @param setBackward function to set direction 2 of motor
 * @param brake function to brake and stop motor
 */
void updateDcMotorState(int motorIndex, int status, int direction, int speed, functiontype setForward, functiontype setBackward, functiontype brake)
{
  myMotors[motorIndex].status = status;
  myMotors[motorIndex].speed = speed;
  myMotors[motorIndex].direction = direction;
  if (myMotors[motorIndex].status)
  {
    myMotors[motorIndex].direction == 1 ? setForward() : setBackward();
    pwmMotorsController.updatePinPwmDutyCycle(motorIndex, speed);
  }
  else
  {
    brake();
  }
}

void stopAllMotors()
{
  for (int i = 0; i < motorCount; i++)
  {
    myMotors[i].status = 0;
  }
  brake_FL();
  brake_FR();
  brake_BL();
  brake_BR();
}

/**
 * @brief Stops all motors when duration based motor motion is complete
 *
 */
void handleDurationBasedMotorMotion()
{
  if (motion_duration > 0)
  {
    currentMillis = millis();
    if (currentMillis - motion_start_millis > motion_duration)
    {
      motion_duration = 0;
      stopAllMotors();
    }
  }
}

/**
 * @brief Brakes and stops all motors
 *
 */
void handleStop()
{
  stopAllMotors();
}

/**
 * @brief Calls respective handler based on command
 *
 */
void handleCommand()
{
  int flDIR = 1;
  int frDIR = 1;
  int blDIR = 1;
  int brDIR = 1;
  int flPWM = payload[0];
  int frPWM = payload[1];
  int blPWM = payload[2];
  int brPWM = payload[3];
  if (flPWM < 0 ) {
    int flDIR = 0;
    flPWM = -flPWM;
  }
  if (frPWM < 0 ) {
    int frDIR = 0;
    frPWM = -frPWM;
  }
  if (blPWM < 0 ) {
    int blDIR = 0;
    blPWM = -blPWM;
  }
  if (brPWM < 0 ) {
    int brDIR = 0;
    brPWM = -brPWM;
  }
  updateDcMotorState(WHEELS_INDEX::FRONT_LEFT, 1, flDIR, flPWM, set_FL_Forward, set_FL_Backward, brake_FL);
  updateDcMotorState(WHEELS_INDEX::FRONT_RIGHT, 1, frDIR, frPWM, set_FR_Forward, set_FR_Backward, brake_FR);
  updateDcMotorState(WHEELS_INDEX::BACK_LEFT, 1, blDIR, blPWM, set_BL_Forward, set_BL_Backward, brake_BL);
  updateDcMotorState(WHEELS_INDEX::BACK_RIGHT, 1, brDIR, brPWM, set_BR_Forward, set_BR_Backward, brake_BR);
}

/**
 * @brief Checks for new serial inputs and calls command handler
 *
 */
void processNewData()
{
  if (newData == true)
  {
    parseInput();
    handleCommand();
  }
  newData = false;
}

void getEncoderData() {
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e3);

  if (deltaT > measure_interval)
  {

    prevT = currT;

    int posb_FL = 0;
    int posb_FR = 0;
    int posb_BL = 0;
    int posb_BR = 0;

    posb_FL = posFL;
    posb_FR = posFR;
    posb_BL = posBL;
    posb_BR = posBR;

    // rad/s
    ang_vel_FL = (float)(posb_FL / deltaT / ENC_FL_REV * rev_to_rad);
    ang_vel_FR = (float)(posb_FR / deltaT / ENC_FR_REV * rev_to_rad);
    ang_vel_BL = (float)(posb_BL / deltaT / ENC_BL_REV * rev_to_rad);
    ang_vel_BR = (float)(posb_BR / deltaT / ENC_BR_REV * rev_to_rad);

    dtostrf(ang_vel_FL, 7, 3, final_FL);
    strcat(speedChars, final_FL);
    strcat(speedChars, ",");
    dtostrf(ang_vel_FR, 7, 3, final_FR);
    strcat(speedChars, final_FR);
    strcat(speedChars, ",");
    dtostrf(ang_vel_BL, 7, 3, final_BL);
    strcat(speedChars, final_BL);
    strcat(speedChars, ",");
    dtostrf(ang_vel_BR, 7, 3, final_BR);
    strcat(speedChars, final_BR);
    
    speedChars[0] = 0;
    
    posFL = 0;
    posFR = 0;
    posBL = 0;
    posBR = 0;
  }
}

void emergencyStop()
{
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 50)
  {
    last_interrupt_time = interrupt_time;
    if (digitalRead(ESTOP_INT) == LOW)
    {
      emergency = true;
      stopAllMotors();
    }
  }
}

void setup()
{
  // Begin I2C and Setup Pins
  Wire.begin(0x70);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  for (int index = 0; index < digitalMotorPinCount; index++)
  {
    pinMode(digitalMotorPins[index], OUTPUT);
  }
  for (int index = 0; index < digitalEncoderPinCount; index++)
  {
    pinMode(digitalEncoderPins[index], INPUT);
  }
  pinMode(ESTOP_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_INT), emergencyStop, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), readFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), readFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), readBL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), readBR, RISING);
  pwmMotorsController.init(motorMicroInterval, motorPwmMax, motorPwmPins, motorPinsCount);
  settingUp = false;
}

void loop()
{
  if (digitalRead(ESTOP_INT) == LOW)
  {
    emergency = true;
    stopAllMotors();
  } 
  else
  {
    emergency = false;
  }
  if (!settingUp)
  {
    if (!emergency)
    {
      processNewData();
      pwmMotorsController.handlePWM();
      //getEncoderData();
    }
    else
    {
      parseInput();
      //getEncoderData();
    }
  }
}
