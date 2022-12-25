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
#include "src/I2C_PROCESSING.h"
#include "src/PWM_PROCESSING.h"

// Front Left Wheel - DC Controller 1 Motor 1
#define FL_EN 9
#define FL_IN1 49
#define FL_IN2 48

// Front Right Wheel - DC Controller 1 Motor 2
#define FR_EN 13
#define FR_IN1 43
#define FR_IN2 47

// Back Left Wheel - DC Controller 2 Motor 1
#define BL_EN 6
#define BL_IN1 7
#define BL_IN2 8

// Back Right Wheel - DC Controller 2 Motor 2
#define BR_EN 10
#define BR_IN1 12
#define BR_IN2 11

#define ESTOP_INT 3

// Helpers
bool settingUp = true;
bool emergency = false;
typedef void (*functiontype)();
unsigned long last_interrupt_time = millis();
unsigned long currentMicros = micros();
unsigned long currentMillis = millis();

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
void updateDcMotorState(int motorIndex, int direction, int speed, functiontype setForward, functiontype setBackward, functiontype brake)
{
  myMotors[motorIndex].speed = speed;
  myMotors[motorIndex].direction = direction;
  if (myMotors[motorIndex].speed)
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
    flDIR = 0;
    flPWM = -flPWM;
  }
  if (frPWM < 0 ) {
    frDIR = 0;
    frPWM = -frPWM;
  }
  if (blPWM < 0 ) {
    blDIR = 0;
    blPWM = -blPWM;
  }
  if (brPWM < 0 ) {
    brDIR = 0;
    brPWM = -brPWM;
  }
  updateDcMotorState(WHEELS_INDEX::FRONT_LEFT, flDIR, flPWM, set_FL_Forward, set_FL_Backward, brake_FL);
  updateDcMotorState(WHEELS_INDEX::FRONT_RIGHT, frDIR, frPWM, set_FR_Forward, set_FR_Backward, brake_FR);
  updateDcMotorState(WHEELS_INDEX::BACK_LEFT, blDIR, blPWM, set_BL_Forward, set_BL_Backward, brake_BL);
  updateDcMotorState(WHEELS_INDEX::BACK_RIGHT, brDIR, brPWM, set_BR_Forward, set_BR_Backward, brake_BR);
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
  for (int index = 0; index < digitalMotorPinCount; index++)
  {
    pinMode(digitalMotorPins[index], OUTPUT);
  }
  pinMode(ESTOP_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_INT), emergencyStop, FALLING);
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
    }
    else
    {
      parseInput();
    }
  }
}
