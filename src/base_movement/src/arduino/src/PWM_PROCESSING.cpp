#include "PWM_PROCESSING.h"

/**
 * @brief Construct a new pwm processing::pwm processing object
 *
 * @param microInterval
 * @param pwmMax
 * @param pwmPins
 */
void PWM_PROCESSING::init(int microInterval, int pwmMax, const byte *pwmPins, const byte pwmPinsCount)
{
    this->previousMicros = 0;
    this->microInterval = microInterval;
    this->pwmMax = pwmMax;
    this->pwmPinCount = pwmPinsCount;
    this->setupPWMpins(pwmPins);
}

/**
 * Execute Software PWM Loop
 * ...
 *
 * Increments pwmTickCount for each PWM Pin
 * If ON: If pwmTickCount >= pwmValue for pin, sets pinState to OFF
 *  (ie. if pwmValue = 125, btw 125 and 255 will OFF)
 * If OFF: If pwmTickCount >= pwmMax for pin, sets pinState to ON
 *  (ie. exceed 255)
 */
void PWM_PROCESSING::handlePWM()
{
    unsigned long currentMicros = micros();
    if (currentMicros - this->previousMicros >= this->microInterval)
    {
        for (int index = 0; index < this->pwmPinCount; index++)
        {
            myPWMpinsArray[index].pwmTickCount++;
            if (myPWMpinsArray[index].pinState == ON)
            {
                if (myPWMpinsArray[index].pwmTickCount >= myPWMpinsArray[index].pwmValue)
                {
                    myPWMpinsArray[index].pinState = OFF;
                }
            }
            else
            {
                if (myPWMpinsArray[index].pwmTickCount >= pwmMax)
                {
                    myPWMpinsArray[index].pinState = ON;
                    myPWMpinsArray[index].pwmTickCount = 0;
                }
            }
            digitalWrite(myPWMpinsArray[index].pin, myPWMpinsArray[index].pinState);
        }
        this->previousMicros = currentMicros;
    }
}

void PWM_PROCESSING::updatePinPwmValue(int pinIndex, int newPwmValue)
{
    this->myPWMpinsArray[pinIndex].pwmValue = newPwmValue;
}
void PWM_PROCESSING::updatePinPwmDutyCycle(int pinIndex, int dutyCycle)
{
    this->myPWMpinsArray[pinIndex].pwmValue = dutyCycle*0.01*pwmMax;
}

void PWM_PROCESSING::setupPWMpins(const byte *pwmPins)
{
    for (int index = 0; index < pwmPinCount; index++)
    {
        myPWMpinsArray[index] = (pwmPin){.pin = pwmPins[index], .pwmValue = 0, .pinState = ON, .pwmTickCount = 0};
        pinMode(pwmPins[index], OUTPUT);
    }
}