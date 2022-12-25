#ifndef PWM_PROCESSING_H
#define PWM_PROCESSING_H
#define ON true
#define OFF false
#define MAX_DC_MOTOR_COUNT 8
#include <Arduino.h>

class PWM_PROCESSING
{
public:
    struct pwmPin
    {
        unsigned int pin;          // Pin Number
        unsigned int pwmValue;     // Pwm Value (duty cycle)
        bool pinState;    // Pin Output State
        unsigned int pwmTickCount; // PWM Counter Value
    };
    PWM_PROCESSING(){};
    ~PWM_PROCESSING(){};
    void init(int microInterval, int pwmMax, const byte *pwmPins, const byte pwmPinsCount);
    void handlePWM();
    void updatePinPwmValue(int pinIndex, int newPwmValue);
    void updatePinPwmDutyCycle(int pinIndex, int dutyCycle);
    void setupPWMpins(const byte *pwmPins);
private:
    unsigned long previousMicros;
    unsigned long microInterval;
    pwmPin myPWMpinsArray[MAX_DC_MOTOR_COUNT];
    unsigned int pwmPinCount;
    unsigned int pwmMax;
};
#endif