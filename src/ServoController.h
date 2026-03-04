#pragma once
#include <Arduino.h>
#include "RP2040_PWM.h"

class ServoController 
{
public:
    ServoController(uint8_t pin, uint16_t min, uint16_t max, uint16_t speed);

    void onTimer();

    uint16_t getTarget() { return target; }
    bool setTarget(uint16_t value, bool checkLimits);
    uint16_t getCurrent() { return current; }
    void setCurrent(uint16_t value);

    uint16_t getUpperLimit() const { return upperLimit; }
    void setUpperLimit(uint16_t value);

    uint16_t getLowerLimit() const { return lowerLimit; }
    void setLowerLimit(uint16_t value);

    uint16_t getSpeed() const { return speed; }
    void setSpeed(uint16_t value) { speed = value; }

    uint16_t getMinimum() const { return min; }
    uint16_t getMaximum() const { return max; }

    uint8_t getPin() const;

    void disable();

    bool isMoving() const;
    bool isReady() const;
    bool isInverted() const;

    static bool repeating_timer_callback(struct repeating_timer *t);
    ServoController* next;
    static ServoController* first;
    static ServoController* last;

protected:
  void setDutyCycle();

    RP2040_PWM* pwm;
    int         lastValue;
    int         current;
    int         target;
    bool        disabled;  // set to true if PWM should by turned off and pin set to high
    uint8_t     pin;
    uint16_t    min;   // the absoulte minimum value for the servo 
    uint16_t    max;   // the absoulte maximum value for the servo

    uint16_t    speed; // the maximum change of the servo position in 1/8 us per 20ms   
    uint16_t    lowerLimit; // the user configured minimum value for the servo position
    uint16_t    upperLimit; // the user configured maximum value for the servo position

    struct repeating_timer rTimer;
};