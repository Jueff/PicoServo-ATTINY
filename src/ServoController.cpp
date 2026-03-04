#include "ServoController.h"

ServoController* ServoController::first = NULL;
ServoController* ServoController::last = NULL;

ServoController::ServoController(uint8_t pin, uint16_t min, uint16_t max, uint16_t speed)
{
  // a 50hz PWM
  // PWM - Periode is 20.000 microseconds(µs)
  // frequency = 1 / Periode
  // 20.000 µs = 20 ms = 0,02 Sekunden
  // 1 / 0,02 s = 50 Hz

  pwm = new RP2040_PWM(pin, 20000, 0);
  this->pin = pin;
  this->min = min;
  this->max = max;
  this->speed = speed;
  target = current = -1;
  lastValue = 0;
  if (first == NULL)
  {
    first = this;
    last = this;
    add_repeating_timer_us(20000, ServoController::repeating_timer_callback, NULL, &rTimer);    // 20ms timer for movement of the servo to target value
  }
  else
  {
    last->next = this;
    last = this;
  }
  next = NULL;
}

bool ServoController::setTarget(uint16_t value, bool checkLimits)
{
  if (value<1) return false;            // MLL sends 0 for pattern after restart - ignore that values(workaround until store_status for pattern works with MLL)
  if (disabled)         // re-enable PWM
  {
    Serial.printf("enable servo for pin %d\r\n", pin);
    disabled = false;
  }

  if (lastValue==value) return false;   // value didn't change
  lastValue = value;
  
  if (checkLimits)
  {
    if (lowerLimit < upperLimit)  // normal direction
    {
      if (value > upperLimit) target = upperLimit;
      else if (value < lowerLimit) target = lowerLimit;
      else target = value;
    }
    else                      // inverted direction
    {
      if (value < upperLimit) target = upperLimit;
      else if (value > lowerLimit) target = lowerLimit;
      else target = value;
    }
  }
  else
  {
    if (value > max) target = max;
    else if (value < min) target = min;
    else target = value;
  }
  Serial.printf("new value for pin %d = %d\r\n", pin, target);
  return true;
}
  
void ServoController::setDutyCycle()
{
  if (disabled) return;
  if (current==target) return;
  if (current==-1) // initial value
  {
    current=target;
  }

  if (speed == 0)
  {
    current = target;
  }
  else
  {
    int diff = target - current;
    if (abs(diff) > speed)
    {
      diff = speed * (diff < 0 ? -1 : 1);
    }
    current += diff;
  }
  pwm->setPWM_Int(pin, 50, current);
  //Serial.printf("duty cycle for Pin %d target %d current %d \r\n", pin, target, current);
  //Serial.printf("%d %d\n", millis(), current);
}

void ServoController::disable()
{
  if (!disabled)
  {
    Serial.printf("disable servo for pin %d\r\n", pin);
    pwm->setPWM_Int(pin, 50, INT_MAX);
    disabled = true;
  }
}

void ServoController::onTimer()
{
    setDutyCycle();
}

bool ServoController::repeating_timer_callback(struct repeating_timer *t)
{
    ServoController* current = first;
    while(current!=NULL)
    {
        current->onTimer();
        current = current->next;
    }
    return true;
}

void ServoController::setLowerLimit(uint16_t value)
{
  if (value < min) lowerLimit = min;
  else if (value > max) lowerLimit = max;
  else lowerLimit = value;
}

void ServoController::setUpperLimit(uint16_t value)
{
  if (value < min) upperLimit = min;
  else if (value > max) upperLimit = max;
  else upperLimit = value;
}

bool ServoController::isMoving() const
{
  return current != target;
}

bool ServoController::isInverted() const
{
  return upperLimit<lowerLimit;
}

bool ServoController::isReady() const
{
  return current != -1;
}

uint8_t ServoController::getPin() const
{
  return pwm->getPin();
}