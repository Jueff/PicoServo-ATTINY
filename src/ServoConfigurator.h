#pragma once

#include "ServoController.h"
#include "PicoFlashStorage.h"

using namespace PicoFlashStorage;
class StatefulServoController;

class ServoConfigurator 
{
public:
  enum class eAction : int8_t {
    None = 0,
    Invalid = -1,
    Servo0 = 1,
    Servo1 = 2,
    Servo2 = 3,
    SetMinMax = 4,
    SetSpeed = 5,
    SetByButton = 6,
    Ok = 7
  };
  enum eButton : int8_t 
  {
    None = 0,
    Dec = -1,
    Inc = 1,
    Button1 = 2,
    Nothing1 = 3,
    Save = 4
  };

  enum eState : uint8_t       // States for the Mode variable
  {
    Init = 0,
    ServoSel = 1,
    MinMax = 2,
    MinMaxButtons = 3,
    Speed = 4,
    SpeedButtons = 5,
    TerServo = 6
  };

  enum eButtonMode : uint8_t 
  {
    ReadMin = 1,
    WaitEndMin = 2,
    ReadMax = 3,
    WaitEndMax = 4,
    End = 5
  };


public:
  ServoConfigurator(FlashStorage* pFlashStorage, uint8_t storageOffset, uint8_t numberOfServos, uint8_t pins[]);
  void process(uint8_t ledValue[]);
  bool isInSetup() const;
  bool isMoving() const;
  float getPercentage() const;

private:
  void changeModeServo(uint8_t servoNumber, eAction action);
  void processModeServo(uint8_t servoNumber, uint8_t ledValue);
  void controlServo(uint8_t ledValue, uint8_t servoNumber, bool limitRange);
  void saveMinMax(uint8_t servoNumber);
  void saveSpeed(uint8_t servoNumber);
  void savePosition(uint8_t servoNumber);

  eAction ledValueToAction(uint8_t pwm);
  eButton processUpDownButtons(uint8_t LED_pwm, int& Val, int min, int max, uint16_t minStep, uint16_t maxStep);
  void processTerServo(uint8_t LED_pwm);
  void readMinMaxButton(uint8_t ledValue);
  void readMinMax(uint8_t ledValue);
  void readSpeed(uint8_t ledValue);
  inline int16_t ledValueToSpeed(uint8_t value);
  bool isServoSelected() const; // true if a valid servo is selected for configuration, false otherwise

  StatefulServoController* pServo[3];
  FlashStorage* pFlashStorage;
  uint8_t       storageOffset;
  uint8_t       numberOfServos;
  uint32_t      lastWipeTime;        // the last time in millis a wipe occured
  uint32_t      lastChangeTime;      // the last time in millis a change of the speed occured in SpeedButtons mode, used to avoid changing the speed to fast

  eAction       lastAction;          // actions for special functions
  eState        state;               // Modes for the configuration state machine
  uint8_t       selectedServo;       // Number of the selected servo

  int16_t       currentMinVal;       // MinVal read im ReadMinMax()
  int16_t       currentMaxVal;       // MaxVal read im ReadMinMax()
  eButtonMode   buttonMode;          // Button Mode in ReadMinMaxButton()

};