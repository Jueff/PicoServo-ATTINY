/*
 * SPDX-FileCopyrightText: 2025-2026 Juergen Winkler <MobaLedLib@gmx.at>
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * This class is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This class is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this class; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/


#include "ServoConfigurator.h"

#include "StatefulServoController.h"

using eAction = ServoConfigurator::eAction;
using eButton = ServoConfigurator::eButton;
using eState = ServoConfigurator::eState;
using eButtonMode = ServoConfigurator::eButtonMode;

#define SERVO_MAX_VALUE 220
#define NO_SERVO_SEL 0xff

#define MIN_SERVO         ( 500*5)  // [us]/5   Die Servos haben ganz unterschiedliche PWM Bereiche
#define MAX_SERVO         (2500*5)  //   "      - SG90 hat den groessten Bereich von 600-2700us
#define DEF_MIN_SERVO     (1000*5)  //   "      Default values used if the EEPROM values are wrong
#define DEF_MAX_SERVO     (1500*5)  //   "

#define CNG_POS_SLOW          2     // slow change of the position in ReadMinMaxButton() per 20 ms
#define CNG_POS_FAST        200     // fast   "              "                "

#define CNG_SPD_SLOW          1     // slow change of the speed in ReadSpeed()
#define CNG_SPD_FAST         20     // fast   "             "         "

#define MIN_MOVE              1
#define MAX_MOVE            300

#define MIN_MOVE_BUTT         1
#define MAX_MOVE_BUTT       300

#define FLASHBLOCK_TYPE_MINMAX   0xF0
#define FLASHBLOCK_TYPE_SPEED    0xF1
#define FLASHBLOCK_TYPE_POSITION 0xF2

#define OptMap(x, in_min, in_max, out_min, out_max) (((long)((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min)) + (out_min))

ServoConfigurator::ServoConfigurator(FlashStorage* pFlashStorage, uint8_t storageOffset, uint8_t numberOfServos, uint8_t pins[])
{
  lastChangeTime = 0;
  lastWipeTime = 0;
  selectedServo = NO_SERVO_SEL;
  lastAction = eAction::None;
  state = eState::Init;
  this->pFlashStorage = pFlashStorage;
  this->storageOffset = storageOffset;
  this->numberOfServos = numberOfServos;
  FlashBlock fbs;

  uint16_t lowerLimit;
  uint16_t upperLimit;
  uint16_t speed;
  uint16_t position;

  for (int i = 0; i < numberOfServos; ++i)
  {
    this->pServo[i] = new StatefulServoController(pins[i], MIN_SERVO, MAX_SERVO, CNG_POS_SLOW);

    // read the min and max position for the servo from flash storage, if not valid use default values
    if (pFlashStorage->getBlock(fbs, FLASHBLOCK_TYPE_MINMAX, storageOffset + i))
    {
      Serial.printf("%d-%d: limit storage for servo is valid\n", storageOffset, i);
      lowerLimit = fbs.getWord(0);
      upperLimit = fbs.getWord(2);
    }
    else
    {
      Serial.printf("%d-%d: using default limit values\n", storageOffset, i);
      lowerLimit = DEF_MIN_SERVO;
      upperLimit = DEF_MAX_SERVO;
      // todo error handling
    }

    // read the speed for the servo from flash storage, if not valid use default value
    if (pFlashStorage->getBlock(fbs, FLASHBLOCK_TYPE_SPEED, storageOffset + i))
    {
      speed = fbs.getWord(0);
      Serial.printf("%d-%d: speed storage is valid\n", storageOffset, i);
    }
    else
    {
      Serial.printf("%d-%d: using default speed values\n", storageOffset, i);
      speed = (CNG_POS_SLOW + CNG_POS_FAST) / 2;
      // todo error handling
    }

    // read the position for the servo from flash storage, if not valid use default value
    if (pFlashStorage->getBlock(fbs, FLASHBLOCK_TYPE_POSITION, storageOffset + i))
    {
      pFlashStorage->dumpMemory(fbs.getAddress(), 8); 
      position = fbs.getWord(0);
      Serial.printf("%d-%d: position storage for is valid\n", storageOffset, i, position);
    }
    else
    {
      Serial.printf("%d-%d: using default position values\n", storageOffset, i);
      position = (lowerLimit + upperLimit) / 2;
      // todo error handling
    }

    pServo[i]->setLowerLimit(lowerLimit);
    pServo[i]->setUpperLimit(upperLimit);
    pServo[i]->setSpeed(speed);
    pServo[i]->setCurrent(position);  // todo check how to correclty init without setting the PWM
    //pServo[i]->setTarget(position, false);
    Serial.printf("%d-%d: position %d lowerLimit %d upperLimit %d speed %d\n", storageOffset, i,
      pServo[i]->getTarget(), pServo[i]->getLowerLimit(), pServo[i]->getUpperLimit(), pServo[i]->getSpeed());
  }
}

void ServoConfigurator::process(uint8_t ledValue[])
{
  for (uint8_t servoNumber = 0; servoNumber < numberOfServos; servoNumber++)
  {
    processModeServo(servoNumber, ledValue[servoNumber]);
  }
}

//-------------------------------
void ServoConfigurator::changeModeServo(uint8_t servoNumber, eAction action)
//-------------------------------
// Is called once if a different action was detected
{
  switch (action)
  {
    case eAction::Servo0:     //                 (LED PWM 225)
    case eAction::Servo1:     //                 (LED PWM 230)
    case eAction::Servo2:     //                 (LED PWM 235)
    {
      state = eState::ServoSel;
      auto newSel = (uint8_t)action - (uint8_t)eAction::Servo0;
      if (newSel < numberOfServos)
      {
        selectedServo = newSel;
        Serial.printf("%d-%d servo selected\r\n", storageOffset, selectedServo);
      }
      break;
    }

    case eAction::SetMinMax: 
      if (isServoSelected())  // State eState::MinMax  (LED PWM 240)
      {
        state = eState::MinMax;
        currentMinVal = pServo[selectedServo]->getMaximum();  // reset the values
        currentMaxVal = pServo[selectedServo]->getMinimum();  // reset the values
        pServo[selectedServo]->buttonChanged = 0;
        //Serial.println("%d-%d: eState::MinMax", storageOffset, i);
        break;
      }

    case eAction::SetSpeed:  // State eState::Speed   (LED PWM 245)
    {
      if (isServoSelected())
      {
        if (state == eState::MinMax || state == eState::MinMaxButtons)
        {
          currentMaxVal = pServo[selectedServo]->getTarget();
          saveMinMax(selectedServo);
          pServo[selectedServo]->setTarget(pServo[selectedServo]->getCurrent(), true);
          Serial.printf("%d-%d: lowerLimit %d upperlimit %d\n", storageOffset, pServo[selectedServo]->getLowerLimit(), pServo[selectedServo]->getUpperLimit());
          state = eState::Speed;
        }
      }
      break;
    }

    case eAction::SetByButton:  // (LED PWM 250)
      Serial.printf("%d-%d: eAction::SetByButton state %d\n", storageOffset, state);
      if (state == eState::MinMax && isServoSelected())
      {
        state = eState::MinMaxButtons;
        Serial.printf("%d-%d: new state is MinMaxButtons\n", storageOffset, state);
        currentMinVal = pServo[selectedServo]->getLowerLimit();
        currentMaxVal = pServo[selectedServo]->getUpperLimit();
        buttonMode = eButtonMode::ReadMin;
        pServo[selectedServo]->setTarget(pServo[selectedServo]->getLowerLimit(), true);
      }
      if (state == eState::Speed && isServoSelected())
      {
        state = eState::SpeedButtons;
      }
      else if (state == eState::Init)
      {
        Serial.printf("%d-%d: TerServo\n", storageOffset, selectedServo);         // Debug: State eState::TerServo (LED PWM 250)
        state = eState::TerServo;
        selectedServo = servoNumber;   // Prevent that other channels disable the mode again because then the mode is toggled permanently
      }
      break;

    case eAction::Ok: switch (state)  // Save  (LED PWM 254) not 255 because 255 is not always detected
    {
    case eState::MinMax:
    case eState::MinMaxButtons: saveMinMax(selectedServo);    break;
    case eState::Speed:
    case eState::SpeedButtons:  saveSpeed(selectedServo);     break;
    default:; // Prevent compiler warning
      // no break, continue with the next block
    }

    case eAction::None:
    {
      if (state != eState::Init)
      {
        state = eState::Init;
        Serial.printf("%d-%d: change to eState::Init\n", storageOffset, state);
        selectedServo = NO_SERVO_SEL;
      }
      break;
    }
  }
}

void ServoConfigurator::processModeServo(uint8_t servoNumber, uint8_t ledValue)
{
  if (state == eState::Init || selectedServo == servoNumber)
  {
    eAction action = ledValueToAction(ledValue); // 0=0, 1-222=-1, 223-227=1, 228-232=2, 233-237=3, 238-242=4, 243-247=5, 248-252=6, 253-255=7
    //if (((int8_t)action)>1) Serial.printf("%d-%d: ProcModeServer: state %d LED_pwm %d Action %d\r\n", storageOffset, servoNumber, state, ledValue, action);

    if (action != eAction::Invalid && action != lastAction)
    {
      if (isServoSelected()) Serial.printf("%d-%d: Action %d\n", storageOffset, servoNumber, action); // Debug
      lastAction = action;
      changeModeServo(servoNumber, action);
    }
    else
    {
      switch (state)
      {
      case eState::Init:          controlServo(ledValue, servoNumber, true);       break;// The selected servo is controlled
      case eState::ServoSel:      controlServo(ledValue, selectedServo, true); break;// The selected servo is controlled
      case eState::MinMax:
      case eState::MinMaxButtons: readMinMax(ledValue);                        break;
      case eState::Speed:
      case eState::SpeedButtons:  readSpeed(ledValue);                         break;
      case eState::TerServo:      processTerServo(ledValue);                   break;
      default: break;
      }
    }
    if (isServoSelected()) pServo[selectedServo]->lastLedValue = ledValue;
  }
}

//----------------------------------------------------------------------
void ServoConfigurator::controlServo(uint8_t ledValue, uint8_t servoNumber, bool limitRange)
//----------------------------------------------------------------------
{
  //if (servoNumber==0) Serial.printf("%d-%d: Control_Servo value %d\n", storageOffset, servoNumber, ledValue);
  if (ledValue == 0)
  {
    pServo[servoNumber]->disable();
  }
  else
  {
    if (ledValue <= SERVO_MAX_VALUE)
    {
      uint16_t val;
      if (limitRange)
      {
        uint8_t tmpledValue = ledValue; // pServo[servoNumber]->isInverted() ? SERVO_MAX_VALUE - ledValue + 1 : ledValue;
        val = map(tmpledValue, 1, SERVO_MAX_VALUE, pServo[servoNumber]->getLowerLimit(), pServo[servoNumber]->getUpperLimit());
      }
      else
      {
        val = map(ledValue, 1, SERVO_MAX_VALUE, pServo[servoNumber]->getMinimum(), pServo[servoNumber]->getMaximum());
      }
      
      if (val != pServo[servoNumber]->getTarget())
      {
        Serial.printf("%d-%d: controlServo value %d mapped to %d\n", storageOffset, servoNumber, ledValue, val);
        pServo[servoNumber]->setTarget(val, limitRange);                     // 1 - 220
      }
    }
  }

  // Save the Servo position
  if (pServo[servoNumber]->shouldSavePosition())
  {
    savePosition(servoNumber);
    pServo[servoNumber]->resetSavePosition();
  }
  if (pServo[servoNumber]->shouldAutomaticallyDisable())
  {
    pServo[servoNumber]->disable();
  }
}

//-------------------------------------------------------------------------------------------------------------------
eButton ServoConfigurator::processUpDownButtons(uint8_t LED_pwm, int& val, int min, int max, uint16_t minStep, uint16_t maxStep)
//-------------------------------------------------------------------------------------------------------------------
// Change Val with "buttons" (LED_pwm) in the range of 1..100..200
// 0       = Noting    return  0
// 1-95    = Dec Val   return -1
// 100     = Nothing   return  0
// 105-200 = Inc Val   return  1
// 205     = Button 1  return  2
// 210-220 = Nothing   return  3
// 225-255 = Nothing   return  4
{
  eButton res;
  if (LED_pwm < 1) return eButton::None;
  else if (LED_pwm <= 95 + 2) { res = eButton::Dec; val -= OptMap(LED_pwm, 1, 95, maxStep, minStep); }
  else if (LED_pwm <= 100 + 2) return eButton::None;
  else if (LED_pwm <= 200 + 2) { res = eButton::Inc; val += OptMap(LED_pwm, 105, 200, minStep, maxStep); }
  else if (LED_pwm <= 205 + 2) return eButton::Button1;
  else if (LED_pwm <= 220 + 2) return eButton::Nothing1;
  else                         return eButton::Save;

  val = constrain(val, min, max);
  return  res;
}

//---------------------
void ServoConfigurator::readMinMaxButton(uint8_t ledValue)
//---------------------
// Move the servo with some "Buttons" and store the MinMax values
{
  int current = pServo[selectedServo]->getTarget();
  if (pServo[selectedServo]->lastLedValue == ledValue)
  {
    return;
  }
  eButton button = processUpDownButtons(ledValue, current, MIN_SERVO, MAX_SERVO, CNG_POS_SLOW, CNG_POS_FAST);

  if (pServo[selectedServo]->getTarget() != current)
  {
    Serial.printf("%d-%d: value before %d, now %d\n", storageOffset, selectedServo, pServo[selectedServo]->getTarget(), current);
    pServo[selectedServo]->setTarget(current, false, true);  // todo move the servo with full speed
  }

  // 0/100=Nothing, ret 0; 1-95=Dec Val, ret -1; 105-200=Inc Val, ret 1, 205=Button1, ret 2,
  // 210-220=Noting, ret 3; 225-255=Noting, ret 4
  if (!(button == 1 || button == -1)) // 0, 100, > 200
  {
    if (!pServo[selectedServo]->isMoving()) pServo[selectedServo]->disable();
  }

  if (button == 1 || button == -1) pServo[selectedServo]->buttonChanged = 1;

  if (button == 2) // 205
  { // Button 1 pressed
      Serial.printf("%d-%d: button pressed, buttonMode = %d\n", storageOffset, selectedServo, buttonMode);
      switch (buttonMode)
      {
      case eButtonMode::ReadMin:
          currentMinVal = current;
          buttonMode= eButtonMode::WaitEndMin;
          pServo[selectedServo]->setTarget(pServo[selectedServo]->getUpperLimit(), false);
          break;

      case eButtonMode::ReadMax:
          currentMaxVal = current;
          buttonMode = eButtonMode::WaitEndMax;
          break;
      }
      pServo[selectedServo]->buttonChanged = 0;
  }
  else 
  { // Button released or other PWM signal
    if (buttonMode == eButtonMode::WaitEndMin)
    {
      buttonMode = eButtonMode::ReadMax;
    }
    else if (buttonMode >= eButtonMode::ReadMax && button == eButton::Save) // res == 4: PWM 225 - 255
    {
        currentMaxVal = current;
        saveMinMax(selectedServo);
        Serial.printf("%d-%d: limit setting ends\n", storageOffset, selectedServo); // Debug
    }
  }
}

//---------------
void ServoConfigurator::readMinMax(uint8_t ledValue)
//---------------
// Two different ways to set the min/max positions
// - A "button" mode where the position is inc/dec with a LED PWM number in the range of 1-95: Dec, 105-200: Inc
//   The Min / Max values are stored if the PWM value 205 is received
// - A absolute mode where the Min and Max values are set with LED PWM values in the range of 1 to 220
{
  if (state == eState::MinMaxButtons)
  {
    readMinMaxButton(ledValue);
  }
  else 
  {
    controlServo(ledValue, selectedServo, false); // The selected servo is controlled
    auto target = pServo[selectedServo]->getTarget();
    if (target != 0)
    {
      if (target < currentMinVal) currentMinVal = target;
      if (target > currentMaxVal) currentMaxVal = target;
    }
  }
}


//--------------
void ServoConfigurator::readSpeed(uint8_t ledValue)
//--------------
{
  // Wipe
  StatefulServoController* pCurrent = pServo[selectedServo];
  uint32_t t = millis();
  if (!pCurrent->isMoving())
  {
    if (t - lastWipeTime > 500) // 500 ms pause at the end
    {
      if (pCurrent->getTarget()== pCurrent->getUpperLimit()) pCurrent->setTarget(pCurrent->getLowerLimit(), true);
      else
      {
        //cdSerial.printf("Wipe offset %d pin %d Ch %d to %d\n", storageOffset, pCurrent->getPin(), selectedServo, pCurrent->getUpperLimit());
        pCurrent->setTarget(pCurrent->getUpperLimit(), true);
      }
      if (pCurrent->getSpeed() == 0) lastWipeTime = t; // At maximal speed the LastWipeT is not set below
                                                  // => The waiting time is reduced by the moving duration
    }                                             //    No pause if the servo is slow and the travel distance high
  }
  else lastWipeTime = t; // Is not called if Sel_p->Move_Inc == 0

  // Change the speed
  if (state == eState::SpeedButtons)
  {
    if (t - lastChangeTime > 200) // Don't change the speed to fast
    {
      int speed = pCurrent->getSpeed();
      auto eButton = processUpDownButtons(ledValue, speed, MIN_MOVE_BUTT, MAX_MOVE_BUTT, CNG_SPD_SLOW, CNG_SPD_FAST);
      //Serial.printf("speed change button %d speed %d led %d\r\n", eButton, speed, ledValue);
      if (eButton::None != eButton)
      {
        lastChangeTime = t;
        pCurrent->setSpeed(speed);
        Serial.printf("%d-%d: new speed %d\n", storageOffset, selectedServo, speed); // Debug
      }
    }
  }
  else 
  {
    if (ledValue > 0)
    {
      if (ledValue < 218)
      {
        pCurrent->setSpeed(ledValueToSpeed(ledValue));
      }
      else
      {
        if (ledValue < 223) pCurrent->setSpeed(0);;      // Maximal Speed
      }
    }
  }
}

//----------------------------------
void ServoConfigurator::processTerServo(uint8_t ledValue)
//----------------------------------
// Control all servos with one LED PWM channel.
// The PWM values are coded ternary.
// Servo values:
//  0 = Disable Servo signal
//  1 = Minimal Position  (L)   (Left and right may be swapped depending on the servo)
//  2 = Maximal Position  (R)
//
// PWM Values in the range form 35 to 165 are used:
//
// PWM   Servo A Servo B Servo C                   (L,R could be swapped depending of the servo type)
// ~~~   ~~~~~~~ ~~~~~~~ ~~~~~~~
// 35    off     off     off
// 40    L       off     off
// 45    R       off     off
// 50    off     L       off
// 55    L       L       off
// 60    R       L       off
// 65    off     R       off
// 70    L       R       off
// 75    R       R       off
// 80    off     off     L
// 85    L       off     L
// 90    R       off     L
// 95    off     L       L
// 100   L       L       L
// 105   R       L       L
// 110   off     R       L
// 115   L       R       L
// 120   R       R       L
// 125   off     off     R
// 130   L       off     R
// 135   R       off     R
// 140   off     L       R
// 145   L       L       R
// 150   R       L       R
// 155   off     R       R
// 160   L       R       R
// 165   R       R       R
{

  Serial.println(F("TerServo")); // Debug

  /*
  if (LED_pwm < 33 || LED_pwm > 167) LED_pwm = 35; // All Servos are unchanged

  uint8_t v = (LED_pwm - 35 + 2) / 5; // -35 = Offset, +2 in case the signal is two points smaller then expected (Valid range 35 => 33 - 37)
  Channel_Data_t* lcp = &Channel_Data[0];
  for (uint8_t i = 0; i < 3; i++, lcp++)
  {
    uint8_t Servo = v % 3;
    uint8_t Ctl_LED_pwm;
    switch (Servo)
    {
    case 0:  Ctl_LED_pwm = 0;   break; // Disable the servo
    case 1:  Ctl_LED_pwm = 1;   break; // MinVal
    default: Ctl_LED_pwm = 220;        // MaxVal
    }
    Control_Servo(Ctl_LED_pwm, i, 1);
    v /= 3;

    if (do_print) Print_(Servo); // Debug (Doesn't use memory if DEBUG_TERSERVO is not defined)
  }
  if (do_print) Println(F(""));  // Debug
  */
}


// -------------------- various helper functions to convert ledValue --------------------
eAction ServoConfigurator::ledValueToAction(uint8_t value)
//-----------------------------
//       0 = 0
//   1-222 =-1
// 223-227 = 1 \              Servo 0
// 228-232 = 2  Select servo  Servo 1
// 233-237 = 3 /              Servo 2
// 238-242 = 4  Set Min_Max
// 243-247 = 5  Set Speed
// 248-252 = 6  Set By_Button
// 253-255 = 7  Ok
//
{
  if (value == 0)   return eAction::None;
  if (value < 223) return eAction::Invalid;
  return static_cast<eAction>OptMap(value, 223, 253, 1, 7);
}

inline int16_t ServoConfigurator::ledValueToSpeed(uint8_t value)
{
  return OptMap(value, 2, 220, MIN_MOVE, MAX_MOVE);
}


// --------------------- getters ---------------------

bool ServoConfigurator::isServoSelected() const
{
  return selectedServo < numberOfServos;
}

bool ServoConfigurator::isInSetup() const
{
  return state == eState::MinMax || state == eState::MinMaxButtons || state == eState::Speed || state == eState::SpeedButtons;
}

bool ServoConfigurator::isMoving() const
{
  if (!isServoSelected()) return false;
  return pServo[selectedServo]->isMoving();
}

float ServoConfigurator::getPercentage() const
{
  if (!isServoSelected()) return 0;
  if (state == eState::Speed || state == eState::SpeedButtons)
  {
    uint16_t current = pServo[selectedServo]->getSpeed();
    // return the percent of speed between min and max
    float speedPercent = ((current - MIN_MOVE) * 100);
    speedPercent = speedPercent / (MAX_MOVE - MIN_MOVE);
    //Serial.printf("%d-%d: getPercentage: current %d min %d max %d speedPercent %f\n", storageOffset, selectedServo, current, MIN_MOVE, MAX_MOVE, speedPercent);
    return speedPercent / 100;
  }

  uint16_t current = pServo[selectedServo]->getCurrent();
  // return the percent of current between min and max
  float posPercent = ((current - pServo[selectedServo]->getMinimum()) * 100);
  posPercent  = posPercent / (pServo[selectedServo]->getMaximum() - pServo[selectedServo]->getMinimum());
  //Serial.printf("%d-%d: getPosition: current %d min %d max %d posPercent %f\n", storageOffset, selectedServo, current, pServo[selectedServo]->getMinimum(), pServo[selectedServo]->getMaximum(), posPercent);
  return posPercent / 100;
}

// --------------------- save to flash ---------------------
void ServoConfigurator::savePosition(uint8_t servoNumber)
//-----------------------------------
{
  if (servoNumber >= numberOfServos) return;
  auto val = pServo[servoNumber]->getCurrent();

  FlashWriteBlock fwb(FLASHBLOCK_TYPE_POSITION, storageOffset + servoNumber);
  fwb.setWord(val, 0);
  if (!pFlashStorage->write(&fwb))
  {
    Serial.printf("%d-%d: error saving position %d\n", storageOffset, servoNumber, val);
    // todo error handling
  }
  else
  {
    Serial.printf("%d-%d: position %d saved\n", storageOffset, servoNumber, val);
  }
}

void ServoConfigurator::saveMinMax(uint8_t servoNumber)
{
  if (servoNumber >= numberOfServos) return;

  pServo[servoNumber]->setLowerLimit(currentMinVal);
  pServo[servoNumber]->setUpperLimit(currentMaxVal);

  FlashWriteBlock fwb(FLASHBLOCK_TYPE_MINMAX, storageOffset + servoNumber);
  fwb.setWord(currentMinVal, 0);
  fwb.setWord(currentMaxVal, 2);
  if (!pFlashStorage->write(&fwb))
  {
    Serial.printf("%d-%d: error saving limits min=%d max=%d\n",
      storageOffset, servoNumber, pServo[servoNumber]->getLowerLimit(), pServo[servoNumber]->getUpperLimit());
    // todo error handling
  }
  else
  {
    Serial.printf("%d-%d: limits min=%d max=%d saved\n",
      storageOffset, servoNumber, pServo[servoNumber]->getLowerLimit(), pServo[servoNumber]->getUpperLimit());
  }
}

void ServoConfigurator::saveSpeed(uint8_t servoNumber)
//--------------
{

  if (servoNumber >= numberOfServos) return;
  auto val = pServo[servoNumber]->getSpeed();

  FlashWriteBlock fwb(FLASHBLOCK_TYPE_SPEED, storageOffset + servoNumber);
  fwb.setWord(val, 0);
  if (!pFlashStorage->write(&fwb))
  {
    Serial.printf("%d-%d: error saving speed %d\n", storageOffset, servoNumber, val);
    // todo error handling
  }
  else
  {
    Serial.printf("%d-%d: speed %d saved\n", storageOffset, servoNumber, val);
  }
}
