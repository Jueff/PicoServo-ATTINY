/*
 * SPDX-FileCopyrightText: 2025-2026 Juergen Winkler <MobaLedLib@gmx.at>
 * SPDX-License-Identifier: BSL-1.1
 *
 * Description:
 *  This firmware runs on a Raspberry Pi Pico and controls up to 6 servo motors.
 *  It is part of the MobaLedLib project. It receives LED data, controls
 *  up to 6 servos (in two groups), and provides visual feedback via the onboard RGB LED
 *  Servo configuration and limits are stored in flash memory. The solution supports
 *  interactive setup and status indication using FastLED and MobaLedLib. 
 */

#include "pico/multicore.h"
#include "MobaLedLib.h"
#include "PicoFlashStorage.h"
#include "MLLServoConfigurator.h"
#include <AceButton.h>  
#include <Arduino.h>
#include <climits> 
#include "RP2040_PWM.h"

using namespace ace_button;

 // stringification helpers: convert numeric macro to string
#ifndef STRINGIFY2
#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)
#endif

// fallback in case APP_VERSION is not set
#ifndef APP_VERSION
#define APP_VERSION 0.0
#endif

const uint DATA_IN_PIN = 15;
const uint DATA_OUT_PIN = 14;
const uint STATUSLED_PIN = 13;
const uint DEBUG_PIN = 8;

const uint NUM_LEDS_TO_EMULATE = 2;                       // number of LEDs to be read from the data stream
const uint NUM_LEDS_TO_SKIP = NUM_LEDS_TO_EMULATE - 1;    // number of LEDs to skip in the data stream, folloing data are forwared to DOut
const uint NUM_SERVO_CONTROLLERS = 2;                     // number of servo controllers, each controller can handle 3 channels, so with 1 controller up to 3 servos can be controlled
const uint NUM_LEDS = 1;                                  // number of FASTLed output LEDs, only one LED used for status indication and setup mode
const char* bootMessage = "MobaLedLib Pico 6x-Servo ATTiny V" STRINGIFY(APP_VERSION);

#include "LEDReceiver.h"
#include "ServoConfigurator.h" 

// LED receiver members
LEDReceiver* pLEDReceiver;
uint8_t ledData[NUM_LEDS_TO_EMULATE * 3];

// servo members
ServoConfigurator* pConfigurator[NUM_SERVO_CONTROLLERS];
bool isInSetup = false;

// debug 
AceButton button(DEBUG_PIN);
uint8_t LogLevel;
uint8_t logState = 0;
#define MLLAPP_LOG(level,...) if (LogLevel>=level) Serial.printf(__VA_ARGS__);

// flash storage
#define SECTORS_TO_USE 4
using namespace PicoFlashStorage;
FlashStorage* pStorage;

// Status signal members and defines
uint8_t lastSignal = 0xff;
const uint8_t HB_INPUT = 0;
const uint8_t MAX_SIGNAL = 4;
const uint8_t MAX_BRIGHT = 20;
CHSV HSV;

// MobaLedLib
CRGB leds[NUM_LEDS];

MobaLedLib_Configuration()
{
  Blink3(0, C_BLUE, HB_INPUT + 0, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)               // blue flashing
  Blink3(0, C_YELLOW, HB_INPUT + 1, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)             // yellow flashing
  Blink3(0, C_RED, HB_INPUT + 2, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)                // red flashing
  // APatternT1(0, 193, HB_INPUT + 3, 1, 5, MAX_BRIGHT, 0, PF_EASEINOUT, 1 Sec, 1)  // green fading
  ConstRGB(0, HB_INPUT + 3, 0, 0, 0, 0, 0, 0)                                       // led off
  PatternT4(0, _NStru(C1, 4, 1), HB_INPUT + 4, _Cx2LedCnt(C1), 0, 255, 0, 0, 24 ms, 74 ms, 24 ms, 512 ms, _Cx2P_DBLFL(C1))  // red warning flashlight
  EndCfg // End of the configuration
};
MobaLedLib_Create(leds); // Define the MobaLedLib instance

void turnInputsOff()
{
  for (int i = 0; i <= MAX_SIGNAL; i++)
  {
    MobaLedLib.Set_Input(HB_INPUT + i, 0);
  }
  lastSignal = 0xff;
  MobaLedLib.Update();
}

void setSignal(LEDReceiver::State signal)
{
  /*
    Error       = 0,
    DataMissing = 1,
    Offline     = 2,
    Online      = 3,
    FlashError  = 4,
  */
  uint8_t newSignal = (uint8_t)signal;
  if (newSignal > MAX_SIGNAL) return;
  if (newSignal == lastSignal) return;
  turnInputsOff();
  lastSignal = newSignal;
  MLLAPP_LOG(3, "Set signal %d\r\n", newSignal);
  MobaLedLib.Set_Input(HB_INPUT + newSignal, 1);
}

RP2040_PWM* pwm;

void setup()
{
  Serial.begin(115200);
  for (int i = 0; i < 30; i++)
  {
    pinMode(i, INPUT);
  }
  pinMode(DEBUG_PIN, INPUT_PULLUP);

  // check if debug button is pressed during startup, if yes, wait 3 seconds for usb serial is connected and enable all logs
  if (digitalRead(DEBUG_PIN) == LOW)
  {
    // wait three seconds for the usb serial to attach
    auto startTime = millis();
    while ((millis() - startTime) < 3000)
    {
      // only enable for debugging purpose to see trace output of boot code
      if (Serial) break;
      delay(10);
    }
    MLLServoConfigurator::LogLevel = 8;
    FlashStorage::LogLevel = 8;
    LogLevel = 8;
    logState = 3;
    MLLAPP_LOG(1, "debug mode: All logs enabled\n");
  }
  else
  {
    MLLServoConfigurator::LogLevel = 0;
    FlashStorage::LogLevel = 0;
    LogLevel = 0;
  }
  MLLAPP_LOG(1, bootMessage);
  MLLAPP_LOG(3, "Initialize LED Receiver");
  pLEDReceiver = new LEDReceiver(&ledData[0], NUM_LEDS_TO_EMULATE, NUM_LEDS_TO_SKIP, DATA_IN_PIN, DATA_OUT_PIN);
  pLEDReceiver->setRepeaterLEDColor(0, 0, 0);

  MLLAPP_LOG(3, "Initialize FastLED");

  FastLED.addLeds<NEOPIXEL, STATUSLED_PIN>(leds, NUM_LEDS); // Initialize the FastLED library
  FastLED.addLeds<NEOPIXEL, 16>(leds, NUM_LEDS); // Initialize the FastLED library
  FastLED.setDither(DISABLE_DITHER);       // avoid sending slightly modified brightness values

  turnInputsOff();

/*  digitalWrite(2, HIGH);
  pwm = new RP2040_PWM(2, 50, INT_MAX);
  digitalWrite(2, HIGH);
  /*pwm->setPWM_Int(2, 50, INT_MAX);
  pwm->setPWM_Int(2, 50, INT_MAX);
  pwm->setPWM_Int(2, 50, INT_MAX);*/
  
  MLLAPP_LOG(3, "Initialize servos");

  uint16_t baseSectorNumber = (PICO_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE) - SECTORS_TO_USE;
  pStorage = new FlashStorage(baseSectorNumber, SECTORS_TO_USE, (uint8_t*)"MLLSRS10");
  if (!pStorage->isValid())
  {
    showCriticalError("can't use flash storage");
  }
  
  setupServos(pStorage, 0);
  MLLAPP_LOG(3, "Activating buttons");
  ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleButton);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
}

void setupServos(FlashStorage* pStorage, uint8_t basePin)
{

  for (int i = 0; i < NUM_SERVO_CONTROLLERS; i++)
  {
    uint8_t pins[3] = { (uint8_t)(basePin+i*3), (uint8_t)(basePin+1 + i * 3), (uint8_t)(basePin+2 + i * 3) };
    pConfigurator[i] = new ServoConfigurator(pStorage, i*3, 3, pins);
  }
}

void loop()
{
  pLEDReceiver->loop();
  if (LogLevel>0 && pLEDReceiver->hasDataChanged())
  {
    pLEDReceiver->DebugOutputLedData();
  }

  updateServos(0);
  if (!isInSetup) 
  {
    setSignal(pLEDReceiver->getState());
  }
  MobaLedLib.Update();
  FastLED.show();                       // Show the LEDs (send the leds[] array to the LED stripe)
  button.check();
}

void updateServos(uint8_t ledOffset)
{
  for (int i = 0; i < NUM_SERVO_CONTROLLERS; i++)
  {
    pConfigurator[i]->process(ledData + ledOffset + i * 3);
  }

  bool inSetupNow = false;
  for (int i = 0; i < NUM_SERVO_CONTROLLERS; i++)
  {
    if (pConfigurator[i]->isInSetup())
    {
      auto hue = (uint8_t)((1.0 - pConfigurator[i]->getPercentage()) * 160);  // 0% = 160 (Blau), 100% = 0 (rot)
      inSetupNow = true;
      if (!isInSetup)
      {
        MLLAPP_LOG(2, "Enter setup mode");
        isInSetup = true;
        turnInputsOff();
      }
      // use fastled to set the HSV value of led[0]
      if (HSV.hue != hue)
      {
        HSV.hue = hue;
        HSV.sat = 255;
        MLLAPP_LOG(4, "Servo %d in setup mode, position = %f hue = %d\r\n", i, pConfigurator[i]->getPercentage(), HSV.hue);
      }

      // if the percentage is below 10% or above 90%, make the LED blink to show possible critical values, otherwise show a constant light
      if (pConfigurator[i]->getPercentage() < 0.1 || pConfigurator[i]->getPercentage() > 0.9)
      {
        HSV.val = (millis() & 0x80)>0 ? 100: 0;
        leds[0] = HSV;
      }
      else
      { 
        HSV.val = 100;
      }
      leds[0] = HSV;
      break;
    }
  }
  if (isInSetup && !inSetupNow)
  {
    MLLAPP_LOG(2, "Leave setup mode");
    isInSetup = false;
    leds[0].setRGB(0, 0, 0);
    turnInputsOff();
  }
}

void showCriticalError(const char* message)
{
  auto lastTick = millis();
  do
  {
    if ((millis() - lastTick) > 1000)
    {
      lastTick = millis();
      Serial.println(message);
    }
    MobaLedLib.Update();
    FastLED.show(); // Show the LEDs (send the leds[] array to the LED stripe)
    delay(10);
  } while (true);
}

void handleButton(AceButton* button, uint8_t eventType, uint8_t buttonState) 
{
  switch (eventType) 
  {
  case AceButton::kEventPressed:
    shortPress();
    break;
  case AceButton::kEventLongPressed:
    longPress();
    break;
  }
}

void shortPress() 
{
  switch (logState)
  {
  case 0:
    LogLevel = 3;
    MLLAPP_LOG(3, "Log for main application enabled\n");
    MLLServoConfigurator::LogLevel = 0;
    FlashStorage::LogLevel = 0;
    logState = 1;
    break;
  case 1:
    MLLAPP_LOG(3, "Log for MLLServoConfigurator enabled\n");
    MLLServoConfigurator::LogLevel = 1;
    logState = 2;
    break;
  case 2:
    MLLAPP_LOG(3, "Log for FlashStorage enabled\n");
    FlashStorage::LogLevel = 4;
    logState = 0;
    break;
  }
}

void longPress() 
{
  MLLAPP_LOG(3, "logs turned off\n");
  MLLServoConfigurator::LogLevel = 0;
  FlashStorage::LogLevel = 0;
  LogLevel = 0;
  logState = 0;
}