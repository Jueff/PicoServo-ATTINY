/*
 * SPDX-FileCopyrightText: 2025-2026 Juergen Winkler <MobaLedLib@gmx.at>
 * SPDX-License-Identifier: CC-BY-NC-4.0
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

 // stringification helpers: convert numeric macro to string
#ifndef STRINGIFY2
#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)
#endif

// Fallback, falls APP_VERSION nicht gesetzt ist
#ifndef APP_VERSION
#define APP_VERSION 0.0
#endif

const uint DATA_IN_PIN = 15;
const uint DATA_OUT_PIN = 14;
const uint STATUSLED_PIN = 13;

const uint NUM_LEDS_TO_EMULATE = 2;
const uint NUM_LEDS_TO_SKIP = NUM_LEDS_TO_EMULATE - 1;
const uint NUM_SERVO_CONTROLLERS = 2;
const char* bootMessage = "MobaLedLib Pico 6x-Servo ATTiny V" STRINGIFY(APP_VERSION);

#include "LEDReceiver.h"
#include "ServoConfigurator.h" 

bool dataChanged = false;
bool isInSetup = false;
LEDReceiver* pLEDReceiver;
#define NUM_LEDS 20  
CRGB leds[NUM_LEDS];           // Define the array of leds

uint8_t lastSignal = 0xff;
uint8_t ledData[NUM_LEDS_TO_EMULATE * 3];

ServoConfigurator* pConfigurator[NUM_SERVO_CONTROLLERS];

#define SECTORS_TO_USE 4
using namespace PicoFlashStorage;

FlashStorage* pStorage;

#define HB_INPUT 0
#define MAX_SIGNAL 4
#define MAX_BRIGHT 20

MobaLedLib_Configuration()
{
  //BlueLight1(0, C3, HB_INPUT + 2)
  Blink3(0, C_BLUE, HB_INPUT + 0, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)
  Blink3(0, C_YELLOW, HB_INPUT + 1, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)
  Blink3(0, C_RED, HB_INPUT + 2, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)
  APatternT1(0, 193, HB_INPUT + 3, 1, 5, MAX_BRIGHT, 0, PF_EASEINOUT, 1 Sec, 1)
  PatternT4(0, _NStru(C1, 4, 1), HB_INPUT + 4, _Cx2LedCnt(C1), 0, 255, 0, 0, 24 ms, 74 ms, 24 ms, 512 ms, _Cx2P_DBLFL(C1))

  //Switchable_RGB_Heartbeat_Color(0, HB_INPUT + 3, 20, 70, 170, 1000)

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
  Serial.printf("Set signal %d\r\n", newSignal);
  MobaLedLib.Set_Input(HB_INPUT + newSignal, 1);
}

void setup()
{
  Serial.begin(115200);
  // only enable for debugging purpose to see trace output of boot code
  // while (!Serial) {}
  Serial.println(bootMessage);
 
  for (int i = 0; i < 30; i++)
  {
    pinMode(i, INPUT);
  }

  Serial.println("Initialize LED Receiver");
  pLEDReceiver = new LEDReceiver(&ledData[0], NUM_LEDS_TO_EMULATE, NUM_LEDS_TO_SKIP, DATA_IN_PIN, DATA_OUT_PIN);

  Serial.println("Initialize FastLED");

  FastLED.addLeds<NEOPIXEL, STATUSLED_PIN>(leds, NUM_LEDS); // Initialize the FastLED library
  FastLED.addLeds<NEOPIXEL, 16>(leds, NUM_LEDS); // Initialize the FastLED library
  FastLED.setDither(DISABLE_DITHER);       // avoid sending slightly modified brightness values

  turnInputsOff();

  Serial.println("Initialize servos");
  setupServos();
}

void setupServos()
{
  uint16_t baseSectorNumber = (PICO_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE) - SECTORS_TO_USE;
  pStorage = new FlashStorage(baseSectorNumber, SECTORS_TO_USE, (uint8_t*)"MLLSRS10");
  if (!pStorage->isValid())
  {
    ShowCriticalError("can't use flash storage");
  }

  for (int i = 0; i < NUM_SERVO_CONTROLLERS; i++)
  {
    uint8_t pins[3] = { (uint8_t)(0+i*3), (uint8_t)(1 + i * 3), (uint8_t)(2 + i * 3) };
    pConfigurator[i] = new ServoConfigurator(pStorage, 0, 3, pins);
  }
}

void loop()
{
  pLEDReceiver->loop();

  for (int i = 0; i < NUM_SERVO_CONTROLLERS; i++)
  {
    pConfigurator[i]->process(ledData + i*3);
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
        Serial.println("Enter setup mode");
        isInSetup = true;
        turnInputsOff();
      }
      // use fastled to set the HSV value of led[0]
      CHSV HSV;
      if (HSV.hue != hue)
      {
        HSV.hue = hue;
        Serial.printf("Servo %d in setup mode, position = %f hue = %d\r\n", i, pConfigurator[i]->getPercentage(), HSV.hue);
        HSV.sat = 255;
        HSV.val = 100;
        leds[0] = HSV;
      }
      break;
    }
  }
  if (isInSetup && !inSetupNow)
  {
    Serial.println("Leave setup mode");
    isInSetup = false;
    leds[0].setRGB(0, 0, 0);
    turnInputsOff();
  }
  if (!isInSetup) setSignal(pLEDReceiver->getState());
  MobaLedLib.Update();
  FastLED.show();                       // Show the LEDs (send the leds[] array to the LED stripe)
}

void ShowCriticalError(const char* message)
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
