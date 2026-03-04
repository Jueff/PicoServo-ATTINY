#include "pico/multicore.h"
#include "MobaLedLib.h"
#include "PicoFlashStorage.h"

// cool tool: https://godbolt.org/

const uint DATA_IN_PIN = 15;
const uint DATA_OUT_PIN = 14;
const uint STATUSLED_PIN = 13;

const uint NUM_LEDS_TO_EMULATE = 2;
const uint NUM_LEDS_TO_SKIP = NUM_LEDS_TO_EMULATE - 1;

#include "LEDReceiver.h"
#include "ServoConfigurator.h" 

const char* bootMessage = "MobaLedLib Pico 6x-Servo ATTiny V0.01";

bool dataChanged = false;
LEDReceiver* pLEDReceiver;
#define NUM_LEDS 20  
CRGB leds[NUM_LEDS];           // Define the array of leds

uint8_t lastSignal = 0xff;
uint8_t ledData[NUM_LEDS_TO_EMULATE * 3];

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#define SECTORS_TO_USE 4
using namespace PicoFlashStorage;

FlashStorage* pStorage;

#define HB_INPUT 0
#define MAX_BRIGHT 20

MobaLedLib_Configuration()
{
  //BlueLight1(0, C3, HB_INPUT + 2)
  Blink3(0, C_BLUE, HB_INPUT + 0, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)
  Blink3(0, C_YELLOW, HB_INPUT + 1, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)
  Blink3(0, C_RED, HB_INPUT + 2, 0.5 Sek, 0.5 Sek, 5, MAX_BRIGHT, 0)
  APatternT1(0, 193, HB_INPUT + 3, 1, 5, MAX_BRIGHT, 0, PF_EASEINOUT, 1 Sec, 1)

  //Switchable_RGB_Heartbeat_Color(0, HB_INPUT + 3, 20, 70, 170, 1000)

  EndCfg // End of the configuration
};

MobaLedLib_Create(leds); // Define the MobaLedLib instance

void turnInputsOff()
{
  for (int i = 0; i < 4; i++)
  {
    MobaLedLib.Set_Input(HB_INPUT + i, 0);
  }
  MobaLedLib.Update();
}

void setSignal(LEDReceiver::State signal)
{
  /*
    Error       = 0,
    DataMissing = 1,
    Offline     = 2,
    Online      = 3
  */
  uint8_t newSignal = (uint8_t)signal;
  if (newSignal > 3) return;
  if (newSignal == lastSignal) return;
  lastSignal = newSignal;
  Serial.printf("Set signal %d\r\n", newSignal);
  turnInputsOff();
  MobaLedLib.Set_Input(HB_INPUT + newSignal, 1);

}

void setup()
{
  Serial.begin(115200);
  // only enable for debugging purpose to see trace output of boot code
  while (!Serial) {}
  Serial.println(bootMessage);

 
  for (int i = 0; i < 30; i++)
  {
    pinMode(i, INPUT);
  }
  pinMode(29, OUTPUT);

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  Serial.println("Initialize LED Receiver");
  pLEDReceiver = new LEDReceiver(&ledData[0], NUM_LEDS_TO_EMULATE, NUM_LEDS_TO_SKIP, DATA_IN_PIN, DATA_OUT_PIN);
  Serial.println("Initialize FastLED");

  FastLED.addLeds<NEOPIXEL, STATUSLED_PIN>(leds, NUM_LEDS); // Initialize the FastLED library
  FastLED.addLeds<NEOPIXEL, 16>(leds, NUM_LEDS); // Initialize the FastLED library
  FastLED.setDither(DISABLE_DITHER);       // avoid sending slightly modified brightness values

  turnInputsOff();

  Serial.println("Initialize servos");
  setupServos();

  //multicore_launch_core1(core1_entry);
}

ServoConfigurator* pConfigurator[2];

void setupServos()
{
  uint16_t baseSectorNumber = (PICO_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE) - SECTORS_TO_USE;
  pStorage = new FlashStorage(baseSectorNumber, SECTORS_TO_USE, (uint8_t*)"MLLSRS10");
  if (!pStorage->isValid())
  {
    // todo error handling
    do
    {
      Serial.println("can't use flash storage");
      delay(1000);
    } while (true);
  }
  unsigned long lastAlive = millis();

  uint8_t pins1[3] = { 0, 1, 2 };
  pConfigurator[0] = new ServoConfigurator(pStorage, 0, 3, pins1);
  uint8_t pins2[3] = { 3, 4, 5 };
  pConfigurator[1] = new ServoConfigurator(pStorage, 3, 3, pins2);
}

void core1_entry()
{
  setupServos();

  while (true)
  {
    // print a alive message every 1 second
/*    if (millis() - lastAlive >= 1000)
    {
      lastAlive += 1000;
      Serial.println("core1 alive");
    }*/
    for (int i = 0; i < 2; i++)
    {
      pConfigurator[i]->process(ledData + i * 3);
    }
  }
}

void loop()
{
  pLEDReceiver->loop();
  for (int i = 0; i < 2; i++)
  {
    pConfigurator[i]->process(ledData + i*3);
  }
  bool inConfiguration = false;
  for (int i = 0; i < 2; i++)
  {
    if (pConfigurator[i]->isInSetup())
    {
      // use fastled to set the HSV value of led[0]
      CHSV HSV;
      HSV.hue = (uint8_t)((1.0 - pConfigurator[i]->getPercentage()) * 160);  // 0% = 160 (Blau), 100% = 0 (rot)
      //Serial.printf("Servo %d in setup mode, position = %f hue = %d\r\n", i, pConfigurator[i]->getPosition(), HSV.hue);
      HSV.sat = 255;
      HSV.val = 100;
      leds[i] = HSV;
      inConfiguration = true;
      turnInputsOff();
      break;
    }
  }
  if (!inConfiguration) setSignal(pLEDReceiver->getState());
  MobaLedLib.Update();
  FastLED.show();                       // Show the LEDs (send the leds[] array to the LED stripe)
}

