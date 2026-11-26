// MLLAdminCommunication.h - Administration und Identifikation ueber serielle Schnittstelle (nur ESP32 und Pico)
// Version 1.01 (2026-11-26): MLL_FW_TYPE als Makro, damit jedes Projekt seinen Typ melden kann
// Version 1.00 (2026-11-22): neu, vereint MLLCFG und MLLIDENT Protokolle unter dem gemeinsamen Prefix '~'
// Author: Juergen Winkler
//
// Serielles Protokoll (alle Zeilen beginnen mit '~'):
//
// --- Identifikation (MLLIDENT) --------------------------------------------------------
//   ~MLLIDENT 1   Blinken starten bzw. verlaengern (der PC schickt die Zeile jede Sekunde)  -> ~MLLIDENT ON  (nur beim Start)
//   ~MLLIDENT 0   Blinken beenden                                                         -> ~MLLIDENT OFF
// Kommt MLL_IDENT_TIMEOUT ms lang keine Zeile mehr (serielle Verbindung getrennt, Dialog geschlossen), endet das Blinken.
// Ein Neustart beendet es ebenfalls (der Zustand liegt nur im RAM).
//
// --- Typ und Version (MLLINFO) ---------------------------------------------------------
//   ~MLLINFO      Antwort: ~MLLINFO type=<MLL_FW_TYPE> platform=<MLL_PLATFORM> ver=<APP_VERSION>
// MLL_FW_TYPE wird vor dem Include in der jeweiligen .ino definiert (z.B. PICOSERVO, PICOSOUND, PICO2X3).
//
// ---------------------------------------------------------------------------------------

#ifndef __MLL_ADMIN_COMMUNICATION_H__
#define __MLL_ADMIN_COMMUNICATION_H__

#ifndef MLL_CFG_LINE_MAX
#define MLL_CFG_LINE_MAX     30
#endif

// Firmware-Typ, der mit ~MLLINFO gemeldet wird. Standardwert, falls die .ino nichts definiert.
#ifndef MLL_FW_TYPE
#define MLL_FW_TYPE "UNDEFINED"
#endif

// Plattform laut .desc (alle Module basieren auf der PicoSoundServo 512 Platine).
#ifndef MLL_PLATFORM
#define MLL_PLATFORM "MLL512V1"
#endif

//----------------------------------------
static void MllCommunicationError(const __FlashStringHelper* Reason)
{
  Serial.print(F("~MLLERR "));
  Serial.println(Reason);
}

#include "MLLIdentify.h"

//----------------------------------------
static void MllInfoReply()
//----------------------------------------
// Antwort auf '~MLLINFO': "~MLLINFO type=<MLL_FW_TYPE> platform=<MLL_PLATFORM> ver=<APP_VERSION>". Die Lichtmaschinen-Erkennung (webMLL) liest diese Zeile.
{
  Serial.print(F("~MLLINFO type=" MLL_FW_TYPE " platform=" MLL_PLATFORM " ver="));
  Serial.println(F(STRINGIFY(APP_VERSION)));
}

//----------------------------------------
bool MllAdminReceive(char c)
//----------------------------------------
// Zentrale Empfangsroutine fuer alle Admin-Kommandos (Prefix '~').
// Wird fuer jedes empfangene Zeichen aufgerufen.
// Gibt true zurueck, wenn das Zeichen zu einer '~'-Zeile gehoert.
// Verarbeitet sowohl MLLCFG als auch MLLIDENT Kommandos.
{
  if (c != '~') return false;
  static char Line[MLL_CFG_LINE_MAX];
  size_t n = 0;
  bool Over = false, Complete = false;
  unsigned long Deadline = millis() + 1000;
  while (millis() < Deadline)
     {
     if (!Serial.available()) continue;
     char x = Serial.read();
     Deadline = millis() + 1000;
     if (x == '\n') { Complete = true; break; }
     if (x == '\r') continue;
     if (n < sizeof(Line) - 1) Line[n++] = x;
     else Over = true;
     }
  Line[n] = 0;
  if (!Complete || Over)                      MllCommunicationError(F("line"));
  else if (strncmp(Line, "MLLIDENT ", 9) == 0) MllIdentProcessLine(Line + 9);  // MLLIDENT wird in MllIdentProcessLine verarbeitet
  else if (strcmp(Line, "MLLINFO") == 0)      MllInfoReply();                       // ~MLLINFO: Typ und Version des Moduls
  else                                        MllCommunicationError(F("cmd"));
  return true;
}
#endif // __MLL_ADMIN_COMMUNICATION_H__
