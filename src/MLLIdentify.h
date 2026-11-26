// MLLIdentify.h
// Version 1.00 (2026-11-22): Initial release
// Author: Juergen Winkler

#ifndef __MLL_IDENTIFY_H__
#define __MLL_IDENTIFY_H__

static bool mllIdentActive = false;
static unsigned long mllIdentLastCmd = 0;

#ifndef MLL_IDENT_TIMEOUT
#define MLL_IDENT_TIMEOUT 3000  // ms, time after which the identity blinking stops if no command is received 
#endif

//-----------------------------------------
static void MllIdentStart()
//-----------------------------------------
{
  if (mllIdentActive) return;
  mllIdentLastCmd = millis();
  mllIdentActive = true;
}

//-----------------------------------------
static void MllIdentStop()
//-----------------------------------------
{
  if (!mllIdentActive) return;
  mllIdentActive = false;
}

//-----------------------------------------
static void MllIdentProcessLine(char* Line)
//-----------------------------------------
// Verarbeitet MLLIDENT Kommandos: "MLLIDENT 1" startet Blinken, "MLLIDENT 0" stoppt
{
  if (strcmp(Line, "1") == 0) MllIdentStart();
  else if (strcmp(Line, "0") == 0) MllIdentStop();
  else MllCommunicationError(F("MLLIDENT"));
}

//-----------------------------------------
static void MllIdentUpdate()
//-----------------------------------------
{
  if (!mllIdentActive) return;
  unsigned long Now = millis();
  if (Now - mllIdentLastCmd > MLL_IDENT_TIMEOUT) { MllIdentStop(); return; }
}


#endif // __MLL_IDENTIFY_H__
