/*
 * SPDX-FileCopyrightText: 2025-2026 Juergen Winkler <MobaLedLib@gmx.at>
 * SPDX-License-Identifier: CC-BY-NC-4.0
*/

#pragma once
#include "ServoController.h"

class StatefulServoController : public ServoController
{
public:
    StatefulServoController(uint8_t pin, uint16_t min, uint16_t max, int speed)
        : ServoController(pin, min, max, speed)
    {
        lastMoveTime = 0;
        lastLedValue = 0;
    }
    bool shouldSavePosition()
    {
        if (!isReady()) return false;
        if (isMoving())
        {
            lastMoveTime = millis();
            return false;
        }
        if (lastMoveTime == 0) return false;
        return (millis() - lastMoveTime) > 250;
    }
    bool shouldAutomaticallyDisable()
    {
        if (!isReady() || isMoving() || lastMoveTime == 0) return false;
        return (millis() - lastMoveTime) > 250;                           // change from 100 to 250, older slow servos need more time to reach the target position
    }

    void resetSavePosition()
    {
        lastMoveTime = 0;
    }
public:
    uint8_t buttonChanged;
    uint8_t lastLedValue;
    unsigned long lastMoveTime;
};