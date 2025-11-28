#pragma once
#include "tuning.h"

void initBatterySense(BatteryState* state);
void updateBatterySense(BatteryState* batteryState, DriveParams* driveParams);
