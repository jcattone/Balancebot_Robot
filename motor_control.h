#pragma once
#include "tuning.h"
#include "types.h"

void initMotors(DriveParams* params, DriveState* state, PitchPIDParams* pitchParams);
void updateMotors(BalanceState* state);
