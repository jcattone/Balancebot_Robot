#pragma once
#include "tuning.h"
#include "types.h"

void initMotors(DriveParams* params, DriveState* state, PitchPIDParams* pitchParams, VelocityPIDParams* velocityParams);
void updateMotors(BalanceState* state);
