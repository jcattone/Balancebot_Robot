#pragma once
#include "tuning.h"
#include "types.h"

void initMotors(DriveParams* params, DriveState* state);
void updateMotors(BalanceState* state);
