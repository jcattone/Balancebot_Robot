#pragma once
#include "tuning.h"
#include "types.h"

void initMotors(MotorConfig* motorConfig);
void updateMotors(MotorConfig* motorConfig, ImuConfig* imuConfig);
