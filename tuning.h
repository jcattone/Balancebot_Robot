#pragma once
#include "EspNowRemote.h"
#include "EspNowRemote_Events.h"
#include "types.h"

// Updated by the callbacks from the remote instance
extern EspNowRemote::joystick_state_t g_joystick_state;
extern EspNowRemote::joystick_analog_state_t g_joystick_analog_state;
extern uint8_t g_last_message_type;
extern unsigned long last_hid_input_timestamp;

// Updated by updateOrientation()
extern OrientationAngles gOrientation;

// How often do the control functions run?
constexpr int g_sample_freq = 200;  // Hz
constexpr int g_update_freq = 200;  // Hz
constexpr int g_sample_period = 1000 / g_sample_freq;  // ms
constexpr int g_update_period = 1000 / g_update_freq;  // ms

constexpr int PWM_PRECISION = 12;
constexpr int PWM_MAX = ((1 << PWM_PRECISION) - 1);
constexpr int PWM_SCALE_FROM_8BIT = (1 << (PWM_PRECISION - 8));

// Tuning parameters
extern bool gImuFault;
extern float gMahonyKp;
extern float gMahonyKi;
#ifdef ADAPTIVE_FUSION_KI
extern float gAccelPeakDecay;
#endif
extern float gMahonyKiScale;
extern eHBridgeIdleMode gHBridgeIdleMode;
extern float gDeadZone;
extern float gPwmMinDuty;
extern float gPwmMaxDuty;
extern float gPwmDutyAccumulator;
extern float gPwmDutyAppliedMagnitude;
extern int gPwmFreq;
extern float gPitchTrim;
extern float gYawTrim;
extern float gThrottleBias;
extern float gSteeringBias;
extern float gMaxThrottleBias;
extern float gVoltage;
extern float gVoltagePercent;
extern float gPitchPidKp;
extern float gPitchPidKi;
extern float gPitchPidKd;
extern float gDIIRWeight;
extern float gSpeedIIRWeight;
extern float gVelocityPidKp;
extern float gVelocityPidKi;
extern float gVelocityPidKd;
extern float gVelocityDIIRWeight;
extern bool gInvertVelocityPid;
extern float gMotorFilter;

// Control / interaction methods to be called from Loop()
void handleInput();
void updateRemoteDisplay(EspNowRemote::RmtBase* remote);