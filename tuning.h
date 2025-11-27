#pragma once
#include "EspNowRemote.h"
#include "EspNowRemote_Events.h"
#include "types.h"
//#include "motor_control.h" // caused circular inclusion

// Updated by the callbacks from the remote instance
extern EspNowRemote::joystick_state_t g_joystick_state;
extern EspNowRemote::joystick_analog_state_t g_joystick_analog_state;
extern uint8_t g_last_message_type;
extern unsigned long last_hid_input_timestamp;


// How often do the control functions run?
constexpr int g_sample_freq = 200;                     // Hz
constexpr int g_update_freq = 200;                     // Hz
constexpr int g_sample_period = 1000 / g_sample_freq;  // ms
constexpr int g_update_period = 1000 / g_update_freq;  // ms

constexpr int PWM_PRECISION = 12;
constexpr int PWM_MAX = ((1 << PWM_PRECISION) - 1);
constexpr int PWM_SCALE_FROM_8BIT = (1 << (PWM_PRECISION - 8));

// Tuning parameters
#ifdef ADAPTIVE_FUSION_KI
extern float gAccelPeakDecay;
#endif


struct ImuParams {
  int sampleFreq;
  float kp;
  float ki;
  float kiScale;
};

struct ImuState {
  bool fault;
  OrientationAngles orientation;
};

// DriveParams (specific to the 2-wheel main drive)
struct DriveParams {
  eHBridgeIdleMode idleMode;
  float deadZone;
  float pwmMinDuty;
  float pwmMaxDuty;
  float pitchTrim;
  float yawTrim;
  float maxThrottleBias;
  float motorFilterWeight;

  float throttleBias;
  float steeringBias;
};

struct DriveState {
  float pwmDutyAccumulator;      // gPwmDutyAccumulator;
  float pwmDutyAppliedMagnitude; // gPwmDutyAppliedMagnitude;
  int pwmFreq;                   // gPwmFreq;
};

// BatteryState
extern float gVoltage;
extern float gVoltagePercent;

// PitchPidParams
extern float gPitchPidKp;
extern float gPitchPidKi;
extern float gPitchPidKd;
extern float gDIIRWeight;

// SpeedPidParams
extern float gSpeedIIRWeight;
extern float gVelocityPidKp;
extern float gVelocityPidKi;
extern float gVelocityPidKd;
extern float gVelocityDIIRWeight;
extern bool gInvertVelocityPid;

struct BalanceState {
  ImuParams imuParams;
  ImuState imuState;
  DriveParams driveParams;
  DriveState driveState;
};

// Control / interaction methods to be called from Loop()
void initInput(EspNowRemote::RmtBase* remote);
void handleInput(BalanceState* state);
void updateRemoteDisplay(EspNowRemote::RmtBase* remote, BalanceState* state);