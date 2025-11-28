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
  float pwmDutyAccumulator;       // gPwmDutyAccumulator;
  float pwmDutyAppliedMagnitude;  // gPwmDutyAppliedMagnitude;
  int pwmFreq;                    // gPwmFreq;
};

// Originally: pitch 0.3/0/0.05 (IIR .16), vel 0.14/0/0 (IIR .8) w/ intrinsic 60x
struct PitchPIDParams {
  float kp;          // gPitchPidKp = 0.46f;
  float ki;          // gPitchPidKi = 0.0f;
  float kd;          // gPitchPidKd = 0.06f;
  float dIIRWeight;  // gPitchDIIRWeight = 1.0f;
};

struct VelocityPIDParams {
  float inputIIRWeight;
  float kp;
  float ki;
  float kd;
  float dIIRWeight;
  bool invertFeedback;
};

struct BatteryState {
  float voltage;
  float voltagePercent;
};

struct BalanceState {
  ImuParams imuParams;
  ImuState imuState;
  DriveParams driveParams;
  DriveState driveState;
  PitchPIDParams pitchPidParams;
  VelocityPIDParams velocityPidParams;
  BatteryState batteryState;
};

// Control / interaction methods to be called from Loop()
void initInput(EspNowRemote::RmtBase* remote);
void handleInput(BalanceState* state);
void updateRemoteDisplay(EspNowRemote::RmtBase* remote, BalanceState* state);