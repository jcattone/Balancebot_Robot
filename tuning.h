#pragma once
#include "EspNowRemote.h"
#include "EspNowRemote_Events.h"
#include "types.h"

// How often do the control functions run?
constexpr int UPDATE_FREQ = 200;                   // Hz
constexpr int UPDATE_PERIOD = 1000 / UPDATE_FREQ;  // ms

constexpr int PWM_PRECISION = 12;
constexpr int PWM_MAX = ((1 << PWM_PRECISION) - 1);
constexpr int PWM_SCALE_FROM_8BIT = (1 << (PWM_PRECISION - 8));

struct RemoteInput {
  EspNowRemote::joystick_state_t joystick_state;
  EspNowRemote::joystick_analog_state_t joystick_analog_state;
  uint8_t last_message_type;
  unsigned long last_hid_input_timestamp;
  eConfigMode current_config_mode;
};

struct ImuParams {
  int sampleFreq;
  float kp;
  float ki;
  float kiScale;
#ifdef ADAPTIVE_FUSION_KI
  float accelPeakDecay;
#endif
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

struct ImuConfig {
  ImuParams params;
  ImuState state;
};

struct MotorConfig {
  DriveParams driveParams;
  DriveState driveState;
  PitchPIDParams pitchPidParams;
  VelocityPIDParams velocityPidParams;
};

// Control / interaction methods to be called from Loop()
void initInput(EspNowRemote::RmtBase* remote, RemoteInput* remoteInput);
void handleInput(RemoteInput* remoteInput, MotorConfig* motor, ImuConfig* imu, BatteryState* battery);
void updateRemoteDisplay(EspNowRemote::RmtBase* remote, RemoteInput* ri, MotorConfig* motor, ImuConfig* imu, BatteryState* battery);