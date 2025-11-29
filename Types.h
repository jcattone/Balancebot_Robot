#pragma once
#include <Arduino.h>

// #define ADAPTIVE_FUSION_KI 1

struct OrientationAngles {
  float pitch;
  float roll;
  float yaw;
};

// To support multiple config modes...
// TODO: Add the disply-only yaw/pitch/roll & diag for remote display, so that we can remove the SSD1306
enum eConfigMode {
  ePitchTrim,

  ePwmMin,
  ePwmMax,
  ePwmFreq,

  ePitchPidKp,
  ePitchPidKi,
  ePitchPidKd,
  eDIIRWeight,

  eVelocityPidKp,
  eVelocityPidKi,
  eVelocityPidKd,
  eVelocityDIIRWeight,
  eVelocityCurSpeedIIRWeight,

  eMaxThrottle,

  eMotorSmoothing,
  eDeadzone,
  eHBridgeIdle,
  eFusionKp,
  eFusionKi,
#ifdef ADAPTIVE_FUSION_KI
  eAccelPeakDecay,
#endif
  eInvertVelocityPid,
  eIMUDisplay,
  eReset,
  eBattery,
  eMaxConfigMode,
  eDefaultConfigMode = ePitchTrim
};


enum eHBridgeIdleMode {
  eBraking,
  eCoasting,
  eIdleUnknown
};


enum RelativeDirection {
  eStopped,
  eForward,
  eBackward,
  eDirectionUnknown
};
