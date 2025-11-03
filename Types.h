#pragma once
#include <Arduino.h>


// With each update, the orientation will be captured in gOrientation
struct OrientationAngles {
  float pitch;
  float roll;
  float yaw;
} gOrientation;

// To support multiple config modes...
// TODO: Add the disply-only yaw/pitch/roll & diag for remote display, so that we can remove the SSD1306
enum eConfigMode {
  ePwmMin,
  ePwmMax,
  ePwmFreq,
  ePidKp,
  ePidKi,
  ePidKd,
  eHBridgeIdle,
  eFusionKp,
  eFusionKi,
  eIMUDisplay,
  eMaxConfigMode,
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