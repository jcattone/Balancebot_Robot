#include "tuning.h"
#include "types.h"
#include "pins.h"

using namespace EspNowRemote;

eConfigMode gCurrentConfigMode = eDefaultConfigMode;

joystick_state_t g_joystick_state = {};
joystick_analog_state_t g_joystick_analog_state = {};
uint8_t g_last_message_type = MSGTYPE_UNKNOWN;

OrientationAngles gOrientation;

unsigned long last_hid_input_timestamp = 0;

bool gImuFault = false;

// Initialize the IMU filter weights:
// Kp ~= trust accel data (gravity vector) to correct gyro drift
//   High Kp = correct quickly, but linear acceleration (i.e., translation) can be misread as orientation change
//   Low Kp = trust the gyro more, can be sluggish to respond
//   i.e., choose lowest Kp with acceptable linear acceleration characteristics
// Ki ~= correction speed for gyro bias (from accel gravity reference)
//   High Ki = aggressively remove bias, but can become sluggish.  In practice, overshoot is observed.
//   Low Ki = may allow bias to persistently affect the output
// In practice, I'm seeing eyeball-reasonable results with Kp=10..25, and Ki=0..5
//   (10.0 & 3.0 seems fastish and low noise)
//   (32/2 seems snappy)
// Theory-to-practice...
//   The jumpiness was indeed exacerbated by a high IMU Kp, presumably
//   by allowing large pitch deltas due to linear acceleration.
//   By reducing Kp, we work more from the integrated gyro
//   (i.e., integrated rotation delta), rather than from the
//   accel (gravity) data, which is perturbed by linear motion.
//   A small Ki weight still corrects integration error from the
//   accel (gravity) vector.
float gMahonyKp = 3.7f;
float gMahonyKi = 0.3f;
#ifdef ADAPTIVE_FUSION_KI
float gAccelPeakDecay = 0.990f;
#endif
float gMahonyKiScale = 1.0f;


eHBridgeIdleMode gHBridgeIdleMode = eBraking;
// The following three are floats (instead of int) to avoid runtime conversion
// to float when comparing to gPwmDutyAccumulator / gPwmDutyAppliedMagnitude
float gDeadZone = 1;
float gPwmMinDuty = 23;
float gPwmMaxDuty = 240;                // 160 is nominal ((2 * 4.2) - 0.7) * (160 / 255) ~= 5V (2x 18650 - diode drop * pwm ratio = rated TT motor voltage)
                                        // but a bit more oomph helps recovery
float gPwmDutyAccumulator = 0.0f;       // The raw PWM target
float gPwmDutyAppliedMagnitude = 0.0f;  // gPwmDutyAccumulator, but scaled to exclude the dead zone and map into the min/max PWM range
int gPwmFreq = 16000;
float gPitchTrim = -2.8f;  // The IMU tends to shift, and the CoM isn't quite over the axle, so -2.6..-4.0 seems to be the sweet spot

// The fraction shifted from one motor to the other
// TODO: Implement PID control for this, driven by encoder input?
float gYawTrim = 0.0f;

float gThrottleBias = 0.0f;
float gSteeringBias = 0.0f;
float gMaxThrottleBias = (160.0f / 255.0f);  // Target roughly 5V max throttle (with headroom for correction)

float gVoltage = 0.0f;
float gVoltagePercent = 0.0f;

// Or 0.16 - 0.24 seems to work well with Kd=~0.06-0.07
// Lower values help with smooth stability at low deflection, but don't respond quickly enough to correct for nudges

// Originally: pitch 0.3/0/0.05 (IIR .16), vel 0.14/0/0 (IIR .8) w/ intrinsic 60x
float gPitchPidKp = 0.46f;
float gPitchPidKi = 0.0f;
float gPitchPidKd = 0.06f;

float gDIIRWeight = 1.0f;

float gSpeedIIRWeight = 0.020;  // Was 0.1.  .01 weights the current speed; the IIR decays to < 2% in one second
float gVelocityPidKp = 6.0f;
float gVelocityPidKi = 2.0f;  // Experimental - overcomes carpet 'stuck', but exacerbates over-acceleration
float gVelocityPidKd = 0.0f;
float gVelocityDIIRWeight = 0.50f;
bool gInvertVelocityPid = true;

float gMotorFilter = 0.908f;

void adjustPidK(float* f, float delta) {
  *f += delta;
  if (*f < 0.0f)
    *f = 0.0f;
}
#define SCREEN_CHAR_WIDTH 21 // 5+1 pixel font width
void handleInput() {
  static unsigned long lastUpdate = 0;
  static bool suppressModeChange = false;
  // unsigned int now = millis();
  // if (now - lastUpdate < 20)
  //   return;
  // lastUpdate = now;

  static unsigned long last_hid_message_processed = 0;
  if (last_hid_message_processed == last_hid_input_timestamp)
    return;
  last_hid_message_processed = last_hid_input_timestamp;

  // Process joystick events if there's an unprocessed count.
  if (g_last_message_type == MSGTYPE_RMT_JOYSTICK_ANALOG) {
    // -1.0 .. +1.0
    gSteeringBias = g_joystick_analog_state.x_axis;
    // Remap to +/- 20 degrees
    gThrottleBias = g_joystick_analog_state.y_axis;
  } else if (g_last_message_type == MSGTYPE_RMT_JOYSTICK && g_joystick_state.count > 0) {
    // Extract the joystick state
    bool up = (g_joystick_state.joystick_direction & JOYSTICK_UP) != 0;
    bool down = (g_joystick_state.joystick_direction & JOYSTICK_DOWN) != 0;
    bool left = (g_joystick_state.joystick_direction & JOYSTICK_LEFT) != 0;
    bool right = (g_joystick_state.joystick_direction & JOYSTICK_RIGHT) != 0;
    int count = std::max((uint8_t)1, g_joystick_state.count);

    // reset (consume) the count, so that events are not reprocessed
    g_joystick_state.count = 0;

    // For ergonomics, config mode is single-step (one step per tap).
    // Once changed, wait for the joystick to return to a
    // vertically-neutral position before allowing another tap.
    if (suppressModeChange) {
      // If neither up nor down, we're vertically neurtal - we can
      // start watching for another mode change.
      if (!(up || down))
        suppressModeChange = false;
    } else {
      // Roll the config mode, but then suppress additional mode
      // changes to prevent rapidly scrolling through config options.
      if (up) {
        if (gCurrentConfigMode == 0)
          gCurrentConfigMode = (eConfigMode)(eMaxConfigMode);
        gCurrentConfigMode = (eConfigMode)(gCurrentConfigMode - 1);
        suppressModeChange = true;
      } else if (down) {
        gCurrentConfigMode = (eConfigMode)(gCurrentConfigMode + 1);
        if (gCurrentConfigMode == eMaxConfigMode)
          gCurrentConfigMode = (eConfigMode)0;
        suppressModeChange = true;
      }
    }

    // Right/left apply a change to the current mode, scaled by the count when applicable.
    switch (gCurrentConfigMode) {
      case eFusionKp:
        if (right) gMahonyKp = min(100.0f, gMahonyKp + 0.1f * count);
        else if (left) gMahonyKp = max(0.0f, gMahonyKp - 0.1f * count);
        break;
      case eFusionKi:
        if (right) gMahonyKi = min(100.0f, gMahonyKi + 0.01f * count);
        else if (left) gMahonyKi = max(0.0f, gMahonyKi - 0.01f * count);
        break;
#ifdef ADAPTIVE_FUSION_KI
      case eAccelPeakDecay:
        if (right) gAccelPeakDecay = min(1.0f, gAccelPeakDecay + 0.001f * count);
        else if (left) gAccelPeakDecay = max(0.0f, gAccelPeakDecay - 0.001f * count);
        break;
#endif
      case ePitchTrim:
        if (right)
          gPitchTrim = min(10.0f, gPitchTrim + 0.1f * count);
        else if (left)
          gPitchTrim = max(-10.0f, gPitchTrim - 0.1f * count);
        break;
      case ePwmMin:
        if (right) gPwmMinDuty = min(gPwmMaxDuty, gPwmMinDuty + (float)count);
        else if (left) gPwmMinDuty = max(0.0f, gPwmMinDuty - (float)count);
        break;
      case ePwmMax:
        if (right) gPwmMaxDuty = min(255.0f, gPwmMaxDuty + (float)count);
        else if (left) gPwmMaxDuty = max(gPwmMinDuty, gPwmMaxDuty - (float)count);
        break;
      case ePwmFreq:
        {
          for (int i = 0; i < count; ++i) {
            if (right) gPwmFreq = min(19000, max(gPwmFreq + 1, (int)(gPwmFreq * 1.1f)));
            else if (left) gPwmFreq = max(10, min(gPwmFreq - 1, (int)(gPwmFreq / 1.1f)));
          }

          if (right || left) {
            analogWriteFrequency(MOTORA_PIN_1, gPwmFreq);
            analogWriteFrequency(MOTORA_PIN_2, gPwmFreq);
            analogWriteFrequency(MOTORB_PIN_1, gPwmFreq);
            analogWriteFrequency(MOTORB_PIN_2, gPwmFreq);
          }
        }
        break;
      case eMotorSmoothing:
        if (right)
          gMotorFilter = min(1.0f, gMotorFilter + 0.001f * count);
        else if (left)
          gMotorFilter = max(0.0f, gMotorFilter - 0.001f * count);
        break;
      case eDIIRWeight:
        if (right)
          gDIIRWeight = min(1.0f, gDIIRWeight + 0.01f * count);
        else if (left)
          gDIIRWeight = max(0.0f, gDIIRWeight - 0.01f * count);
        break;
      case eVelocityDIIRWeight:
        if (right)
          gVelocityDIIRWeight = min(1.0f, gVelocityDIIRWeight + 0.01f * count);
        else if (left)
          gVelocityDIIRWeight = max(0.0f, gVelocityDIIRWeight - 0.01f * count);
        break;
      case eVelocityCurSpeedIIRWeight:
        if (right)
          gSpeedIIRWeight = min(1.0f, gSpeedIIRWeight + 0.001f * count);
        else if (left)
          gSpeedIIRWeight = max(0.0f, gSpeedIIRWeight - 0.001f * count);
        break;
      case eMaxThrottle:
        if (right)
          gMaxThrottleBias = min(1.0f, gMaxThrottleBias + 0.01f * count);
        else if (left)
          gMaxThrottleBias = max(0.0f, gMaxThrottleBias - 0.01f * count);
        break;
      case eDeadzone:
        if (right)
          gDeadZone = min(255.0f, gDeadZone + (float)count);
        else if (left)
          gDeadZone = max(0.0f, gDeadZone - (float)count);
        break;
      case eHBridgeIdle:
        if (right)
          gHBridgeIdleMode = eCoasting;
        else if (left)
          gHBridgeIdleMode = eBraking;
        break;
      case ePitchPidKp:
        adjustPidK(&gPitchPidKp, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case ePitchPidKi:
        adjustPidK(&gPitchPidKi, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case ePitchPidKd:
        adjustPidK(&gPitchPidKd, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eVelocityPidKp:
        adjustPidK(&gVelocityPidKp, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eVelocityPidKi:
        adjustPidK(&gVelocityPidKi, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eVelocityPidKd:
        adjustPidK(&gVelocityPidKd, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eInvertVelocityPid:
        if (right)
          gInvertVelocityPid = true;
        else if (left)
          gInvertVelocityPid = false;
        break;
      case eReset:
        if (count > 5) {
          gHBridgeIdleMode = eBraking;
          gDeadZone = 1;
          gPwmMinDuty = 23;
          gPwmMaxDuty = 200;
          gPwmDutyAccumulator = 0.0f;
          gPwmDutyAppliedMagnitude = 0.0f;
          gPwmFreq = 2 * g_update_freq;
          gPitchTrim = -2.6f;
          gYawTrim = 0.0f;
          gThrottleBias = 0.0f;
          gSteeringBias = 0.0f;
          gPitchPidKp = 0.0f;
          gPitchPidKi = 0.0f;
          gPitchPidKd = 0.0f;
          gMotorFilter = 1.0f;
          gDIIRWeight = 1.0f;
          gVelocityDIIRWeight = 1.0f;
          gVelocityPidKp = 0.0f;
          gVelocityPidKi = 0.0f;
          gVelocityPidKd = 0.0f;
        }
        break;
      case eIMUDisplay:
      default:
        break;
    }
  }
}


// TODO: ABort immediately if not connected, and always update when reconnected
void updateRemoteDisplay(RmtBase* remote) {
  unsigned int now = millis();

  static int frameCount = 0;
  static int lastFrameRate = 0;
  static unsigned long lastFrameStart = 0;
  ++frameCount;
  if (now - lastFrameStart >= 1000) {
    lastFrameRate = frameCount;
    frameCount = 0;
    lastFrameStart = now;
  }

  static unsigned long lastUpdate = 0;
  if (now - lastUpdate < 100)
    return;
  lastUpdate = now;

  char detailString[SCREEN_CHAR_WIDTH + 1];
  switch (gCurrentConfigMode) {
    case eReset:
      snprintf(detailString, sizeof(detailString), "Reset (hold)");
      break;
    case eIMUDisplay:
      snprintf(detailString, sizeof(detailString), "IMU: R%+04.1f P%+04.1f", gOrientation.roll, gOrientation.pitch);
      break;
    case eFusionKp:
      snprintf(detailString, sizeof(detailString), "Kp=*%.1f Ki=%.2f", gMahonyKp, gMahonyKi);
      break;
    case eFusionKi:
      snprintf(detailString, sizeof(detailString), "Kp=%.1f Ki=*%.2f", gMahonyKp, gMahonyKi);
      break;
#ifdef ADAPTIVE_FUSION_KI
    case eAccelPeakDecay:
      snprintf(detailString, sizeof(detailString), "KiDecay=*%.1f%%", gAccelPeakDecay * 100.0f);
      break;
#endif
    case ePwmMin:
      snprintf(detailString, sizeof(detailString), "pwm=*%.0f-%.0f (%.1f)", gPwmMinDuty, gPwmMaxDuty, gPwmDutyAccumulator);
      break;
    case ePwmMax:
      snprintf(detailString, sizeof(detailString), "pwm=%.0f-*%.0f (%.1f)", gPwmMinDuty, gPwmMaxDuty, gPwmDutyAccumulator);
      break;
    case ePwmFreq:
      snprintf(detailString, sizeof(detailString), "pwmFreq=%d (%.1f)", gPwmFreq, gPwmDutyAccumulator);
      break;
    case eMotorSmoothing:
      snprintf(detailString, sizeof(detailString), "mSmooth: %.3f", gMotorFilter);
      break;
    case ePitchTrim:
      snprintf(detailString, sizeof(detailString), "Trim: %.2f", gPitchTrim);
      break;
    case eDIIRWeight:
      snprintf(detailString, sizeof(detailString), "P_IIR: %.2f", gDIIRWeight);
      break;
    case eVelocityDIIRWeight:
      snprintf(detailString, sizeof(detailString), "V_IIR: %.2f", gVelocityDIIRWeight);
      break;
    case eVelocityCurSpeedIIRWeight:
      snprintf(detailString, sizeof(detailString), "SP_IIR: %.3f", gSpeedIIRWeight);
      break;
    case eMaxThrottle:
      snprintf(detailString, sizeof(detailString), "Throttle: %.0f%%", gMaxThrottleBias * 100.0f);
      break;
    case eDeadzone:
      snprintf(detailString, sizeof(detailString), "mDead: %.0f", gDeadZone);
      break;
    case eHBridgeIdle:
      if (gHBridgeIdleMode == eBraking)
        snprintf(detailString, sizeof(detailString), "Idle=[B] /  C ");
      else
        snprintf(detailString, sizeof(detailString), "Idle= B  / [C]");
      break;
    case ePitchPidKp:
      snprintf(detailString, sizeof(detailString), "pPID *%.2f %.2f %.2f", gPitchPidKp, gPitchPidKi, gPitchPidKd);
      break;
    case ePitchPidKi:
      snprintf(detailString, sizeof(detailString), "pPID %.2f *%.2f %.2f", gPitchPidKp, gPitchPidKi, gPitchPidKd);
      break;
    case ePitchPidKd:
      snprintf(detailString, sizeof(detailString), "pPID %.2f %.2f *%.2f", gPitchPidKp, gPitchPidKi, gPitchPidKd);
      break;
    case eVelocityPidKp:
      snprintf(detailString, sizeof(detailString), "vPID *%.2f %.2f %.2f", gVelocityPidKp, gVelocityPidKi, gVelocityPidKd);
      break;
    case eVelocityPidKi:
      snprintf(detailString, sizeof(detailString), "vPID %.2f *%.2f %.2f", gVelocityPidKp, gVelocityPidKi, gVelocityPidKd);
      break;
    case eVelocityPidKd:
      snprintf(detailString, sizeof(detailString), "vPID %.2f %.2f *%.2f", gVelocityPidKp, gVelocityPidKi, gVelocityPidKd);
      break;
    case eInvertVelocityPid:
      if (gInvertVelocityPid)
        snprintf(detailString, sizeof(detailString), "-VelPID= N  / [Y]");
      else
        snprintf(detailString, sizeof(detailString), "-VelPID=[N] /  Y ");
      break;
    case eBattery:
      snprintf(detailString, sizeof(detailString), "Batt: %.2f (%.0f%%)", gVoltage, gVoltagePercent);
      break;
  }

  // Send to the remote

  static unsigned long last_title_sent_millis = 0L;
  char titleString[SCREEN_CHAR_WIDTH] = { 0 };
  static char lastTitleBuf[SCREEN_CHAR_WIDTH + 1] = {};
  snprintf(titleString, sizeof(titleString), "Pit=%+04.1f PWM=%.1f", gOrientation.pitch, gPwmDutyAccumulator);  // lastFrameRate
  if (strcmp(titleString, lastTitleBuf) || now - last_title_sent_millis > 1000) {
    remote->Send(MSGTYPE_CTL_TITLE, reinterpret_cast<const uint8_t*>(titleString), SEND_NULLTERMINATED);
    strcpy(lastTitleBuf, titleString);
    last_title_sent_millis = now;
  }

  static unsigned long last_detail_sent_millis = 0L;
  static char lastDetailBuf[SCREEN_CHAR_WIDTH + 1] = {};
  if (strcmp(detailString, lastDetailBuf) || now - last_detail_sent_millis > 1000) {
    remote->Send(MSGTYPE_CTL_DETAIL, (uint8_t*)detailString, SEND_NULLTERMINATED);
    strcpy(lastDetailBuf, detailString);
    last_detail_sent_millis = now;
  }
}
