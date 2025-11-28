#include "tuning.h"
#include "types.h"
#include "pins.h"

using namespace EspNowRemote;

eConfigMode gCurrentConfigMode = eDefaultConfigMode;

joystick_state_t g_joystick_state = {};
joystick_analog_state_t g_joystick_analog_state = {};
uint8_t g_last_message_type = MSGTYPE_UNKNOWN;

unsigned long last_hid_input_timestamp = 0;

#ifdef ADAPTIVE_FUSION_KI
float gAccelPeakDecay = 0.990f;
#endif

float gVoltage = 0.0f;
float gVoltagePercent = 0.0f;


bool OnControllerMessage(uint8_t msg_type, const uint8_t* data, int data_len);

void initInput(EspNowRemote::RmtBase* newRemote) {
  // To use lambdas as callbacks, we can't use a capture expression.
  // Instead, explicitly capture the remote in the parent scope.
  static EspNowRemote::RmtBase* capturedRemote;
  capturedRemote = newRemote;

  // Forward esp wifi events to the remote
  esp_now_register_send_cb([](const esp_now_send_info_t* tx_info, esp_now_send_status_t send_status) {
    capturedRemote->HandleDataSent(tx_info, send_status);
  });
  esp_now_register_recv_cb([](const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) IRAM_ATTR {
    capturedRemote->HandleDataReceived(esp_now_info, data, data_len);
  });

  capturedRemote->Setup();
  capturedRemote->RegisterMsgHandler(OnControllerMessage);
  capturedRemote->Start();
}

// When the remote controller wants to raise a message, it will be published here.
bool OnControllerMessage(uint8_t msg_type, const uint8_t* data, int data_len) {
  switch (msg_type) {
    case EspNowRemote::MSGTYPE_SELF_STATUS:
      // Loopback status from the local remote instance... not from the controller.
      // memcpy(textBuffer[1], data, data_len);
      // textBuffer[1][data_len] = 0;
#ifdef SERIAL_DIAG
      Serial.println((const char*)data);
#endif
      break;

    case MSGTYPE_RMT_JOYSTICK:
      memcpy(&g_joystick_state, data, data_len);
      g_last_message_type = msg_type;
      last_hid_input_timestamp = micros();
      break;

    case MSGTYPE_RMT_JOYSTICK_ANALOG:
      memcpy(&g_joystick_analog_state, data, data_len);
      g_last_message_type = msg_type;
      last_hid_input_timestamp = micros();
      break;
  }
  return true;
}

void adjustPidK(float* f, float delta) {
  *f += delta;
  if (*f < 0.0f)
    *f = 0.0f;
}
#define SCREEN_CHAR_WIDTH 21  // 5+1 pixel font width
void handleInput(BalanceState* state) {
  static unsigned long lastUpdate = 0;
  static bool suppressModeChange = false;
  // unsigned int now = millis();
  // if (now - lastUpdate < 20)
  //   return;
  // lastUpdate = now;

  auto& dp = state->driveParams;
  auto& ds = state->driveState;
  auto& pp = state->pitchPidParams;
  auto& vp = state->velocityPidParams;

  static unsigned long last_hid_message_processed = 0;
  if (last_hid_message_processed == last_hid_input_timestamp)
    return;
  last_hid_message_processed = last_hid_input_timestamp;

  // Process joystick events if there's an unprocessed count.
  if (g_last_message_type == MSGTYPE_RMT_JOYSTICK_ANALOG) {
    // -1.0 .. +1.0
    dp.steeringBias = g_joystick_analog_state.x_axis;
    // Remap to +/- 20 degrees
    dp.throttleBias = g_joystick_analog_state.y_axis;
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
        {
          auto& kp = state->imuParams.kp;
          if (right) kp = min(100.0f, kp + 0.1f * count);
          else if (left) kp = max(0.0f, kp - 0.1f * count);
          break;
        }
      case eFusionKi:
        {
          auto& ki = state->imuParams.ki;
          if (right) ki = min(100.0f, ki + 0.01f * count);
          else if (left) ki = max(0.0f, ki - 0.01f * count);
          break;
        }
#ifdef ADAPTIVE_FUSION_KI
      case eAccelPeakDecay:
        if (right) gAccelPeakDecay = min(1.0f, gAccelPeakDecay + 0.001f * count);
        else if (left) gAccelPeakDecay = max(0.0f, gAccelPeakDecay - 0.001f * count);
        break;
#endif
      case ePitchTrim:
        if (right)
          dp.pitchTrim = min(10.0f, dp.pitchTrim + 0.1f * count);
        else if (left)
          dp.pitchTrim = max(-10.0f, dp.pitchTrim - 0.1f * count);
        break;
      case ePwmMin:
        if (right) dp.pwmMinDuty = min(dp.pwmMaxDuty, dp.pwmMinDuty + (float)count);
        else if (left) dp.pwmMinDuty = max(0.0f, dp.pwmMinDuty - (float)count);
        break;
      case ePwmMax:
        if (right) dp.pwmMaxDuty = min(255.0f, dp.pwmMaxDuty + (float)count);
        else if (left) dp.pwmMaxDuty = max(dp.pwmMinDuty, dp.pwmMaxDuty - (float)count);
        break;
      case ePwmFreq:
        {
          for (int i = 0; i < count; ++i) {
            if (right) ds.pwmFreq = min(19000, max(ds.pwmFreq + 1, (int)(ds.pwmFreq * 1.1f)));
            else if (left) ds.pwmFreq = max(10, min(ds.pwmFreq - 1, (int)(ds.pwmFreq / 1.1f)));
          }

          if (right || left) {
            analogWriteFrequency(MOTORA_PIN_1, ds.pwmFreq);
            analogWriteFrequency(MOTORA_PIN_2, ds.pwmFreq);
            analogWriteFrequency(MOTORB_PIN_1, ds.pwmFreq);
            analogWriteFrequency(MOTORB_PIN_2, ds.pwmFreq);
          }
        }
        break;
      case eMotorSmoothing:
        if (right)
          dp.motorFilterWeight = min(1.0f, dp.motorFilterWeight + 0.001f * count);
        else if (left)
          dp.motorFilterWeight = max(0.0f, dp.motorFilterWeight - 0.001f * count);
        break;
      case eDIIRWeight:
        if (right)
          pp.dIIRWeight = min(1.0f, pp.dIIRWeight + 0.01f * count);
        else if (left)
          pp.dIIRWeight = max(0.0f, pp.dIIRWeight - 0.01f * count);
        break;
      case eVelocityDIIRWeight:
        if (right)
          vp.dIIRWeight = min(1.0f, vp.dIIRWeight + 0.01f * count);
        else if (left)
          vp.dIIRWeight = max(0.0f, vp.dIIRWeight - 0.01f * count);
        break;
      case eVelocityCurSpeedIIRWeight:
        if (right)
          vp.inputIIRWeight = min(1.0f, vp.inputIIRWeight + 0.001f * count);
        else if (left)
          vp.inputIIRWeight = max(0.0f, vp.inputIIRWeight - 0.001f * count);
        break;
      case eMaxThrottle:
        if (right)
          dp.maxThrottleBias = min(1.0f, dp.maxThrottleBias + 0.01f * count);
        else if (left)
          dp.maxThrottleBias = max(0.0f, dp.maxThrottleBias - 0.01f * count);
        break;
      case eDeadzone:
        if (right)
          dp.deadZone = min(255.0f, dp.deadZone + (float)count);
        else if (left)
          dp.deadZone = max(0.0f, dp.deadZone - (float)count);
        break;
      case eHBridgeIdle:
        if (right)
          dp.idleMode = eCoasting;
        else if (left)
          dp.idleMode = eBraking;
        break;
      case ePitchPidKp:
        adjustPidK(&pp.kp, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case ePitchPidKi:
        adjustPidK(&pp.ki, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case ePitchPidKd:
        adjustPidK(&pp.kd, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eVelocityPidKp:
        adjustPidK(&vp.kp, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eVelocityPidKi:
        adjustPidK(&vp.ki, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eVelocityPidKd:
        adjustPidK(&vp.kd, right ? 0.01f * count : (left ? -0.01f * count : 0.0f));
        break;
      case eInvertVelocityPid:
        if (right)
          vp.invertFeedback = true;
        else if (left)
          vp.invertFeedback = false;
        break;
      case eReset:
        if (count > 5) {
          dp.idleMode = eBraking;
          dp.deadZone = 1;
          dp.pwmMinDuty = 23;
          dp.pwmMaxDuty = 200;
          dp.pitchTrim = -2.6f;
          dp.yawTrim = 0.0f;
          dp.throttleBias = 0.0f;
          dp.steeringBias = 0.0f;
          dp.motorFilterWeight = 1.0f;

          ds.pwmDutyAccumulator = 0.0f;
          ds.pwmDutyAppliedMagnitude = 0.0f;
          ds.pwmFreq = 2 * g_update_freq;

          pp.kp = 0.0f;
          pp.ki = 0.0f;
          pp.kd = 0.0f;
          pp.dIIRWeight = 1.0f;

          vp.dIIRWeight = 1.0f;
          vp.kp = 0.0f;
          vp.ki = 0.0f;
          vp.kd = 0.0f;
        }
        break;
      case eIMUDisplay:
      default:
        break;
    }
  }
}

// TODO: Abort immediately if not connected, and always update when reconnected
void updateRemoteDisplay(RmtBase* remote, BalanceState* state) {
  unsigned int now = millis();
  auto& dp = state->driveParams;
  auto& ds = state->driveState;
  auto& pp = state->pitchPidParams;
  auto& vp = state->velocityPidParams;

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
      snprintf(detailString, sizeof(detailString), "IMU: R%+04.1f P%+04.1f", state->imuState.orientation.roll, state->imuState.orientation.pitch);
      break;
    case eFusionKp:
      snprintf(detailString, sizeof(detailString), "Kp=*%.1f Ki=%.2f", state->imuParams.kp, state->imuParams.ki);
      break;
    case eFusionKi:
      snprintf(detailString, sizeof(detailString), "Kp=%.1f Ki=*%.2f", state->imuParams.kp, state->imuParams.ki);
      break;
#ifdef ADAPTIVE_FUSION_KI
    case eAccelPeakDecay:
      snprintf(detailString, sizeof(detailString), "KiDecay=*%.1f%%", gAccelPeakDecay * 100.0f);
      break;
#endif
    case ePwmMin:
      snprintf(detailString, sizeof(detailString), "pwm=*%.0f-%.0f (%.1f)", dp.pwmMinDuty, dp.pwmMaxDuty, ds.pwmDutyAccumulator);
      break;
    case ePwmMax:
      snprintf(detailString, sizeof(detailString), "pwm=%.0f-*%.0f (%.1f)", dp.pwmMinDuty, dp.pwmMaxDuty, ds.pwmDutyAccumulator);
      break;
    case eMotorSmoothing:
      snprintf(detailString, sizeof(detailString), "mSmooth: %.3f", dp.motorFilterWeight);
      break;
    case ePitchTrim:
      snprintf(detailString, sizeof(detailString), "Trim: %.2f", dp.pitchTrim);
      break;
    case eMaxThrottle:
      snprintf(detailString, sizeof(detailString), "Throttle: %.0f%%", dp.maxThrottleBias * 100.0f);
      break;
    case eDeadzone:
      snprintf(detailString, sizeof(detailString), "mDead: %.0f", dp.deadZone);
      break;
    case eHBridgeIdle:
      if (dp.idleMode == eBraking)
        snprintf(detailString, sizeof(detailString), "Idle=[B] /  C ");
      else
        snprintf(detailString, sizeof(detailString), "Idle= B  / [C]");
      break;
    case ePwmFreq:
      snprintf(detailString, sizeof(detailString), "pwmFreq=%d", ds.pwmFreq);
      break;
    case eDIIRWeight:
      snprintf(detailString, sizeof(detailString), "P_IIR: %.2f", pp.dIIRWeight);
      break;
    case eVelocityDIIRWeight:
      snprintf(detailString, sizeof(detailString), "V_IIR: %.2f", vp.dIIRWeight);
      break;
    case eVelocityCurSpeedIIRWeight:
      snprintf(detailString, sizeof(detailString), "SP_IIR: %.3f", vp.inputIIRWeight);
      break;
    case ePitchPidKp:
      snprintf(detailString, sizeof(detailString), "pPID *%.2f %.2f %.2f", pp.kp, pp.ki, pp.kd);
      break;
    case ePitchPidKi:
      snprintf(detailString, sizeof(detailString), "pPID %.2f *%.2f %.2f", pp.kp, pp.ki, pp.kd);
      break;
    case ePitchPidKd:
      snprintf(detailString, sizeof(detailString), "pPID %.2f %.2f *%.2f", pp.kp, pp.ki, pp.kd);
      break;
    case eVelocityPidKp:
      snprintf(detailString, sizeof(detailString), "vPID *%.2f %.2f %.2f", vp.kp, vp.ki, vp.kd);
      break;
    case eVelocityPidKi:
      snprintf(detailString, sizeof(detailString), "vPID %.2f *%.2f %.2f", vp.kp, vp.ki, vp.kd);
      break;
    case eVelocityPidKd:
      snprintf(detailString, sizeof(detailString), "vPID %.2f %.2f *%.2f", vp.kp, vp.ki, vp.kd);
      break;
    case eInvertVelocityPid:
      if (vp.invertFeedback)
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
  snprintf(titleString, sizeof(titleString), "Pit=%+04.1f PWM=%.1f", state->imuState.orientation.pitch, ds.pwmDutyAccumulator);  // lastFrameRate
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
