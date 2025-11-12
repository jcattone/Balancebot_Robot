// Pinout:
//   I2C SDA: 21
//   I2C SCL: 22
//   Analog Joystick X/Y: 25, 26
// Devices
//   MPU-6050: I2C
//     22uF cap across power
//   SSD1306 128x32: I2C
//   DRV8833
//     Motor 1: Pins 18, 19
//   10k pull-up resistors on both I2C lines

#include <cmath>  // sin()

// The Adafruit library gives us basic MPU-6050 accel and gyro data
#include <Adafruit_MPU6050.h>

#ifdef HAS_DISPLAY
// We'll use an SSD1306 128x32 display in lieu of serial output,
// which can interfere with the I2C bus
#include <Adafruit_SSD1306.h>
#endif

// AHRS algorithms produce a stablized estimation of bearing from a 6-axis (or 9-axis) IMU.
// The Mahony algorithm is computationally inexpensive, and seems to provide sufficiently
// good results (better than a basic complementary filter)
// Other algos (Madgwick, NXP) were difficult to tune, and were not delivering better data
// for the purpose of a balance bot
#include "Adafruit_AHRS_Mahony.h"

#include <esp32-hal-ledc.h>

// ESP-NOW control
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "EspNowRemote.h"
#include "EspNowRemote_Events.h"
using namespace EspNowRemote;

#include "Types.h"
#include "BalancePID.h"

RmtBase* remote = EspNowRemote::MakeController();
joystick_state_t g_joystick_state = {};

const int g_sample_freq = 200;
const int g_update_freq = 200;

const int g_sample_period = 1000 / g_sample_freq;
const int g_update_period = 1000 / g_update_freq;

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
Adafruit_Mahony filter(16.6f, 0.3f);  // ...(float prop_gain, float int_gain) // Kp, Ki

// The IMU instance itself
Adafruit_MPU6050 mpu;

#ifdef HAS_DISPLAY
// The display... Assumed to be on the default I2C pins
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
#endif

bool gHasDisplay = false;
#define SCREEN_WIDTH 128                                     // OLED display width, in pixels
#define SCREEN_CHAR_WIDTH 21                                 // 5+1 pixel font width
#define SCREEN_HEIGHT 32                                     // OLED display height, in pixels
#define SCREEN_HEIGHT_ROWS 4                                 // 7+1 pixel font height
#define OLED_RESET -1                                        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C                                  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
char textBuffer[SCREEN_HEIGHT_ROWS][SCREEN_CHAR_WIDTH + 1];  // the +1 is for a null terminator

eConfigMode gCurrentConfigMode = eDefaultConfigMode;



#define MOTORA_PIN_1 25
#define MOTORA_PIN_2 26
#define MOTORB_PIN_1 32
#define MOTORB_PIN_2 33

#define LED_PIN 2

// TODO: Motor gain/trim (i.e., adjust for differences in left/right speed or stiction-break)
// TODO: Motor ratio as proxy for steering
// TODO: Remote-control steering
// TODO: Correction from bounce/bump is often excessive (insufficient compensation, or maybe needs to go above maxpwm briefly?)
// TODO: Wheel encoders for feedback / auto-calibrate trim
// TODO: Floor-proximity and collision sensors
// TODO: Third-tier w/ additional sensors?
// TODO: Clean up wiring
// TODO: Easier onboard vs. usb power switching
// TODO: Settings persistence

eHBridgeIdleMode gHBridgeIdleMode = eBraking;
// The following three are floats (instead of int) to avoid runtime conversion
// to float when comparing to gPwmDutyAccumulator / gPwmDutyAppliedMagnitude
float gDeadZone = 1;
float gPwmMinDuty = 22;
float gPwmMaxDuty = 160;                  // ((2 * 4.2) - 0.7) * (160 / 255) ~= 5V (2x 18650 - diode drop * pwm ratio = rated TT motor voltage)
float gPwmDutyAccumulator = 0.0f;       // The raw PWM target
float gPwmDutyAppliedMagnitude = 0.0f;  // gPwmDutyAccumulator, but scaled to exclude the dead zone and map into the min/max PWM range
int gPwmFreq = 2 * g_update_freq;
float gLastSetpoint = 0.0f;
float gPitchTrim = 0.8f;

float gPitchPidKp = 0.17f;  // Or 0.24,0,0.06
float gPitchPidKi = 0.0f;
float gPitchPidKd = 0.07f;

float gMotorFilter = 0.684f;
float gDIIRWeight = 0.16f;

float gVelocityDIIRWeight = 0.80f;
float gVelocityPidKp = 0.14f;
float gVelocityPidKi = 0.0f;
float gVelocityPidKd = 0.0f;

// esp_now_send_cb_t
void OnDataSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t send_status) {
#ifdef SERIAL_DIAG
  Serial.printf("OnDataSent, type=%d, len=%d\n", tx_info->data[3], tx_info->data_len);
#endif
  remote->HandleDataSent(tx_info, send_status);
}

// esp_now_recv_cb_t
// All received ESP-NOW traffic arrives here, and is funnelled into the remote.
void IRAM_ATTR OnDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
#ifdef SERIAL_DIAG
  Serial.printf("ESPNOW, OnDataRecv => %d bytes\n", data_len);
#endif
  remote->HandleDataReceived(esp_now_info, data, data_len);
}

// When the remote controller wants to raise a message, it will be published here.
unsigned long last_hid_input_timestamp = 0;
bool OnControllerMessage(uint8_t msg_type, const uint8_t* data, int data_len) {
  switch (msg_type) {
    case EspNowRemote::MSGTYPE_SELF_STATUS:
      // Loopback status from the local remote instance... not from the controller.
      memcpy(textBuffer[1], data, data_len);
      textBuffer[1][data_len] = 0;
      //#ifdef SERIAL_DIAG
      Serial.println((const char*)data);
      //#endif
      break;

    case MSGTYPE_RMT_JOYSTICK:
      memcpy(&g_joystick_state, data, data_len);
      last_hid_input_timestamp = micros();
      break;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  // ESP-NOW control
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);  // Required to set a specific channel
  esp_wifi_set_channel(1 /* channel */, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // TODO: initMotors()
  pinMode(MOTORA_PIN_1, OUTPUT);
  pinMode(MOTORA_PIN_2, OUTPUT);
  pinMode(MOTORB_PIN_1, OUTPUT);
  pinMode(MOTORB_PIN_2, OUTPUT);

  analogWriteFrequency(MOTORA_PIN_1, gPwmFreq);
  analogWriteFrequency(MOTORA_PIN_2, gPwmFreq);
  analogWriteFrequency(MOTORB_PIN_1, gPwmFreq);
  analogWriteFrequency(MOTORB_PIN_2, gPwmFreq);
  // Initializing the motor pins uniformly solves the 'jerk on startup' problem
  if (gHBridgeIdleMode == eBraking) {
    analogWrite(MOTORA_PIN_1, 1);
    analogWrite(MOTORA_PIN_2, 1);
    analogWrite(MOTORB_PIN_1, 1);
    analogWrite(MOTORB_PIN_2, 1);
  } else {
    analogWrite(MOTORA_PIN_1, 0);
    analogWrite(MOTORA_PIN_2, 0);
    analogWrite(MOTORB_PIN_1, 0);
    analogWrite(MOTORB_PIN_2, 0);
  }

  initDisplay();
  initImu();

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  remote->Setup();
  remote->RegisterMsgHandler(OnControllerMessage);
  remote->Start();
}

void loop() {
  remote->Loop();

  handleInput();
  updateOrientation();
  updateDisplay();
  updateRemoteDisplay();
  updateMotors();
}

// TODO: position or velocity pid control via angle setpoint guidance
// Velocity: based on the rate the inactive bot would tip & fall, a desired
// speed can be achieved by leaning into the desired direction.  As the
// target speed is achieved, the lean must be reversed.  If aiming for
// station-holding, seek a pwm rate of 0.  Infer speed from the current
// pwm duty cycle.
// Position: this requires integration of motor output over time, or
// even the use of a position encoder.

// Given a desired angle, guide acceleration.
bool pitchPidUpdate(float desiredAngle, float deltaTSec, float& accelOut) {
  // TODO: Map accel based on angle, knowing that small angles need
  // very little correction, but high angles need super-linear adjustment.
  // e.g., the accel could be proportional to cos(errorAngle),
  // or (perhaps more accurately), cos(angle) where vertical is 0

  // PID per-update inputs
  // TODO: trim for the balance point
  // TODO: setpoint offset for travel
  // [-90, -90] generally speaking (pitch decreases after 90 for some reason?)
  float currentAngle = gOrientation.pitch;

  // --------------------------------------
  // Pitch-driven fault detection with hysteresis
  bool pitchPidFault;
  {
    static bool pitchFaultFlag = false;
    if (!pitchFaultFlag)
      pitchPidFault = currentAngle < -50 || currentAngle > 50;
    else
      pitchPidFault = currentAngle < -5 || currentAngle > 5;
    pitchFaultFlag = pitchPidFault;
  }

  // -----------------------------
  // Pitch-driven PID

  // How far off are we?
  // TODO: currentAngleAdjusted is where we could subtract the
  // angle error predicted from the current acceleration.
  // This is sometimes known as 'covariance' adjustment,
  // compensating for the acceleration's impact on the
  // apparent gravity vector.
  // The applied acceleration would need to be estimated from the
  // change in speed over time... we could use a circular buffer
  // with a configurable lookback?
  // The challenge is that the IMU responds to accel changes
  // extremely rapidly, making it difficult to accurately predict
  // the accel perturbation.
  // We could also selectively suppress the IMU accel data if under
  // acceleration, shifting emphasis to the integrated gyro for
  // short-term noisy action.  The danger is that balancing is
  // inherently oscillatory, and so frequently under accel.
  float errorAngle = currentAngle - desiredAngle;

  // P: [-90, 90] typical
  float P = errorAngle;

  // I: ?
  static float errorIntegral = 0.0;

  // Clear the integral when we cross the setpoint (don't carry windup across a stable threshold).
  // Note that this doesn't account for inertia - we might wind up crossing rapidly - this suggests
  // a use for the derivative term.
  static int lastErrorSign = 0;
  int currentErrorSign = std::signbit(errorAngle);
  if (currentErrorSign != lastErrorSign)
    errorIntegral = 0;
  lastErrorSign = currentErrorSign;

  // Only accumulate error when we aren't saturated (or very near to it)
  if (gPwmDutyAppliedMagnitude < (gPwmMaxDuty - 1.0f)) {
    errorIntegral += errorAngle * deltaTSec;
  }

  float I = errorIntegral;

  // D=D
  // Identify the rate at which the error is changing
  // As this may be very noisy, a low-pass (short IIR) filtered
  // version is also available.  Maybe take the derivative as the
  // difference between two different IIR periods?
  // Note: the Kalman filter is supposed to predict the current delta,
  // rather than work from historical data - a theoretical improvement
  // at additional complexity.
  static float lastErrorAngle = 0.0f;
  static float errorDeltaPerSecondIIR = 0.0f;
  float errorDeltaPerSecond = (errorAngle - lastErrorAngle) / deltaTSec;
  // IIR; weight the accumulator heavily, and the new value lightly.
  errorDeltaPerSecondIIR = gDIIRWeight * errorDeltaPerSecond + (1.0f - gDIIRWeight) * errorDeltaPerSecondIIR;
  lastErrorAngle = errorAngle;
  float D = errorDeltaPerSecondIIR;
  // { // Debug
  //   static unsigned long lastLogMillis = 0;
  //   if (millis() - lastLogMillis > 50) {
  //     Serial.printf("P:%.2f,I:%.2f,D:%.2f\n", P, I, D);
  //     lastLogMillis = millis();
  //   }
  // }
  accelOut = gPitchPidKp * P + gPitchPidKi * I + gPitchPidKd * D;
  return pitchPidFault;
}


// Given a desired speed, guide the pitch.
bool velocityPidUpdate(float desiredSpeedNormalized, float deltaTSec, float& pitchTarget) {
  // We could look at gPwmDutyAccumulator directly, but we can disregard
  // the deadzone and power scaling by deriving an effective velocity
  // from the applied magnitude relative to the scaled power range.
  // Constrain values lower than gPwmMinDuty to be equivalent to 0.
  float currentVelocityNormalized = max(0.0f, (gPwmDutyAppliedMagnitude - gPwmMinDuty) / (gPwmMaxDuty - gPwmMinDuty));
  bool velocityPidFault = currentVelocityNormalized < -0.01f || currentVelocityNormalized > 1.01f;
  if (gPwmDutyAccumulator < 0.0f)
    currentVelocityNormalized *= -1.0f;


  // -----------------------------
  // Velocity-driven PID
  float errorVelocity = currentVelocityNormalized - desiredSpeedNormalized;
  float P = errorVelocity;

  static float errorIntegral = 0.0;
  // Clear the integral when crossing the setpoint.
  static int lastErrorSign = 0;
  int currentErrorSign = std::signbit(errorVelocity);
  if (currentErrorSign != lastErrorSign)
    errorIntegral = 0;
  lastErrorSign = currentErrorSign;

  // Only accumulate error when we aren't saturated or stationary
  float curVelMag = std::abs(currentVelocityNormalized);
  if (curVelMag > 0.01f && curVelMag < 0.99f) {
    errorIntegral += errorVelocity * deltaTSec;
  }

  float I = errorIntegral;

  static float lastErrorVelocity = 0.0f;
  static float errorDeltaPerSecondIIR = 0.0f;
  float errorDeltaPerSecond = (errorVelocity - lastErrorVelocity) / deltaTSec;
  // IIR; weight the accumulator heavily, and the new value lightly.
  errorDeltaPerSecondIIR = gVelocityDIIRWeight * errorDeltaPerSecond + (1.0f - gVelocityDIIRWeight) * errorDeltaPerSecondIIR;
  lastErrorVelocity = errorVelocity;
  float D = errorDeltaPerSecondIIR;

  pitchTarget = -60 * (gVelocityPidKp * P + gVelocityPidKi * I + gVelocityPidKd * D);
  return velocityPidFault;
}

// TODO: Thoughts...
// The bot is very short, making any correction movement produce
// rapid angle changes.  Slow the response by raising the center
// of mass.
//
// The jumpiness might be exacerbated by a high IMU Kp, allowing
// large pitch deltas due to linear acceleration.  By reducing Kp
// (e.g., to very small levels), we work more from the integrated
// gyro, rather than the accel data.  Just need to be cautious to
// still correct from the accel (gravity) vector, but maybe do
// that via an adaptive weight that enabled gravity correction
// only when the motor has been at near-constant velocity for a
// short while (or just have a supression metric driven by non-zero)
// PID accel output.
//
// Also, the IMU seems to be physically skewed - we might want to
// configure a pitch trim, so that we can zero out the pitch.
//
// The motor response also seems to aggressive, ramping rapidly to
// full power.  Not only does this cause wheel slippage, but it makes
// small-scale balance very difficult.  Look into better low-range
// control, possibly remapping the accel data?
//
// Very small-scale pitch values may result in accel data so small as
// to fall under the quantization threshold.  Report & accumulate
// accel as a float?
//
// The velicity setpoint adjustment seems inverted in feedback direction
// (this has been addressed by negating velocityPidUpdate's setpoint output)
void updateMotors() {
  static int startupPwmAttenuation = 0;

  // TODO: Refactor this:
  // Inputs (signal, tuning), state, outputs

  // ----------------------------------------
  // Throttle
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate < g_update_period)
    return;
  lastUpdate = now;

  // ---------------------------------------
  // Inter-sample time scaling

  // Calculate dT in seconds (the actual time unit is arbitrary, as long as we're consistent)
  // TODO: Is the above true?
  static unsigned long lastSampleTime = 0;
  static bool isFirstSample = true;
  float deltaTSec = static_cast<float>(now - lastSampleTime) / 1000.0f;
  lastSampleTime = now;
  // The first sample is used only to set the sample time, so that
  // the next sample (the first real one) can be evaluated with an
  // accurate inter-sample deltaT.
  if (isFirstSample) {
    isFirstSample = false;
    return;
  }

  // --------------------------------------
  // Pitch-driven PID
  float desiredSpeedNormalized = 0.0f;
  float desiredAngle;
  bool velocityPidFault = velocityPidUpdate(desiredSpeedNormalized, deltaTSec, desiredAngle);
  desiredAngle += gPitchTrim;
  gLastSetpoint = desiredAngle;
  float pidAccelOut;
  bool pitchPidFault = pitchPidUpdate(desiredAngle, deltaTSec, pidAccelOut);

  // ----------------------------------
  // Convert PID guidance to normalized PWM duty cycle

  // Note that we want the acceleration (not speed) to be modulated by the PID output.
  // TODO: Note that this is an integer, limiting fine-grained control especially close to 0
  float rawAccel = constrain(pidAccelOut, -255.0f, 255.0f);

  // Attenuate the PWM output on startup to prevent noisy output
  float constrainedAccel = constrain(rawAccel, (float)-startupPwmAttenuation, (float)startupPwmAttenuation);
  if (startupPwmAttenuation < 255)
    ++startupPwmAttenuation;

  // Adjust the speed by the desired relative acceleration, constraining the duty cycle to the PWM limits.
  // Note that this can be negative, to indicate a reversed direction.
  // TODO: Permit brief excusions beyond gPwmMaxDuty (up to 255) for recovery, but trigger 'unsafe' if operating beyond saturation for more than briefly?)
  gPwmDutyAccumulator = constrain(gPwmDutyAccumulator + constrainedAccel, -gPwmMaxDuty, gPwmMaxDuty);

  // Optional filter
  // TODO: Predictive window-based outlier attenuation, but otherwise allow small variations with no additional latency?
  if (gMotorFilter <= 0.999f) {
    static float motorIIR = 0;
    motorIIR = (gMotorFilter * gPwmDutyAccumulator) + (1.0 - gMotorFilter) * motorIIR;
    gPwmDutyAccumulator = (motorIIR + 0.5);
  }

  if (gPwmDutyAccumulator < gDeadZone && gPwmDutyAccumulator > -gDeadZone) {
    gPwmDutyAccumulator = 0.0f;
    gPwmDutyAppliedMagnitude = 0.0f;
  } else {
    // We're not in the dead zone; compress the output to eliminate the dead zone
    // TODO: each of these coersced constraints can be changed to float as well?
    gPwmDutyAppliedMagnitude = map(std::abs(gPwmDutyAccumulator), gDeadZone, gPwmMaxDuty, gPwmMinDuty, gPwmMaxDuty);
  }


  // -----------------------
  // Safety limiter
  // More than 500ms continuously faulted (e.g., at an unsafe angle) will shut down the motor.
  // Upon reactivation, the soft-start is reset
  // TODO: Reset PID accumulators also?

  // Track the contiguous duration over which any fault exists
  static unsigned long pidFaultTimeMs = 0;
  static bool pidFault = false;
  bool wasPidFault = pidFault;
  pidFault = velocityPidFault || pitchPidFault;
  if (pidFault && !wasPidFault) {
    if (velocityPidFault)
      Serial.println("Velocity Fault!");
    else
      Serial.println("Pitch Fault!");
    pidFaultTimeMs = now;
  }

  static bool faultLedState = false;
  if (pidFault && (now - pidFaultTimeMs) > 100) {
    // In case we're experimenting without the motor enabled,
    // the onboard LED is used as a fault indicator.  The
    // transition is placed here (instead of above) to ensure
    // the LED corresponds with the effective fault treatment
    // (i.e., including the 500ms delay).
    if (!faultLedState) {
      digitalWrite(LED_PIN, HIGH);
      faultLedState = true;
    }
    gPwmDutyAccumulator = 0.0f;
    gPwmDutyAppliedMagnitude = 0.0f;
    startupPwmAttenuation = 0;
  } else if (faultLedState) {
    digitalWrite(LED_PIN, LOW);
    faultLedState = false;
  }

  // -----------------------
  // Translate duty to control signals

  // Depending upon direction, the DRV8833 needs different pins driven.
  bool m1Driven;
  int m1 = 0, m2 = 0;
  int quantizedMagnitude = (int)std::round(gPwmDutyAppliedMagnitude);
  if (gPwmDutyAccumulator < 0.0f) {
    // Note: for fast-decay (coasting), one pin is pulled low, the other is high at the desired duty.
    // For slow-decay (braking), one pin is pulled high, the other is low at the desired duty.
    // Because analogWrite specified the high-side duty cycle, slow-decay mode means we invert both
    // of the duty cycles (i.e., 255 on on to hold it high, and 255-duty on the other, so that it is
    // LOW at the desired duty)
    m2 = quantizedMagnitude;
    m1Driven = false;
  } else {
    m1 = quantizedMagnitude;
    m1Driven = true;
  }

  // Adjust for braking (slow-decay) / coasting (fast-decay) drive modes.
  if (gHBridgeIdleMode == eBraking) {
    // To invert the H-Bridge input, invert the levels and also
    // swap the driven IO to maintain direction.
    int m1Temp = m1;
    m1 = 255 - m2;
    m2 = 255 - m1Temp;
    m1Driven = !m1Driven;
  }

  // Update the motor PWM.
  analogWrite(MOTORA_PIN_1, m1);
  analogWrite(MOTORA_PIN_2, m2);
  analogWrite(MOTORB_PIN_1, m1);
  analogWrite(MOTORB_PIN_2, m2);
}

void initDisplay() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  gHasDisplay = false;
#ifdef HAS_DISPLAY
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
#ifdef SERIAL_DIAG
    Serial.println(F("SSD1306 not found"));
#endif
    return;
  }

  display.display();
  delay(20);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
#endif

  gHasDisplay = true;
}

void initImu() {
  if (!mpu.begin()) {
    Serial.println("MPU-6050 init failed");
    while (1)
      yield();
  }

  // Param: samples per second
  filter.begin(g_sample_freq);
  // THen:
  // filter.updateIMU(gx/y/z, ax/y/z, [optional dT]) // DPS (deg. per sec) / Gs
  // getRoll/Pitch/Yaw(), getGravityVector()
  // or: getQuaternion
}

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
  if (g_joystick_state.count > 0) {
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
        if (right) filter.setKp(min(100.0, filter.getKp() + 0.1 * count));
        else if (left) filter.setKp(max(0.0, filter.getKp() - 0.1 * count));
        break;
      case eFusionKi:
        if (right) filter.setKi(min(100.0, filter.getKi() + 0.1 * count));
        else if (left) filter.setKi(max(0.0, filter.getKi() - 0.1 * count));
        break;
      case ePitchTrim:
        if (right)
          gPitchTrim = min(10.0f, gPitchTrim + 0.1f * count);
        else if (left)
          gPitchTrim = max(-10.0f, gPitchTrim - 0.1f * count);
        break;
      case ePwmMin:
        if (right) gPwmMinDuty = min(gPwmMaxDuty, gPwmMinDuty + (float) count);
        else if (left) gPwmMinDuty = max(0.0f, gPwmMinDuty - (float) count);
        break;
      case ePwmMax:
        if (right) gPwmMaxDuty = min(255.0f, gPwmMaxDuty + (float) count);
        else if (left) gPwmMaxDuty = max(gPwmMinDuty, gPwmMaxDuty - (float) count);
        break;
      case ePwmFreq:
        {
          for (int i = 0; i < count; ++i) {
            if (right) gPwmFreq = min(40000, max(gPwmFreq + 1, (int)(gPwmFreq * 1.1f)));
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
      case eDeadzone:
        if (right)
          gDeadZone = min(255.0f, gDeadZone + (float) count);
        else if (left)
          gDeadZone = max(0.0f, gDeadZone - (float) count);
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
      case eIMUDisplay:
      default:
        break;
    }
  }
}

void adjustPidK(float* f, float delta) {
  *f += delta;
  if (*f < 0.0f)
    *f = 0.0f;
}

void updateDisplay() {
  if (!gHasDisplay)
    return;

  static unsigned long lastUpdate = 0;
  unsigned int now = millis();
  if (now - lastUpdate < g_update_period)
    return;
  lastUpdate = now;

#ifdef HAS_DISPLAY
  display.clearDisplay();

  // Rows 1-3: Orientation
  display.setCursor(0, 0);
  display.printf("Roll:%.2f\nPitch:%.2f\nYaw:%.2f", gOrientation.roll, gOrientation.pitch, gOrientation.yaw);
  display.display();
#endif
}

// TODO: ABort immediately if not connected, and always update when reconnected
void updateRemoteDisplay() {
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
    case eIMUDisplay:
      snprintf(detailString, sizeof(detailString), "IMU: R%+04.1f P%+04.1f", gOrientation.roll, gOrientation.pitch);
      break;
    case eFusionKp:
      sprintf(detailString, "Kp=*%.1f Ki=%.1f", filter.getKp(), filter.getKi());
      break;
    case eFusionKi:
      sprintf(detailString, "Kp=%.1f Ki=*%.1f", filter.getKp(), filter.getKi());
      break;
    case ePwmMin:
      sprintf(detailString, "pwm=*%.0f-%.0f (%.1f)", gPwmMinDuty, gPwmMaxDuty, gPwmDutyAccumulator);
      break;
    case ePwmMax:
      sprintf(detailString, "pwm=%.0f-*%.0f (%.1f)", gPwmMinDuty, gPwmMaxDuty, gPwmDutyAccumulator);
      break;
    case ePwmFreq:
      sprintf(detailString, "pwmFreq=%d (%.1f)", gPwmFreq, gPwmDutyAccumulator);
      break;
    case eMotorSmoothing:
      sprintf(detailString, "mSmooth: %.3f", gMotorFilter);
      break;
    case ePitchTrim:
      sprintf(detailString, "Trim: %.2f", gPitchTrim);
      break;
    case eDIIRWeight:
      sprintf(detailString, "P_IIR: %.2f", gDIIRWeight);
      break;
    case eVelocityDIIRWeight:
      sprintf(detailString, "V_IIR: %.2f", gVelocityDIIRWeight);
      break;
    case eDeadzone:
      sprintf(detailString, "mDead: %.0f", gDeadZone);
      break;
    case eHBridgeIdle:
      if (gHBridgeIdleMode == eBraking)
        sprintf(detailString, "Idle=[B] /  C ");
      else
        sprintf(detailString, "Idle= B  / [C]");
      break;
    case ePitchPidKp:
      sprintf(detailString, "pPID *%.2f %.2f %.2f", gPitchPidKp, gPitchPidKi, gPitchPidKd);
      break;
    case ePitchPidKi:
      sprintf(detailString, "pPID %.2f *%.2f %.2f", gPitchPidKp, gPitchPidKi, gPitchPidKd);
      break;
    case ePitchPidKd:
      sprintf(detailString, "pPID %.2f %.2f *%.2f", gPitchPidKp, gPitchPidKi, gPitchPidKd);
      break;
    case eVelocityPidKp:
      sprintf(detailString, "vPID *%.2f %.2f %.2f", gVelocityPidKp, gVelocityPidKi, gVelocityPidKd);
      break;
    case eVelocityPidKi:
      sprintf(detailString, "vPID %.2f *%.2f %.2f", gVelocityPidKp, gVelocityPidKi, gVelocityPidKd);
      break;
    case eVelocityPidKd:
      sprintf(detailString, "vPID %.2f %.2f *%.2f", gVelocityPidKp, gVelocityPidKi, gVelocityPidKd);
      break;
  }

  // Send to the remote

  static unsigned long last_title_sent_millis = 0L;
  char titleString[SCREEN_CHAR_WIDTH] = { 0 };
  static char lastTitleBuf[SCREEN_CHAR_WIDTH + 1] = {};
  snprintf(titleString, sizeof(titleString), "P%+04.1f M%.1f S%.1f", gOrientation.pitch, gPwmDutyAccumulator, gLastSetpoint);  // lastFrameRate
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

void updateOrientation() {
  static unsigned long lastUpdate = 0;
  unsigned int now = millis();
  // Target 100 Hz, coordinated with the rate we provided to filter.begin()
  if (now - lastUpdate < g_sample_period)
    return;
  lastUpdate = now;

  sensors_event_t a, g, temp;

  // Get the accel (gravity vector) in m/s
  // Get the gyro (turn rate) in radians-per-second
  mpu.getEvent(&a, &g, &temp);

  // Update in degrees-per-second and gravities
  filter.update(
    // From radians-per-second to degrees-per-second
    g.gyro.x * 180.0f / M_PI,
    g.gyro.y * 180.0f / M_PI,
    g.gyro.z * 180.0f / M_PI,
    // From m/s to gravities
    a.acceleration.x / 9.81f,
    a.acceleration.y / 9.81f,
    a.acceleration.z / 9.81f,
    0.0f,
    0.0f,
    0.0f);
  // TODO: add dT, or ensure that updates are closely tied to whatever update
  // rate we provided to the filter.begin (currently, 100 updates / sec)?
  // Calc using micros(), not millis(), for accuracy
  // dT is measures in seconds (i.e., nominally .01 @ 100Hz)
  // TODO: Send loops/sec to the remote

  gOrientation.roll = filter.getRoll();
  gOrientation.pitch = filter.getPitch();
  gOrientation.yaw = filter.getYaw();
}
