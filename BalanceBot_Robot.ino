// Pinout:
//   I2C SDA: 21
//   I2C SCL: 22
// Devices
//   2S-18650: Motor power
//   MP1584 buck converter (5V out for MCU)
//   MPU-6050: I2C
//     (optional) 22uF cap across power
//   DRV8833
//     Motor 1: Pins 32, 33
//     Motor 2: Pins 25, 26
//   10k pull-up resistors on both I2C lines
//   1000uF cap on motor power
//   0.1uF bypass cap on each motor power

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
joystick_analog_state_t g_joystick_analog_state = {};

uint8_t g_last_message_type = MSGTYPE_UNKNOWN;

const int g_sample_freq = 200;  // Hz
const int g_update_freq = 200;  // Hz

const int g_sample_period = 1000 / g_sample_freq;  // ms
const int g_update_period = 1000 / g_update_freq;  // ms

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
Adafruit_Mahony filter{};  // ...(float prop_gain, float int_gain) // Kp (was 16-ish), Ki

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

#define BATTERY_SENSE_PIN 36

#define MOTORA_PIN_1 32
#define MOTORA_PIN_2 33
#define MOTORB_PIN_1 25
#define MOTORB_PIN_2 26
const int PWM_PRECISION = 12;
const int PWM_MAX = ((1 << PWM_PRECISION) - 1);
const int PWM_SCALE_FROM_8BIT = (1 << (PWM_PRECISION - 8));

#define LED_PIN 2

// TODO: Motor gain/trim (i.e., adjust for differences in left/right speed or stiction-break)
// TODO: Wheel encoders for feedback / auto-calibrate trim
// TODO: Floor-proximity and collision sensors
// TODO: Third-tier w/ additional sensors?
// TODO: Clean up wiring
// TODO: Easier onboard vs. usb power switching
// TODO: Settings persistence
// TODO: Allow a small anti-stiction reservoir (reset when the motor is turned off) to briefly (and mildly) boost the power when transitioning out of a full-stop
// TODO: Sometimes locks up after a fall (motor stuck running, no further variation or remote response)
//       (might have been related to a buffer overflow when writing to the remote display buffer)
// TODO: Separate params for coasting/braking modes
// TODO: Partially attenuate Angle PID Kp based on angle max IIR, allowing response
//       to become more subtle near balance, but immediately ramp up for correction.
//       Maybe other params are adaptive as well?
// TODO: Auto-tune pitch trim - observe average power when gThrottleBias == 0, slowly adjust gPitchTrim to bring the averaged gPwmDutyAccumulator closer to 0
// TODO: Battery gauge: investigate BatterySense library, only sample when the motor is off, or at least not accelerating to a greater magnitude?
// TODO: Set a battery fault if the voltage falls below a critical level
// TODO: Use voltage sense to dynamically adjust the pwm range (at the least, the max), but use an IIR or when-motor-off sampling to avoid oscillation from motor draw

// Alt: pwmFreq: 400, Kp 6.9/0.3, p0.86/0/0.3, P_IIR 0.5, v8.0,0,0, V_IIR 0.5, SP_IIR 0.010, Throttle 100%, mSmooth 0.908,
//   Weak balance, somewhat jittery
// Alt: pwmFreq: 16k, Kp 3.7/0.3, p0.46/0/0.04, P_IIR: 0.5, v6/0/0, V_IIR 0.5, SP_IIR 0.010, Throttle 100%, mSmooth 0.908,
//   Seems a resilient (if rubberbandy) balance, driveable, somewhat resistant to sudden wheel blockage
//   pKd=0.07, P_IIR=1.0 seems to reduce jitter and be just enough responsive to moderate disturbances
//   Learning... overly aggressive Mahony Kp was at the heart of much of the jitter and instability.
//   D-smoothing (gDIIRWeight) further caused D to lag.  This might have produced a phase offset (lag)
//     that resulted in oscillation / orbiting in the state-space?
//   While keeping target speeds to < 5V average for the motors' benefit, we can use brief bursts up to full
//     supply voltage (~8.4V) for emergency correction. However, that should be limited to prevent motor damage.

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
float gPitchTrim = -1.8f;  // The IMU tends to shift, and the CoM isn't quite over the axle, so -2.6..-4.0 seems to be the sweet spot

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

  pinMode(BATTERY_SENSE_PIN, INPUT);

  initMotors();
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
  updateBattery();
  updateDisplay();
  updateRemoteDisplay();
  updateMotors();
}

void updateBattery() {
  unsigned long now = millis();
  static unsigned long lastSense = 0;
  static float voltageIIR = 0.0f;
  if (now - lastSense > 1000) {
    lastSense = now;
    uint16_t battery = analogRead(BATTERY_SENSE_PIN);
    float currentVoltage = 3.78f * (3.3f * (float)battery / 4095.0f);

    // Apply heavy time-averaging to the voltage, which will appear to swing significantly
    // due to the motors' current draw.
    // Note that the capacitor will hold a charge for quite a while after removal of the
    // primary power source.
    if (currentVoltage < 1.0f)
      voltageIIR = 0.0f;
    else if (voltageIIR <= 0.0f)
      voltageIIR = currentVoltage;
    else
      voltageIIR = 0.001 * currentVoltage + 0.999 * voltageIIR;
    gVoltage = voltageIIR;
    const float lowLevel = 2 * 3.4f;
    const float highLevel = 2 * 4.2f;
    gVoltagePercent = max(0.0f, (gVoltage - lowLevel) / (highLevel - lowLevel) * 100.0f);

    // Auto-adjust the max throttle to limit nominal speed to a 5V average without prohibiting corrective spikes
    if (gVoltage >= 5.0f) {
      gMaxThrottleBias = min(gPwmMaxDuty / 255.0f * 0.7f, (5.0f / gVoltage));
    }
  }
}

// TODO: position or velocity pid control via angle setpoint guidance
// Velocity: based on the rate the inactive bot would tip & fall, a desired
// speed can be achieved by leaning into the desired direction.  As the
// target speed is achieved, the lean must be reversed.  If aiming for
// station-holding, seek a pwm rate of 0.  Infer speed from the current
// pwm duty cycle.
// Position: this requires integration of motor output over time, or
// even the use of a position encoder.

// TODO: Switch the motor polarity to better match the intuitive drive direction? Also update the comments...
// Given a desired angle, guide acceleration.
//
// The sign of accelOut matches that of the current pitch /error/
// (i.e., if currentAngle > desiredAngle, the accel is positive)
// thus, from a point of stability, increasing desiredAngle (a + change) will result in a (-) accel.
// That corresponds to the base (wheels) moving backwards to achieve a forward tilt.
// Once the current angle exceeds the desired angle (), the accel switches direction,
// seeking to drive the wheels to chase the body.
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

// TODO: I don't think this is doing quite what we want...
// The Velocity error is scaling P between 0 and Kp.
// desiredSpeedNormalized is merely scaling that between +/-1.0xP
// Keep in mind that the velocity control is normally a negative feedback, trying to
// couteract runaway / persistent velocity.
// We want the desired speed to be able to swing the pitch across the neutral point, and significantly so.
// The challenge is that we want rapid throttle feedback, but a more gradual counter to long-term speed.
// Can the D term help here?  Throttle would produce a rapid delta (though one that disappers rapidly)
// TODO: Remove the serial output.
// Given a desired speed, guide the pitch.
// If we need to speed up, lean into the appropriate direction
// If we need to slow down, lean away from the direction of travel
bool velocityPidUpdate(float desiredSpeedNormalized, float deltaTSec, float& pitchTarget) {
  // We could look at gPwmDutyAccumulator directly, but we can disregard
  // the deadzone and power scaling by deriving an effective velocity
  // from the applied magnitude relative to the scaled power range.
  // Constrain values lower than gPwmMinDuty to be equivalent to 0.
  // gPwmDutyAppliedMagnitude is pre-adjusted to account for the net impact of steering.
  float currentVelocityNormalized = max(0.0f, (gPwmDutyAppliedMagnitude - gPwmMinDuty) / (gPwmMaxDuty - gPwmMinDuty));

  // Check for out-of-bounds velocity (shouldn't happen?)
  bool velocityPidFault = currentVelocityNormalized < -0.01f || currentVelocityNormalized > 1.01f;

  // Apply the current direction of travel to the normalized magnitude
  if (gPwmDutyAccumulator < 0.0f)
    currentVelocityNormalized *= -1.0f;

  // Smooth the current velocity
  // NOTE: This smoothing (via gSpeedIIRWeight) plays heavily into the ability to control speed.
  // While we want to smooth the current velocity to ignore jittering, we also need the smoothed
  // velocity to respond fast enough to prevent the speed from accelerating to the point where
  // we no longer have enough corrective capacity to balance.  Similarly, we don't want the memory
  // of fast movement to bias the speed during low-throttle input.
  static float smoothedVelocityNormalized = 0.0f;
  smoothedVelocityNormalized = gSpeedIIRWeight * currentVelocityNormalized + (1.0f - gSpeedIIRWeight) * smoothedVelocityNormalized;

  // The subsequent pitch controller produces accel matching the sign of the current pitch /error/ (opposite to control changes)
  // At this point, a currentVelocityNormalized > 0 is travelling in the same direction as a positive pitch would produce.
  // To counteract drift, we would want to target a pitch opposite in sign of currentVelocityNormalized.
  // So, with a velocity target of 0 and a positive currentVelocityNormalized, the error is positive, so we want a
  // negative (inverted) pitch output, which produces positive accel (wheels passing the body) to result in the
  // desired negative pitch, which will require a reversal in direction to maintain.

  // Calculate current error from target
  // This has an absolute magnitude cap of 2.0
  // Note that this compares the time-averaged velocity to the instantaneous target
  float velocityErrorNormalized = smoothedVelocityNormalized - desiredSpeedNormalized;

  // Proportional feedback
  float P = velocityErrorNormalized;

  // Integral feedback
  static float errorIntegral = 0.0;

  // Clear the integral when crossing the setpoint.
  static int lastErrorSign = 0;
  int currentErrorSign = std::signbit(velocityErrorNormalized);
  if (currentErrorSign != lastErrorSign)
    errorIntegral = 0;
  lastErrorSign = currentErrorSign;

  // Only integrate error when we aren't saturated or stationary
  // This uses the instantaneous (non-smoothed) velocity to ensure we're
  // able to detect saturation without lag.
  float curVelMag = std::abs(currentVelocityNormalized);
  if (curVelMag > 0.01f && curVelMag < 0.99f) {
    // velocityErrorNormalized is unitless, measuring the raw delta between normalized current and target velocity.
    // The normalized velocity ranges across [-1.0, 1.0], and we want integration to be reasonably
    // agnostic to the update period.  So... we scale the velocity error by deltaT so that we're
    // integrating one velocityErrorNormalized per second.
    errorIntegral += velocityErrorNormalized * deltaTSec;
  }

  // Any scaling applied to the integral would just a scalar adjustment to Ki, so we'll avoid any further
  // scaling of the integrated error.
  float I = errorIntegral;

  // Derivative feedback
  static float lastErrorVelocity = 0.0f;
  static float errorDeltaPerSecondIIR = 0.0f;
  float errorDeltaPerSecond = (velocityErrorNormalized - lastErrorVelocity) / deltaTSec;
  // IIR; Smooth the delta over multiple readings
  errorDeltaPerSecondIIR = gVelocityDIIRWeight * errorDeltaPerSecond + (1.0f - gVelocityDIIRWeight) * errorDeltaPerSecondIIR;
  lastErrorVelocity = velocityErrorNormalized;
  float D = errorDeltaPerSecondIIR;

  pitchTarget = (gVelocityPidKp * P + gVelocityPidKi * I + gVelocityPidKd * D);
  if (gInvertVelocityPid)
    pitchTarget = -pitchTarget;
#ifdef SERIAL_DIAG
  Serial.printf("SpeedBias:%.2f,PitchTgt:%.3f\n", desiredSpeedNormalized, pitchTarget);
#endif
  return velocityPidFault;
}

// TODO: Thoughts...
// The bot is very short, making any correction movement produce
// rapid angle changes.  Slow the response by raising the center
// of mass.
//
// The motor response also seems to aggressive, ramping rapidly to
// full power.  Not only does this cause wheel slippage, but it makes
// small-scale balance very difficult.  Look into better low-range
// control, possibly remapping the accel data?
//
// TODO: Known problems:
//   * Runaway / persistent velocity - tends to drift after releasing throttle.
//     Why isn't the velocity pid countering lingering duty w/ counter-tilt?
//   * Sometimes, a large change of angle runs away rapidly.  Seems like pitch
//     pid not responding rapidly enough.  Fusion / IMU issue (Kp/Ki)?
void updateMotors() {
  static int startupPwmAttenuation = 0;

  // ----------------------------------------
  // Perform updates every 5ms (200/sec)
  static unsigned long lastUpdate = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastUpdate < g_update_period)
    return;
  lastUpdate = nowMs;

  // ---------------------------------------
  // Inter-sample time scaling
  // Give the PID feedback accurate time estimates
  // Calculate dT in seconds (the actual time unit is arbitrary, as long as we're consistent)
  static unsigned long lastSampleTimeMs = 0;
  static bool isFirstSample = true;
  float deltaTSec = static_cast<float>(nowMs - lastSampleTimeMs) / 1000.0f;
  lastSampleTimeMs = nowMs;

  // The first sample is used only to set the sample time, so that
  // the next sample (the first real one) can be evaluated with an
  // accurate inter-sample deltaT.
  if (isFirstSample) {
    isFirstSample = false;
    return;
  }

  // ---------------------------------------
  // Diagnostics output trigger (currently disabled)
  static unsigned long lastDiag = 0;
  bool emitDiag = false;
  // emitDiag = nowMs - lastDiag > 200;
  // if (emitDiag)
  //   lastDiag = nowMs;
  if (emitDiag) Serial.printf("------------------------------\n");

  // --------------------------------------
  // Pitch- and Speed-driven PID
  // The desired and average speeds are evaluated, producing a pitch setpoint.
  // The desired and current pitch are evaluated, producing nominal acceleration.
  // The nominal acceleration will later be modified to effect steering.
  // Either PID implementation can indicate a fault to stop the motors after a
  // very short debouncing period.

  // The desired speed is calculated as a normalized (relative) fraction of
  // the configured maximum duty cycle, e.g., [-1.0, 1.0], but is scaled down
  // to leave headroom for balance correction via further acceleration.
  float desiredSpeedNormalized = min(gThrottleBias, gMaxThrottleBias);  // gThrottleBias (unit), gMaxThrottleBias (1.0)
  float desiredPitchOut = 0.0f;
  bool velocityPidFault = velocityPidUpdate(desiredSpeedNormalized, deltaTSec, desiredPitchOut);
  if (emitDiag) {
    Serial.printf("desiredSpeedNormalized   : %.1f\n", desiredSpeedNormalized);
    Serial.printf("desiredPitchOut          : %.1f\n", desiredPitchOut);
  }

  // The current pitch will be compared to the desired pitch setpoint to determine
  // what acceleration is necessary to achieve that pitch setpoint.
  float desiredPitchTrimmed = desiredPitchOut + gPitchTrim;
  float pidAccelOut;  // -255..255 nominal
  bool pitchPidFault = pitchPidUpdate(desiredPitchTrimmed, deltaTSec, pidAccelOut);
  // The acceleration is constrained to a reasonable range
  float rawAccel = constrain(pidAccelOut, -255.0f, 255.0f);
  if (emitDiag) {
    Serial.printf("pidAccelOut              : %.1f\n", pidAccelOut);
    Serial.printf("rawAccel                 : %.1f\n", rawAccel);
  }

  float constrainedAccel = constrain(rawAccel, (float)-startupPwmAttenuation, (float)startupPwmAttenuation);

  // Upon startup, and after recovery from a fault, enforce a soft-start
  // by 'fading in' the max duty cycle
  if (startupPwmAttenuation < 255)
    ++startupPwmAttenuation;
  if (emitDiag) {
    Serial.printf("constrainedAccel         : %.1f\n", constrainedAccel);
  }

#ifdef ADAPTIVE_FUSION_KI
  // The rate at which the fusion filter corrects integrated gyro data from the gravity (accel) vector
  // is inversely proportional to the time-weighted accel average.  That is, gravity correction is
  // preferentially applied when not under the perturbing influence of linear acceleration (fwd/back).
  // TODO: Account for steering's perturbing effects on applied accel (attenuating fwd/back accel,
  // but increasing side-to-side accel).
  static float normalizedAccelMagPeak = 0.0f;
  normalizedAccelMagPeak = std::max(gAccelPeakDecay * normalizedAccelMagPeak, std::abs(constrainedAccel) / 255.0f);
  gMahonyKiScale = 1.0f - normalizedAccelMagPeak;
#endif

  // ----------------------------------
  // Convert PID guidance to normalized PWM duty cycle

  // Adjust the speed by the desired relative acceleration, constraining the duty cycle to the PWM limits.
  // Note that this can be negative, to indicate a reversed direction.
  // TODO: Permit brief excusions beyond gPwmMaxDuty (up to 255) for recovery, but trigger
  //       'unsafe' if operating beyond saturation for more than briefly?)
  //       * Split gPwmMaxDuty into soft & hard limits (default to 255?)
  //       * Maintain a 'cap' reservoir
  //       * Constrain to cap, reduce cap (decrement or decay) if > soft max
  //       * Regenerate toward hard max + delay when target is less than soft max
  //         * don't just reset - need to provide time for the motor to cool down
  //         * delay scale is based on update frequency, decay mode, and permitted time-at-max (should be small)
  // TODO: Physical limit switches to pull DVR8833 SLEEP low if laying down (in addition to the manual switch)
  //       Similarly, use a pulldown resistor with a GPIO as an explicit enable override to avoid spurious motor
  //       activity on startup.
  gPwmDutyAccumulator = constrain(gPwmDutyAccumulator + constrainedAccel, -gPwmMaxDuty, gPwmMaxDuty);
  if (emitDiag) {
    Serial.printf("gPwmDutyAccumulator      : %.1f\n", gPwmDutyAccumulator);
  }
  // An initial attempt to fix steering...
  // Nominally, steering behaves like a vehicle (i.e., the vehicle's path,
  // forward or backwards, bends toward the steering direction). In practice,
  // this means that the direction of rotation changes when the vehicle
  // switched between forward and backward motion.
  // However, if we need to 'twitch' opposite the direction of travel to
  // maintain balance, we want to maintain the direction of rotation.
  // Like a multi-point turn in a vehicle, this is accomplished by inverting
  // the steering direction when the direction of travel changes.  But we
  // ONLY want to do that when the direction change is corrective.
  // EXPERIMENT: I think it would be better to not simulate a 4-wheel vehicle....
  // A 2-wheel balance bot is always turning about its center.
  // Always invert steering when reversing directions
  // The challenge is near-balance...
  // To account for the dead zone, we'll also maintain rotation direction if
  // the current speed
  int intendedDirection = std::signbit(desiredSpeedNormalized);
  int activeDirection = std::signbit(gPwmDutyAccumulator);
  bool isCorrectiveReverse = intendedDirection != activeDirection;
  // abs(gPwmDutyAccumulator) > gPwmMinDuty;

  // Optional filter, smoothing the motor speed output.
  // In practice, the weight has been in the 0.9-1.0 range, making this filter have little real effect.
  // TODO: Predictive window-based outlier attenuation, but otherwise allow small variations with no additional latency?
  if (gMotorFilter <= 0.999f) {
    static float motorIIR = 0.0f;
    motorIIR = (gMotorFilter * gPwmDutyAccumulator) + (1.0f - gMotorFilter) * motorIIR;
    gPwmDutyAccumulator = motorIIR;
    if (emitDiag) {
      Serial.printf("gPwmDutyAccumulatorSm    : %.1f\n", gPwmDutyAccumulator);
    }
  }

  // Apply steering and compress the output to eliminate the dead zone
  float unsteeredDutyMagnitude = std::abs(gPwmDutyAccumulator);
  bool isUnsteeredReversed = gPwmDutyAccumulator < 0.0f;
  if (emitDiag) {
    Serial.printf("unsteeredDutyMagnitude   : %.1f\n", unsteeredDutyMagnitude);
    Serial.printf("isUnsteeredReversed      : %s\n", isUnsteeredReversed ? "true" : "false");
  }

  // Dampen the steering input to reserve sufficient capacity in each motor for correction
  // Both the trim & bias are normalized ([-1.0, 1.0])
  float effectiveYawFraction = gYawTrim + gSteeringBias * 0.6f;
  //if (isCorrectiveReverse)
  if (isUnsteeredReversed)
    effectiveYawFraction *= -1;
  if (emitDiag) {
    Serial.printf("effectiveYawFraction     : %.1f\n", effectiveYawFraction);
  }

  // Steering controls 50% of active range at rest, reduced by 75% at full speed
  // Adaptive steering *reduces* steering influence as speed increases
  // (e.g., to prevent excessively sharp turns at speed)
  float fractionOfMaxSpeed = unsteeredDutyMagnitude / gPwmMaxDuty;
  float maxSteeringDelta = ((gPwmMaxDuty - gPwmMinDuty) / 2.0f) * (1.0f - fractionOfMaxSpeed * 0.75f);
  float appliedSteeringDelta = effectiveYawFraction * maxSteeringDelta;
  if (emitDiag) {
    Serial.printf("appliedSteeringDelta     : %.1f\n", appliedSteeringDelta);
  }

  // Apply the steering, splitting the output into two independent motor channels.
  // We'll work with magnitude (independent of direction), and a flag to indicate each motor's direction.
  // This makes some of the PWM calculations a bit easier.
  // Note that we're NOT changing this math depending upon forward/backward direction...
  // Because we're working with magnitude, for a given yaw direction (e.g., +/clockwise
  // or -/counterclockwise), the same motor increases in speed. This properly simulates
  // a traditional steering approach.
  float unscaledPwmMagnitudeA = min(gPwmMaxDuty, unsteeredDutyMagnitude + appliedSteeringDelta);
  float unscaledPwmMagnitudeB = min(gPwmMaxDuty, unsteeredDutyMagnitude - appliedSteeringDelta);
  if (emitDiag) {
    Serial.printf("unscaledPwmMagnitudeA/B  : %.1f / %.1f\n", unscaledPwmMagnitudeA, unscaledPwmMagnitudeB);
  }

  // The motors' 'reverse' flags are first set by the unsteered direction,
  // but will flip when reconciling (normalizing) a negative speed.
  bool reverseA = isUnsteeredReversed;
  bool reverseB = reverseA;
  if (unscaledPwmMagnitudeA < 0.0f) {
    unscaledPwmMagnitudeA = -unscaledPwmMagnitudeA;
    reverseA = !reverseA;
  }
  if (unscaledPwmMagnitudeB < 0.0f) {
    unscaledPwmMagnitudeB = -unscaledPwmMagnitudeB;
    reverseB = !reverseB;
  }
  if (emitDiag) {
    Serial.printf("reverseA/B               : %s / %s\n", reverseA ? "true" : "false", reverseB ? "true" : "false");
    Serial.printf("unscaledPwmMagnitudeA/B. : %.1f / %.1f\n", unscaledPwmMagnitudeA, unscaledPwmMagnitudeB);
  }

  // Outputs near to zero are entirely shut down. This is the 'dead zone'.
  //   (note that because we're working with magnitude (absolute value), we
  //    don't need to check the negative side of the dead zone)
  // Otherwise, scale the output into the active region
  if (unscaledPwmMagnitudeA <= gDeadZone)
    unscaledPwmMagnitudeA = 0.0f;
  else
    unscaledPwmMagnitudeA = map(std::abs(unscaledPwmMagnitudeA), gDeadZone, gPwmMaxDuty, gPwmMinDuty, gPwmMaxDuty);

  if (unscaledPwmMagnitudeB <= gDeadZone)
    unscaledPwmMagnitudeB = 0.0f;
  else
    unscaledPwmMagnitudeB = map(std::abs(unscaledPwmMagnitudeB), gDeadZone, gPwmMaxDuty, gPwmMinDuty, gPwmMaxDuty);
  if (emitDiag) {
    Serial.printf("unscaledPwmMagnitudeA/B..: %.1f / %.1f\n", unscaledPwmMagnitudeA, unscaledPwmMagnitudeB);
  }

  // Capture the average applied magnitude, which is used as a proxy for current speed,
  // and is required by the velocity PID for speed control.
  if (reverseA == reverseB)
    gPwmDutyAppliedMagnitude = (unscaledPwmMagnitudeA + unscaledPwmMagnitudeB) / 2.0f;
  else
    gPwmDutyAppliedMagnitude = abs(unscaledPwmMagnitudeA - unscaledPwmMagnitudeB) / 2.0f;
  // if (emitDiag) {
  //   Serial.printf("gPwmDutyAppliedMagnitude : %.1f\n", gPwmDutyAppliedMagnitude);
  // }

  // -----------------------
  // Safety limiter
  // More than 100ms continuously faulted (e.g., at an unsafe angle) will shut down the motor.
  // Upon reactivation, the soft-start is reset

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
    pidFaultTimeMs = nowMs;
  }

  static bool faultLedState = false;
  if (pidFault && (nowMs - pidFaultTimeMs) > 100) {
    // In case we're experimenting without the motor enabled,
    // the onboard LED is used as a fault indicator.  The
    // transition is placed here (instead of above) to ensure
    // the LED corresponds with the effective fault treatment
    // (i.e., including the 500ms delay).
    if (!faultLedState) {
      digitalWrite(LED_PIN, HIGH);
      faultLedState = true;
    }

    // Reset a bunch of things when recovering from a fault
    gPwmDutyAccumulator = 0.0f;
    gPwmDutyAppliedMagnitude = 0.0f;
    startupPwmAttenuation = 0;
    gThrottleBias = 0.0f;
    gSteeringBias = 0.0f;
    unscaledPwmMagnitudeA = unscaledPwmMagnitudeB = 0.0f;
  } else if (!pidFault && faultLedState) {
    digitalWrite(LED_PIN, LOW);
    faultLedState = false;
  }

  // -----------------------
  // Translate duty to control signals

  // Depending upon direction, the DRV8833 needs different pins driven.
  // We also scale the magnitude to match the higher precision of the PWM duty cycle (e.g., 12 bits instead of 8)
  int ma1 = 0, ma2 = 0;
  int mb1 = 0, mb2 = 0;


  // Scale the Pwm duty to the full 12-bit precision that we've configured on the ESP32.
  // This allows for smoother control, especially in the lower end of the pwoer range.
  float scaledPwmMagnitudeA = unscaledPwmMagnitudeA * PWM_SCALE_FROM_8BIT;
  float scaledPwmMagnitudeB = unscaledPwmMagnitudeB * PWM_SCALE_FROM_8BIT;

  // Quantize for compatibility with analogWrite
  int quantizedMagnitudeA = (int)std::round(scaledPwmMagnitudeA);
  int quantizedMagnitudeB = (int)std::round(scaledPwmMagnitudeB);
  if (emitDiag) {
    Serial.printf("quantizedMagnitudeA/B    : %d, %df\n", quantizedMagnitudeA, quantizedMagnitudeB);
  }

  // Here's where we reverse each motor, if needed.
  if (reverseA) {
    ma2 = quantizedMagnitudeA;
  } else {
    ma1 = quantizedMagnitudeA;
  }

  if (reverseB) {
    mb2 = quantizedMagnitudeB;
  } else {
    mb1 = quantizedMagnitudeB;
  }

  // Adjust for braking (slow-decay) / coasting (fast-decay) drive modes.
  // For fast-decay (coasting), one pin is held low, the other is high at the desired duty.
  // For slow-decay (braking), one pin is held high, the other is low at the desired duty.
  // Because analogWrite specifies the high-side duty cycle, slow-decay mode means we invert both
  // of the duty cycles (i.e., 255 on on to hold it high, and 255-duty on the other, so that it is
  // LOW at the desired duty)
  if (gHBridgeIdleMode == eBraking) {
    // To invert the H-Bridge input, invert the levels and also
    // swap the driven IO to maintain direction.
    int m1Temp = ma1;
    ma1 = PWM_MAX - ma2;
    ma2 = PWM_MAX - m1Temp;

    m1Temp = mb1;
    mb1 = PWM_MAX - mb2;
    mb2 = PWM_MAX - m1Temp;
  }

  // Update the motor PWM.
  analogWrite(MOTORA_PIN_1, ma1);
  analogWrite(MOTORA_PIN_2, ma2);
  analogWrite(MOTORB_PIN_1, mb1);
  analogWrite(MOTORB_PIN_2, mb2);
}

void initMotors() {
  pinMode(MOTORA_PIN_1, OUTPUT);
  pinMode(MOTORA_PIN_2, OUTPUT);
  pinMode(MOTORB_PIN_1, OUTPUT);
  pinMode(MOTORB_PIN_2, OUTPUT);

  analogWriteFrequency(MOTORA_PIN_1, gPwmFreq);
  analogWriteFrequency(MOTORA_PIN_2, gPwmFreq);
  analogWriteFrequency(MOTORB_PIN_1, gPwmFreq);
  analogWriteFrequency(MOTORB_PIN_2, gPwmFreq);

  analogWriteResolution(MOTORA_PIN_1, PWM_PRECISION);  // scale all pwm output by 2^4 (16)
  analogWriteResolution(MOTORA_PIN_2, PWM_PRECISION);
  analogWriteResolution(MOTORB_PIN_1, PWM_PRECISION);
  analogWriteResolution(MOTORB_PIN_2, PWM_PRECISION);

  // Initializing the motor pins uniformly solves the 'jerk on startup' problem
  if (gHBridgeIdleMode == eBraking) {
    unsigned int pwmAlwaysOn = 255 * PWM_SCALE_FROM_8BIT;
    analogWrite(MOTORA_PIN_1, pwmAlwaysOn);
    analogWrite(MOTORA_PIN_2, pwmAlwaysOn);
    analogWrite(MOTORB_PIN_1, pwmAlwaysOn);
    analogWrite(MOTORB_PIN_2, pwmAlwaysOn);
  } else {
    analogWrite(MOTORA_PIN_1, 0);
    analogWrite(MOTORA_PIN_2, 0);
    analogWrite(MOTORB_PIN_1, 0);
    analogWrite(MOTORB_PIN_2, 0);
  }
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
  if (!std::isfinite(g.gyro.x) || !std::isfinite(g.gyro.y) || !std::isfinite(g.gyro.z) || !std::isfinite(a.acceleration.x) || !std::isfinite(a.acceleration.y) || !std::isfinite(a.acceleration.z)) {
    return;
  }

  // Update in degrees-per-second and gravities
  filter.setKp(gMahonyKp);
  filter.setKi(gMahonyKi * gMahonyKiScale);
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
