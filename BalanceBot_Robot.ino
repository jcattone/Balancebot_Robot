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

// The Adafruit library gives us basic MPU-6050 accel and gyro data
#include <Adafruit_MPU6050.h>

// We'll use an SSD1306 128x32 display in lieu of serial output,
// which can interfere with the I2C bus
#include <Adafruit_SSD1306.h>

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

RmtBase* remote = EspNowRemote::MakeController();
joystick_state_t g_joystick_state = {};

// Initialize the IMU filter weights:
// Kp ~= trust accel data (gravity vector) to correct gyro drift
//   High Kp = correct quickly, but linear acceleration (i.e., translation) can be misread as orientation change
//   Low Kp = trust the gyro more, can be sluggish to respond
//   i.e., choose lowest Kp with acceptable linear acceleration characteristics
// Ki ~= correction speed for gyro bias (from accel gravity reference)
//   High Ki = aggressively remove bias, but can become sluggish.  In practice, overshoot is observed.
//   Low Ki = may allow bias to persistently affect the output
// In practice, I'm seeing eyeball-reasonable results with Kp=10..25, and Ki=0..5
Adafruit_Mahony filter(20, 4);  // ...(float prop_gain, float int_gain) // Kp, Ki

// The IMU instance itself
Adafruit_MPU6050 mpu;

// The display... Assumed to be on the default I2C pins
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_CHAR_WIDTH 21  // 5+1 pixel font width
#define SCREEN_HEIGHT 32      // OLED display height, in pixels
#define SCREEN_HEIGHT_ROWS 4  // 7+1 pixel font height
#define OLED_RESET -1         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
char textBuffer[SCREEN_HEIGHT_ROWS][SCREEN_CHAR_WIDTH];

eConfigMode gCurrentConfigMode = eFusionKp;

eHBridgeIdleMode gHBridgeIdleMode = eBraking;


#define MOTOR_PIN_1 18
#define MOTOR_PIN_2 19


int gPwmMinDuty = 20;
int gPwmMaxDuty = 160;
int gPwmCurrentDuty = 0;
int gPwmDutyMagnitude = 0;
int gPwmFreq = 25;

float gPidKp = 1.0f;
float gPidKi = 0.5f;
float gPidKd = 0.0f;

void OnDataSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t sendStatus) {
  // TODO: Plumb this back into the remote
}

// All received ESP-NOW traffic arrives here, and is funnelled into the remote.
void IRAM_ATTR OnDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
  Serial.printf("ESPNOW, OnDataRecv => %d bytes\n", data_len);
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
      Serial.println((const char*) data);
      break;

    case MSGTYPE_RMT_JOYSTICK:
      memcpy(&g_joystick_state, data, data_len);
      last_hid_input_timestamp = micros();
      break;
  }
  return true;
}

void setup() {
  Serial.begin(38400);
  // while (!Serial);
  Serial.println("MPU6050 demo");

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

  // TODO: initMotors()
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);

  analogWriteFrequency(MOTOR_PIN_1, gPwmFreq);
  analogWriteFrequency(MOTOR_PIN_2, gPwmFreq);
  // Initializing the motor pins uniformly solves the 'jerk on startup' problem
  if (gHBridgeIdleMode == eBraking) {
    analogWrite(MOTOR_PIN_1, 1);
    analogWrite(MOTOR_PIN_2, 1);
  } else {
    analogWrite(MOTOR_PIN_1, 0);
    analogWrite(MOTOR_PIN_2, 0);
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
  updateMotors();
}

void updateMotors() {
  static unsigned long lastUpdate = 0;

  static int lastPos = 90;
  unsigned long now = millis();
  if (now - lastUpdate < 20)
    return;
  lastUpdate = now;

  // PID per-update inputs
  int desiredAngle = 0.0f;
  // [-90, -90] generally speaking (technically, can go to +/- 180)
  float currentAngle = gOrientation.roll;

  // Calculate dT in seconds (the actual time unit is arbitrary, as long as we're consistent)
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

  // How far off are we?
  // TODO: currentAngleAdjusted is where we could subtract the
  // angle error predicted from the current acceleration.
  // This is sometimes known as 'covariance' adjustment,
  // compensating for the acceleration's impact on the
  // apparent gravity vector.
  float currentAngleAdjusted = currentAngle;
  // TODO: TBD whether this error has the correct sign - we may
  // need to flip this to change the direction of the feedback.
  float errorAngle = currentAngleAdjusted - desiredAngle;

  static int lastErrorSign = 0;
  int currentErrorSign = std::signbit(errorAngle);

  // P: [-90, 90] typical
  float P = errorAngle;

  // I: ?
  static float errorIntegral = 0.0;

  // Clear the integral when we cross the setpoint (don't carry windup across a stable threshold).
  // Note that this doesn't account for inertia - we might wind up crossing rapidly - this suggests
  // a use for the derivative term.
  if (currentErrorSign != lastErrorSign)
    errorIntegral = 0;

  // Only accumulate error when we aren't saturated
  if (std::abs(gPwmCurrentDuty) < gPwmMaxDuty) {
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
  static float errorDeltaPerSecondIIRWeight = 0.60f;
  float errorDeltaPerSecond = (errorAngle - lastErrorAngle) / deltaTSec;
  // IIR; weight the accumulator heavily, and the new value lightly.
  errorDeltaPerSecondIIR = errorDeltaPerSecondIIRWeight * errorDeltaPerSecondIIR + (1.0f - errorDeltaPerSecondIIRWeight) * errorDeltaPerSecond;
  //Serial.printf("eA:%.1f,lea:%.1f,dT:%.3f,edps:%.1f,edpsIIR:%.1f\n", errorAngle, lastErrorAngle, deltaTSec, errorDeltaPerSecond, errorDeltaPerSecondIIR);
  lastErrorAngle = errorAngle;
  float D = errorDeltaPerSecondIIR;

  float PIDOutput = gPidKp * P + gPidKi * I + gPidKd * D;

  // Note that we want the acceleration (not speed) to be modulated by the PID output.
  // TODO: Note that this is an integer, limiting fine-grained control especially close to 0
  int rawAccel = constrain(PIDOutput, -255.0f, 255.0f);

  // Compress the speed range to the PWM min/max range
  //int constrainedAccel = map((int)std::abs(rawAccel), -gPwmMaxDuty, gPwmMaxDuty, gPwmMinDuty, gPwmMaxDuty);

  // Serial.printf("cA:%.1f,errA:%.1f,P:%.1f,I:%.1f,D:%.1f,PID:%.1f\n", currentAngle, errorAngle, P, I, D, PIDOutput);

  // Attenuate the PWM output on startup to prevent noisy output
  static int startupPwmAttenuation = 0;
  int constrainedAccel = constrain(rawAccel, -startupPwmAttenuation, startupPwmAttenuation);
  if (startupPwmAttenuation < 255)
    ++startupPwmAttenuation;

  // Adjust the speed by the desired relative acceleration, constraining the duty cycle to the PWM limits.
  // Note that this can be negative, to indicate a reversed direction.
  gPwmCurrentDuty = constrain(gPwmCurrentDuty + constrainedAccel, -gPwmMaxDuty, gPwmMaxDuty);

  // Now compress the output to eliminate the dead zone.
  // TODO: more nuanced mapping (e.g., near-zero dead-zone + fast-ramp)
  gPwmDutyMagnitude = map(std::abs(gPwmCurrentDuty), 0, gPwmMaxDuty, gPwmMinDuty, gPwmMaxDuty);

  // Depending upon direction, the DRV8833 needs different pins driven.
  bool m1Driven;
  int m1 = 0, m2 = 0;
  if (gPwmCurrentDuty < 0) {
    // Note: for fast-decay (coasting), one pin is pulled low, the other is high at the desired duty.
    // For slow-decay (braking), one pin is pulled high, the other is low at the desired duty.
    // Because analogWrite specified the high-side duty cycle, slow-decay mode means we invert both
    // of the duty cycles (i.e., 255 on on to hold it high, and 255-duty on the other, so that it is
    // LOW at the desired duty)
    m2 = gPwmDutyMagnitude;
    m1Driven = false;
  } else {
    m1 = gPwmDutyMagnitude;
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
  analogWrite(MOTOR_PIN_1, m1);
  analogWrite(MOTOR_PIN_2, m2);
}

void initDisplay() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();
  delay(20);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
}

void initImu() {
  if (!mpu.begin()) {
    Serial.println("MPU-6050 init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");

  // Param: samples per second
  filter.begin(100);
  // THen:
  // filter.updateIMU(gx/y/z, ax/y/z, [optional dT]) // DPS (deg. per sec) / Gs
  // getRoll/Pitch/Yaw(), getGravityVector()
  // or: getQuaternion
}

void handleInput() {
  static unsigned long lastUpdate = 0;
  static bool isDebouncing = false;
  unsigned int now = millis();
  if (now - lastUpdate < 20)
    return;
  lastUpdate = now;
  int joystickY = 2048, joystickX = 2048;

  // USE_ESPNOW
  static unsigned long last_hid_message_processed = 0;
  if (last_hid_input_timestamp > last_hid_message_processed) {
    last_hid_message_processed = last_hid_input_timestamp;
    if (g_joystick_state.count > 0) {
      switch (g_joystick_state.joystick_direction) {
        case JOYSTICK_UP:
          joystickY = 4096;
          break;
        case JOYSTICK_DOWN:
          joystickY = 0;
          break;
        case JOYSTICK_RIGHT:
          joystickX = 4096;
          break;
        case JOYSTICK_LEFT:
          joystickX = 0;
          break;
      }
      // 'consume' the event
      g_joystick_state.count = 0;
    }
    // ; // last_hid_input_timestamp
  }

  // Config mode
  if (isDebouncing) {
    if (joystickY > 1000 && joystickY < 3000)
      isDebouncing = false;
  } else {
    if (joystickY < 100) {
      if (gCurrentConfigMode == 0)
        gCurrentConfigMode = (eConfigMode)(eMaxConfigMode);
      gCurrentConfigMode = (eConfigMode)(gCurrentConfigMode - 1);
      isDebouncing = true;
    } else if (joystickY > 3900) {
      gCurrentConfigMode = (eConfigMode)(gCurrentConfigMode + 1);
      if (gCurrentConfigMode == eMaxConfigMode)
        gCurrentConfigMode = (eConfigMode)0;
      isDebouncing = true;
    }
  }

  // Actual config
  bool up = joystickX > 3900;
  bool down = joystickX < 100;
  switch (gCurrentConfigMode) {
    case eFusionKp:
      if (up) filter.setKp(min(100.0, filter.getKp() + 0.1));
      else if (down) filter.setKp(max(0.0, filter.getKp() - 0.1));
      break;
    case eFusionKi:
      if (up) filter.setKi(min(100.0, filter.getKi() + 0.1));
      else if (down) filter.setKi(max(0.0, filter.getKi() - 0.1));
      break;
    case ePwmMin:
      if (up) gPwmMinDuty = min(gPwmMaxDuty, gPwmMinDuty + 1);
      else if (down) gPwmMinDuty = max(0, gPwmMinDuty - 1);
      break;
    case ePwmMax:
      if (up) gPwmMaxDuty = min(255, gPwmMaxDuty + 1);
      else if (down) gPwmMaxDuty = max(gPwmMinDuty, gPwmMaxDuty - 1);
      break;
    case ePwmFreq:
      if (up) gPwmFreq = min(40000, max(gPwmFreq + 1, (int)(gPwmFreq * 1.1)));
      else if (down) gPwmFreq = max(10, min(gPwmFreq - 1, (int)(gPwmFreq / 1.1)));
      if (up || down) {
        analogWriteFrequency(MOTOR_PIN_1, gPwmFreq);
        analogWriteFrequency(MOTOR_PIN_2, gPwmFreq);
      }
      break;
    case eHBridgeIdle:
      if (up)
        gHBridgeIdleMode = eCoasting;
      else if (down)
        gHBridgeIdleMode = eBraking;
      break;
    case ePidKp:
      adjustPidK(&gPidKp, up ? 0.01f : (down ? -0.01f : 0.0f));
      break;
    case ePidKi:
      adjustPidK(&gPidKi, up ? 0.01f : (down ? -0.01f : 0.0f));
      break;
    case ePidKd:
      adjustPidK(&gPidKd, up ? 0.01f : (down ? -0.01f : 0.0f));
      break;
  }
}

void adjustPidK(float* f, float delta) {
  *f += delta;
  if (*f < 0.0f)
    *f = 0.0f;
}

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  unsigned int now = millis();
  if (now - lastUpdate < 10)
    return;
  lastUpdate = now;

  display.clearDisplay();

  // Rows 1-3: Orientation
  display.setCursor(0, 0);
  display.printf("Roll:%.2f\nPitch:%.2f\nYaw:%.2f", gOrientation.roll, gOrientation.pitch, gOrientation.yaw);

  // Row 4: current config
  display.setCursor(0, 3 * 8);
  switch (gCurrentConfigMode) {
    case eFusionKp:
      display.printf("Kp=[%.1f] Ki=%.1f", filter.getKp(), filter.getKi());
      break;
    case eFusionKi:
      display.printf("Kp=%.1f Ki=[%.1f]", filter.getKp(), filter.getKi());
      break;
    case ePwmMin:
      display.printf("pwm=[%d]-%d (%d)", gPwmMinDuty, gPwmMaxDuty, gPwmCurrentDuty);
      break;
    case ePwmMax:
      display.printf("pwm=%d-[%d] (%d)", gPwmMinDuty, gPwmMaxDuty, gPwmCurrentDuty);
      break;
    case ePwmFreq:
      display.printf("pwmFreq=[%d] (%d)", gPwmFreq, gPwmCurrentDuty);
      break;
    case eHBridgeIdle:
      if (gHBridgeIdleMode == eBraking)
        display.printf("Idle=[B] /  C ");
      else
        display.printf("Idle= B  / [C]");
      break;
    case ePidKp:
      display.printf("PID [%.2f] %.2f %.2f", gPidKp, gPidKi, gPidKd);
      break;
    case ePidKi:
      display.printf("PID %.2f [%.2f] %.2f", gPidKp, gPidKi, gPidKd);
      break;
    case ePidKd:
      display.printf("PID %.2f %.2f [%.2f]", gPidKp, gPidKi, gPidKd);
      break;
  }

  display.display();
}

void updateOrientation() {
  static unsigned long lastUpdate = 0;
  unsigned int now = millis();
  if (now - lastUpdate < 10)
    return;
  lastUpdate = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Update in degrees-per-second and gravities
  filter.update(
    // From radians-per-second to degrees-per-second
    g.gyro.x * 180.0 / M_PI,
    g.gyro.y * 180.0 / M_PI,
    g.gyro.z * 180.0 / M_PI,
    // From m/s to gravities
    a.acceleration.x / 9.81,
    a.acceleration.y / 9.81,
    a.acceleration.z / 9.81,
    0.0,
    0.0,
    0.0);

  gOrientation.roll = filter.getRoll();
  gOrientation.pitch = filter.getPitch();
  gOrientation.yaw = filter.getYaw();
}
