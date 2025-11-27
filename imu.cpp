#include <Arduino.h>

// The Adafruit library gives us basic MPU-6050 accel and gyro data
#include <Adafruit_MPU6050.h>

// AHRS algorithms produce a stablized estimation of bearing from a 6-axis (or 9-axis) IMU.
// The Mahony algorithm is computationally inexpensive, and seems to provide sufficiently
// good results (better than a basic complementary filter)
// Other algos (Madgwick, NXP) were difficult to tune, and were not delivering better data
// for the purpose of a balance bot
#include "Adafruit_AHRS_Mahony.h"
#include "types.h"
#include "pins.h"
#include "imu.h"
#include "tuning.h"

Adafruit_MPU6050 mpu;
Adafruit_Mahony filter{};  // ...(float prop_gain, float int_gain) // Kp (was 16-ish), Ki

void initImu(ImuParams* params) {
  if (!mpu.begin()) {
    Serial.println("MPU-6050 init failed");
    while (1)
      yield();
  }

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
  params->sampleFreq = g_sample_freq;
  params->kp = 3.7f;
  params->ki = 0.3f;
  params->kiScale = 1.0f;

  // Param: samples per second
  filter.begin(params->sampleFreq);
  // THen:
  // filter.updateIMU(gx/y/z, ax/y/z, [optional dT]) // DPS (deg. per sec) / Gs
  // getRoll/Pitch/Yaw(), getGravityVector()
  // or: getQuaternion
}


void updateOrientation(ImuParams* params, ImuState* state) {
  // static unsigned long lastUpdate = 0;
  // unsigned int now = millis();
  // // Target 100 Hz, coordinated with the rate we provided to filter.begin()
  // if (now - lastUpdate < g_sample_period)
  //   return;
  // lastUpdate = now;

  sensors_event_t a, g, temp;

  // Get the accel (gravity vector) in m/s
  // Get the gyro (turn rate) in radians-per-second
  mpu.getEvent(&a, &g, &temp);
  if (!std::isfinite(g.gyro.x) || !std::isfinite(g.gyro.y) || !std::isfinite(g.gyro.z) || !std::isfinite(a.acceleration.x) || !std::isfinite(a.acceleration.y) || !std::isfinite(a.acceleration.z)) {
    state->fault = true;
    return;
  }
  state->fault = false;

  // Update in degrees-per-second and gravities
  filter.setKp(params->kp);
  filter.setKi(params->ki * params->kiScale);
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

  state->orientation.roll = filter.getRoll();
  state->orientation.pitch = filter.getPitch();
  state->orientation.yaw = filter.getYaw();
}
