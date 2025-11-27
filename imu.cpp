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
#include "tuning.h"
#include "pins.h"
#include "imu.h"

Adafruit_MPU6050 mpu;
Adafruit_Mahony filter{};  // ...(float prop_gain, float int_gain) // Kp (was 16-ish), Ki

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


void updateOrientation() {
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
    gImuFault = true;
    return;
  }
  gImuFault = false;

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
