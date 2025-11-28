#include <Arduino.h>
#include "tuning.h"
#include "pins.h"
#include "battery_sense.h"

void initBatterySense(BatteryState* batteryState) {
  batteryState->voltage = 0.0f;
  batteryState->voltagePercent = 0.0f;
  pinMode(BATTERY_SENSE_PIN, INPUT);
}

void updateBatterySense(BatteryState* batteryState, DriveParams* driveParams) {
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
    batteryState->voltage = voltageIIR;
    const float lowLevel = 2 * 3.4f;
    const float highLevel = 2 * 4.2f;
    batteryState->voltagePercent = max(0.0f, (batteryState->voltage - lowLevel) / (highLevel - lowLevel) * 100.0f);

    // Auto-adjust the max throttle to limit nominal speed to a 5V average without prohibiting corrective spikes
    if (batteryState->voltage >= 5.0f) {
      driveParams->maxThrottleBias = min(driveParams->pwmMaxDuty / 255.0f * 0.7f, (5.0f / batteryState->voltage));
    }
  }
}
