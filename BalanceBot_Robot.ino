// ESP-NOW control
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "EspNowRemote.h"

#include "types.h"
#include "tuning.h"
#include "battery_sense.h"
#include "motor_control.h"
#include "imu.h"

#include "pins.h"

using namespace EspNowRemote;
RmtBase* remote = EspNowRemote::MakeController();

BalanceState state {};

void setup() {
  Serial.begin(115200);

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

  // The on-board LED is used for fault announcement
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  initBatterySense();
  initMotors(&state.driveParams, &state.driveState, &state.pitchPidParams);
  initImu(&state.imuParams);
  initInput(remote);

}

void loop() {
  // Pump the remote's message queue, allowing rx and tx in a timely manner
  remote->Loop();

  // Handle input events as they occur, but most effects won't 
  // take effect unil the next control loop below.
  handleInput(&state);

  // To ensure that the motor updates are always working with the freshest data possible,
  // externally synchronize updateOrientation and updateMotors.
  static unsigned long lastUpdate = 0;
  unsigned int now = millis();
  if (now - lastUpdate >= g_sample_period) {
    lastUpdate = now;

    updateOrientation(&state.imuParams, &state.imuState);
    // updateBattery and updateRemoteDisplay are not as time-sensitive, and are
    // internally throttled to run less frequently than the main control functions
    updateBatterySense(&state);
    updateRemoteDisplay(remote, &state);

    updateMotors(&state);
  }
}

