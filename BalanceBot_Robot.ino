// ESP-NOW control requires WiFi support
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>

// Private library
#include "EspNowRemote.h"

// Application modules
#include "types.h"
#include "tuning.h"
#include "battery_sense.h"
#include "motor_control.h"
#include "imu.h"

// Hardware config
#include "pins.h"

// Persistent state
EspNowRemote::RmtBase* remote = EspNowRemote::MakeController();
BatteryState batteryState{};
ImuConfig imuConfig{};
// TODO: Multiple motor configs / manual or auto selection based on goal. A/B test w/ persist/reset?
MotorConfig motorConfig{};
RemoteInput remoteInput{};

// TODO: ToF or ultasonic sensors facing front/back - poll one based upon motor direction. Avoid false signal from ground when pitched down?
// TODO: Tire guards / feeler switches, to avoid direct non-ground wheel contact?
// TODO: Retune for third tier

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

  // Module init
  initBatterySense(&batteryState);
  initMotors(&motorConfig);
  initImu(&imuConfig);
  initInput(remote, &remoteInput);

  Wire.setClock(400000);
}

void loop() {
  // Pump the remote's message queue, allowing rx and tx in a timely manner
  remote->Loop();

  // Handle input events as they occur, but most effects won't
  // take effect unil the next control loop below.
  handleInput(&remoteInput, &motorConfig, &imuConfig, &batteryState);

  // To ensure that the motor updates are always working with the freshest data possible,
  // externally synchronize updateOrientation and updateMotors.
  static unsigned long lastUpdate = 0;
  unsigned int now = millis();
  if (now - lastUpdate >= UPDATE_PERIOD) {
    lastUpdate = now;

    updateOrientation(&imuConfig);
    // updateBattery and updateRemoteDisplay are not as time-sensitive, and are
    // internally throttled to run less frequently than the main control functions
    updateBatterySense(&batteryState, &motorConfig.driveParams);
    updateRemoteDisplay(remote, &remoteInput, &motorConfig, &imuConfig, &batteryState);

    updateMotors(&motorConfig, &imuConfig);
  }
}
