#include <cmath>  // sin()


#include <esp32-hal-ledc.h>

// ESP-NOW control
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "types.h"
#include "tuning.h"
#include "motor_control.h"
#include "imu.h"

using namespace EspNowRemote;

#include "pins.h"

RmtBase* remote = EspNowRemote::MakeController();

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
  initImu();

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  remote->Setup();
  remote->RegisterMsgHandler(OnControllerMessage);
  remote->Start();
}

void loop() {
  // Pump the remote's message queue, allowing rx and tx in a timely manner
  remote->Loop();

  // Handle input events as they occur, but most effects won't 
  // take effect unil the next control loop below.
  handleInput();

  // To ensure that the motor updates are always working with the freshest data possible,
  // externally synchronize updateOrientation and updateMotors.
  static unsigned long lastUpdate = 0;
  unsigned int now = millis();
  if (now - lastUpdate >= g_sample_period) {
    lastUpdate = now;

    updateOrientation();
    // updateBattery and updateRemoteDisplay are not as time-sensitive, and are
    // internally throttled to run less frequently than the main control functions
    updateBattery();
    updateRemoteDisplay(remote);

    updateMotors();
  }
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

