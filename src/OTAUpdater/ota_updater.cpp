#include "OTAUpdater/ota_updater.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//* ************************************************************************
//* *********************** OTA UPDATER IMPLEMENTATION *********************
//* ************************************************************************
// Handles WiFi connection and Over-The-Air updates for the ESP32.

const char* ssid = "Everwood";
const char* password = "Everwood-Staff";

void setupOTA() {
  Serial.println("Booting for OTA...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("stage1-esp32s3");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }
      // NOTE: if updating SPIFFS, ensure SPIFFS is mounted via SPIFFS.begin()
      Serial.println("Start updating " + type);
      // //! Optionally, add code here to stop motors or critical processes
      // if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
      // if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
      // digitalWrite(RED_LED, HIGH); // Indicate OTA start
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      // digitalWrite(RED_LED, LOW); // Indicate OTA end
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      // digitalWrite(RED_LED, LOW); // Ensure error LED is off or use specific pattern
      // digitalWrite(YELLOW_LED, HIGH); // Indicate OTA error
    });

  ArduinoOTA.begin();

  Serial.println("OTA Initialized");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void handleOTA() {
  ArduinoOTA.handle();
} 