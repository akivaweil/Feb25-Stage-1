/*
 * OTA_Manager.cpp - Over-The-Air Update Manager for ESP32
 * 
 * SETUP INSTRUCTIONS:
 * 1. Copy this file (OTA_Manager.cpp) and OTA_Manager.h to your project's src/ and include/ folders
 * 2. Copy the platformio.ini configuration (OTA environments)
 * 3. In your main.cpp:
 *    - #include "OTA_Manager.h"
 *    - Call initOTA() in setup()
 *    - Call handleOTA() in loop()
 * 4. Update WiFi credentials in OTA_Manager.h if different from Everwood network
 * 5. Change IP address in platformio.ini to match your ESP32's IP
 * 
 * USAGE:
 * - Use 'pio run -t upload -e freenove_esp32_wrover' for OTA uploads
 * - Monitor serial output to see IP address assigned to ESP32
 * - Update platformio.ini upload_port with the assigned IP
 * 
 * NETWORK REQUIREMENTS:
 * - ESP32 and computer must be on same network
 * - Port 3232 must be open for OTA communication
 */

#include "OTA_Manager.h"

//* ************************************************************************
//* ************************ NETWORK CONFIGURATION **********************
//* ************************************************************************
// WiFi credentials - update if using different network
const char* WIFI_SSID = "Everwood";
const char* WIFI_PASSWORD = "Everwood-Staff";

//* ************************************************************************
//* ************************ WIFI CONNECTION FUNCTIONS ******************
//* ************************************************************************

void initWiFi() {
  Serial.println("\n=== ESP32 OTA Remote Upload Setup ===");
  
  //! Step 1: Connect to WiFi with timeout
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting to WiFi");
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    yield(); // Prevent watchdog reset
    
    // Timeout after 30 seconds
    if (millis() - startTime > 30000) {
      Serial.println("\nWiFi connection timeout! Continuing without WiFi...");
      return;
    }
  }
  
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//* ************************************************************************
//* ************************ OTA SETUP FUNCTIONS ************************
//* ************************************************************************

void initOTA() {
  // Initialize WiFi first
  initWiFi();
  
  // Only proceed with OTA setup if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - skipping OTA setup");
    return;
  }
  
  //! Step 2: Configure OTA
  ArduinoOTA.setHostname("ESP32-Remote");
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  //! Step 3: Start OTA service
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.println("Device ready for remote uploads!");
  Serial.print("Use IP: ");
  Serial.println(WiFi.localIP());
}

//* ************************************************************************
//* ************************ OTA RUNTIME FUNCTIONS **********************
//* ************************************************************************

void handleOTA() {
  //! Handle OTA updates only if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }
}

void displayIP() {
  //! Display IP every 10 seconds
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 10000) {
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    lastPrint = millis();
  }
} 