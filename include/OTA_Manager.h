#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

//* ************************************************************************
//* ************************ NETWORK CONFIGURATION **********************
//* ************************************************************************
// Update these credentials if using different network
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;

//* ************************************************************************
//* ************************ FUNCTION DECLARATIONS **********************
//* ************************************************************************

// WiFi connection functions
void initWiFi();

// OTA setup and management functions
void initOTA();
void handleOTA();
void displayIP();

#endif 