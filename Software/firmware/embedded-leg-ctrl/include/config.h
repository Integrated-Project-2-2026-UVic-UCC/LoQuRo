// CONFIG HEADER FILE
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>

// #define SSID "TP-Link_BF98"
// #define PASS "59429128"
#define SSID "iPhone de Nico "
#define PASS "almohada"
// #define SSID "Red WiFI"
// #define PASS "Password"

// Zenoh-specific parameters
#define MODE "client"
#define LOCATOR "tcp/172.20.10.10:7447"
// #define LOCATOR "tcp/192.168.0.36:7447" // If empty, scout
#define LOCATOR ""

#define KEYEXPR "joint_commands" // ros topic

// to send data to the esp32
#define LED_PIN 2

#endif