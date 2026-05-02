// CONFIG HEADER FILE
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h> // non-volatile memory
#include <Wire.h>

// Zenoh-specific parameters
#define MODE "client"

#define KEYEXPR "joint_commands"       // ros topic jointstates
#define IMU_KEYEXPR "imu/data_raw"     // ros topic imu
#define MAG_KEYEXPR "imu/mag"          // ros topic magnetic field
#define ENCODER_KEYEXPR "joint_states" // ros topic encoders feedback

// to send data to the esp32
#define LED_PIN 2

#endif