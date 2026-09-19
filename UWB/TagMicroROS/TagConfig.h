#pragma once
#include <stdint.h>

// ---- ROS 2 domain (must match ROS_DOMAIN_ID on the RDK) ----
constexpr uint32_t MICROROS_DOMAIN_ID = 1;

// UART0 is only a debug console (micro-ROS runs over WiFi/UDP).
constexpr uint32_t DEBUG_SERIAL_BAUD = 115200;

// ---- WiFi: the same hotspot the RDK is on (ESP32 = 2.4 GHz only) ----
constexpr char MICROROS_WIFI_SSID[] = "OnePlus 13 7E44";
constexpr char MICROROS_WIFI_PASSWORD[] = "YOUR_HOTSPOT_PASSWORD";
constexpr char MICROROS_AGENT_IP[] = "10.168.5.164";   // RDK X5 IP (check with: hostname -I)
constexpr uint16_t MICROROS_AGENT_PORT = 8888;         // must match: udp4 --port 8888
constexpr uint32_t WIFI_FORCE_RECONNECT_MS = 10000;

// ---- Reporting ----
constexpr uint32_t REPORT_INTERVAL_MS = 100;   // 10 Hz
constexpr uint32_t OMIT_OLDER_THAN_MS = 400;
constexpr uint16_t TAG_ANTENNA_DELAY = 16384;

// ---- Agent link supervision ----
constexpr uint32_t AGENT_PING_INTERVAL_MS = 1000;
constexpr uint32_t AGENT_RETRY_MS = 1000;
constexpr int AGENT_PING_TIMEOUT_MS = 100;
constexpr uint8_t AGENT_PING_ATTEMPTS = 2;

constexpr char MICROROS_INPUT_TOPIC[] = "/uwb3/input_json";
