// 
// CONFIG
// 

#include "Ethernet.h"

// #define SERIAL_PRINT
// #define USE_WATCHDOG
#define USE_WATCHDOG_I2C
#define RESET_OTGW_ON_BOOT
// #define WIFI_BSSID
// #define STATIC_IP

static const char wifi_ssid[] PROGMEM         = "WIFI_SSID";
static const char wifi_password[] PROGMEM     = "WIFI_PASSWORD";
#ifdef WIFI_BSSID
static const uint8_t wifi_bssid[6] PROGMEM    = {};
static const int32_t wifi_channel PROGMEM     = -1;
#endif
static const char esp_update_url[] PROGMEM    = "UPDATE_URL";

#ifdef STATIC_IP
  IPAddress host(192, 0, 2, 10);
  IPAddress gateway(192, 0, 2, 1);
  IPAddress netmask(255, 255, 255, 0);
  IPAddress dns1(192, 0, 2, 1);
#endif
