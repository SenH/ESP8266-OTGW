//
// ESP8266 serial wifi bridge
//

#include "Config.h"

#include "lwip/init.h"
#include "core_version.h"
#include "ESP8266WiFi.h"
#include "ESP8266HTTPClient.h"
#include "ESP8266httpUpdate.h"
#include "Wire.h"
#include "Ticker.h"

int rssi_to_percent(float rssi);

#if LWIP_VERSION_MAJOR >= 2 
// https://github.com/esp8266/Arduino/issues/2330#issuecomment-431280122
#include "PingAlive.h"
#endif

#ifdef SERIAL_PRINT
  #define s_print(x) Serial.print(x)
  #define s_println(x) Serial.println(x)
  #define s_printf_P(format,...) Serial.printf_P(format, __VA_ARGS__)
#else
  #define s_print(x)
  #define s_println(x)
  #define s_printf_P(...)
#endif

#ifdef USE_WATCHDOG
  #define s_wd_reset() wd_reset()
#else
  #define s_wd_reset()
#endif

#ifdef USE_WATCHDOG_I2C
  #define s_trig_reset_wd_i2c(x) trig_reset_wd_i2c(x)
#else
  #define s_trig_reset_wd_i2c(x)
#endif

// Retrieve const char* from PROGMEM pointer
#define FPCC(x) (String(FPSTR(x)).c_str())

// 
// GLOBALS
// 

static const byte port PROGMEM                          = 23;
static const byte max_clients PROGMEM                   = 2;
static const int baud_rate PROGMEM                      = 9600;

static const byte esp_led_pin PROGMEM                   = 2;
static const byte node_led_pin PROGMEM                  = 16;
static const byte otgw_reset_pin PROGMEM                = 14;
static const byte wd_i2c_address PROGMEM                = 38;

Ticker ticker_uptime;
unsigned long uptime                                    = 0; // Counting every second until 4294967295 = 130 years
unsigned long wifi_connect_ts                           = 0;

WiFiServer otgw_server(port);
WiFiServer esp_server(port + 1);

WiFiClient otgw_clients[max_clients];
WiFiClient esp_clients[max_clients];
int wifi_led                                            = HIGH;

unsigned long wifi_connect_timer                        = 0;
static const unsigned long wifi_connect_timeout PROGMEM = 60000;

unsigned long wd2_timer                                 = 0;
static const unsigned long wd2_interval PROGMEM         = 1000;

static const char EOL[] PROGMEM                         = "\r\n";
static const char USAGE[] PROGMEM                       = "$SYS $MEM $NET $WIF $PNG $UPD $RST ESP|OTGW $HLP";

// 
// FUNCTIONS
// 

#if LWIP_VERSION_MAJOR >= 2 
void pingFault ()
{
  s_println(F("PNG > Gateway not responding to pingAlive"));
}
#endif

void reset_wd_i2c() {
  int ret = -1;
  Wire.beginTransmission(wd_i2c_address);
  Wire.write(0xA5);
  ret = Wire.endTransmission();

  if (ret == 0) {
    s_println(F("WD > Reset WDT_I2C"));
  } else {
    s_println(F("WD > Failed reset WDT_I2C"));
  }
}

void trig_reset_wd_i2c(unsigned long now) {
  if (now - wd2_timer >= wd2_interval) {
    wd2_timer = now;
    reset_wd_i2c();
  }
}

void reset_otgw() {
  pinMode(otgw_reset_pin, OUTPUT);
  digitalWrite(otgw_reset_pin, LOW);
  delay(500);
  digitalWrite(otgw_reset_pin, HIGH);
  pinMode(otgw_reset_pin, INPUT_PULLUP);
  s_println(F("OTGW > Reset complete"));
}

String get_net_info() {
  String s;
  s += F(" IP: ");
  s += WiFi.localIP().toString();
  s += FPSTR(EOL);
  s += F("SUB: ");
  s += WiFi.subnetMask().toString();
  s += FPSTR(EOL);
  s += F(" GW: ");
  s += WiFi.gatewayIP().toString();
  s += FPSTR(EOL);
  s += F("DNS: ");
  s += WiFi.dnsIP().toString();
  s += F(", ");
  s += WiFi.dnsIP(1).toString();
  s += FPSTR(EOL);
  s += F("MAC: ");
  s += WiFi.macAddress();
  
  return s;
}

void connect_to_wifi() {
  // https://github.com/arendst/Sonoff-Tasmota/blob/development/sonoff/support_wifi.ino#L210
  s_println(F("WiFi > Initialize"));
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(200);
  
  WiFi.mode(WIFI_STA);
  // Sleep is buggy in core 2.4, disable for now
  // https://github.com/arendst/Sonoff-Tasmota/blob/development/sonoff/support_wifi.ino#L184
  // WiFi.setSleepMode(WIFI_NONE_SLEEP);
#ifdef WIFI_BSSID
  uint8_t *bssid = new uint8_t[6];
  if (bssid) {
    memcpy_P(bssid, wifi_bssid, 6);
  }
  WiFi.begin(FPCC(wifi_ssid), FPCC(wifi_password), wifi_channel, bssid);
#else
  WiFi.begin(FPCC(wifi_ssid), FPCC(wifi_password));
#endif
#ifdef STATIC_IP
  WiFi.config(host, gateway, netmask, dns1);
#endif
  
  // Wait for WIFI connection
  wifi_connect_timer = millis();
  while (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    s_println(F("WiFi > Connecting..."));
    s_trig_reset_wd_i2c(now);
    s_wd_reset();

    // Toggle LED
    wifi_led = (wifi_led == LOW) ? HIGH : LOW;
    digitalWrite(esp_led_pin, wifi_led);
    delay(500);

    // Reset ESP after timeout
    if (now - wifi_connect_timer >= wifi_connect_timeout) {
      s_printf_P(PSTR("WiFi > Failed connecting after %d seconds!"), (wifi_connect_timeout / 1000));
      s_println(F("ESP > Reset now"));
      ESP.reset();
    }
  }
  
  wifi_connect_ts = uptime;
  s_printf_P(PSTR("WiFi > Connected after %d seconds%s"), (millis() - wifi_connect_timer) / 1000, FPCC(EOL));
#ifdef SERIAL_PRINT
  WiFi.printDiag(Serial);
#endif
  s_printf_P(PSTR("BSSID: %s%s"), WiFi.BSSIDstr().c_str(), FPCC(EOL));
  s_printf_P(PSTR("RSSI: %d dBm (%d%%)%s"), WiFi.RSSI(), rssi_to_percent(WiFi.RSSI()), FPCC(EOL));
  s_println(get_net_info());
}

void handle_server_clients(WiFiServer server, WiFiClient clients[]) {
  uint8_t i;
  if (server.hasClient()) {
    for (i = 0; i < max_clients; i++) {
      // Find free spot
      if (!clients[i] || !clients[i].connected()) {
        if (clients[i]) {
          clients[i].stop();
        }
        clients[i] = server.available();
        s_print(F("SRV > New client: "));
        s_println(i);
        break;
      }
    }
    // No free spot so reject
    if (i == max_clients) {
      WiFiClient a_client = server.available();
      a_client.stop();
      s_println(F("SRV > Max clients exceeded. Rejecting!"));
    }
  }
}

// https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266httpUpdate/examples/httpUpdate/httpUpdate.ino
void do_http_update(WiFiClient client) {
#ifdef ARDUINO_ESP8266_RELEASE_2_4_2
  t_httpUpdate_return ret = ESPhttpUpdate.update(FPCC(esp_update_url));
#else
  WiFiClient OTAclient;
  ESPhttpUpdate.setLedPin(node_led_pin, LOW);
  t_httpUpdate_return ret = ESPhttpUpdate.update(OTAclient, FPCC(esp_update_url));
#endif
  
  switch (ret) {
    case HTTP_UPDATE_FAILED:
    client.printf_P(PSTR("HTTP update error %d, %s%s"), ESPhttpUpdate.getLastError(), 
      ESPhttpUpdate.getLastErrorString().c_str(), FPCC(EOL));
    break;

    case HTTP_UPDATE_NO_UPDATES:
    client.println(F("HTTP no update available"));
    break;

    case HTTP_UPDATE_OK:
    client.println(F("HTTP update OK"));
    break;
  }
}

int rssi_to_percent(float rssi) {
// https://stackoverflow.com/questions/15797920/
  int i = round(rssi);
  i = 2 * (i + 100);
  i = max(i, 0);
  return min(i, 100);
}

void parse_esp_cmd(WiFiClient client) {
  String cmd;
  cmd.reserve(16);

  client.setTimeout(1); // Max waiting time for readStringUntil()
  cmd = client.readStringUntil('\r');
  client.readStringUntil('\n'); // Discard LF

  if (cmd.length() < 4) {
    client.println(FPSTR(USAGE));
    return;
  }
  
  if (cmd.equals(F("$SYS"))) {
    client.println(ESP.getFullVersion());
    client.printf_P(PSTR("    ESP uptime: %lu seconds%s"), uptime, FPCC(EOL));
    client.printf_P(PSTR("   WiFi uptime: %lu seconds%s"), (uptime-wifi_connect_ts), FPCC(EOL));
    client.printf_P(PSTR("Restart reason: %s%s"), ESP.getResetReason().c_str(), FPCC(EOL));
  } else if (cmd.equals(F("$MEM"))) {
#ifdef ARDUINO_ESP8266_RELEASE_2_4_2
    client.printf_P(PSTR("Free: %d bytes%s"), ESP.getFreeHeap(), FPCC(EOL));
#else
    client.printf_P(PSTR("Free: %d bytes Fragmentation: %d%%%s"), ESP.getFreeHeap(), ESP.getHeapFragmentation(), FPCC(EOL));
#endif
  } else if (cmd.equals(F("$NET"))) {
    client.println(get_net_info());
  } else if (cmd.equals(F("$WIF"))) {
    WiFi.printDiag(client);
    client.printf_P(PSTR("BSSID: %s%s"), WiFi.BSSIDstr().c_str(), FPCC(EOL));
    client.printf_P(PSTR("RSSI: %d dBm (%d%%)%s"), WiFi.RSSI(), rssi_to_percent(WiFi.RSSI()), FPCC(EOL));
#if LWIP_VERSION_MAJOR >= 2 
  } else if (cmd.equals(F("$PNG"))) {
    client.printf_P(PSTR("PingAlive tx %5d rx %5d%s"), ping_seq_num_send, ping_seq_num_recv, FPCC(EOL));
#endif
  } else if (cmd.equals(F("$UPD"))) {
    client.printf_P(PSTR("Update ESP via %s%s"), FPCC(esp_update_url), FPCC(EOL));
    do_http_update(client);
  } else if (cmd.equals(F("$RST ESP"))) {
    client.println(F("ESP reset"));
    delay(100);
    ESP.reset();
  } else if (cmd.equals(F("$RST OTGW"))) {
    client.println(F("OTGW reset"));
    reset_otgw();
    client.println(F("OTGW reset complete"));
  } else if (cmd.equals(F("$HLP"))) {
    client.println(FPSTR(USAGE));
  } else {
    client.println(FPSTR(USAGE));
  }
}

void uptime_inc() {
  uptime++;
}

// 
// MAIN
// 

void setup(void) {
  // Increment uptime every second
  ticker_uptime.attach(1.0, uptime_inc);

  Serial.begin(baud_rate);
  s_println(ESP.getFullVersion());
  s_print(F("ESP > Restart reason: "));
  s_println(ESP.getResetReason());
  
#ifdef USE_WATCHDOG
  wd_enable(1000);
  s_println(F("WD > Enable ESP8266 WD"));
#endif

#ifdef USE_WATCHDOG_I2C
  Wire.begin();
#endif

  // Set LEDS OFF
  pinMode(esp_led_pin, OUTPUT);
  pinMode(node_led_pin, OUTPUT);

  digitalWrite(esp_led_pin, HIGH);
  digitalWrite(node_led_pin, HIGH);
  
#ifdef RESET_OTGW_ON_BOOT
  reset_otgw();
#endif

  connect_to_wifi();
#if LWIP_VERSION_MAJOR >= 2 
  startPingAlive();
#endif
 
  esp_server.begin();
  esp_server.setNoDelay(true);

  otgw_server.begin();
  otgw_server.setNoDelay(true);
}

void loop(void) {
  unsigned long now = millis();
  uint8_t i;

  s_wd_reset();
  s_trig_reset_wd_i2c(now);
  
  // Check WiFi status
  if(WiFi.status() != WL_CONNECTED) {
    connect_to_wifi();
  }

  // Set LEDS off
  digitalWrite(esp_led_pin, HIGH);
  digitalWrite(node_led_pin, HIGH);

  // Get new ESP clients
  if (esp_server.hasClient()) {
    for (i = 0; i < max_clients; i++) {
      // Find free spot
      if (!esp_clients[i] || !esp_clients[i].connected()) {
        if (esp_clients[i]) {
          esp_clients[i].stop();
        }
        esp_clients[i] = esp_server.available();
        s_print(F("SRV > New client: "));
        s_println(i);
        break;
      }
    }
    // No free spot so reject
    if (i == max_clients) {
      WiFiClient a_client = esp_server.available();
      a_client.stop();
      s_println(F("SRV > Max clients exceeded. Rejecting!"));
    }
  }
  
  // Get new OTGW clients
  if (otgw_server.hasClient()) {
    for (i = 0; i < max_clients; i++) {
      // Find free spot
      if (!otgw_clients[i] || !otgw_clients[i].connected()) {
        if (otgw_clients[i]) {
          otgw_clients[i].stop();
        }
        otgw_clients[i] = otgw_server.available();
        s_print(F("SRV > New client: "));
        s_println(i);
        break;
      }
    }
    // No free spot so reject
    if (i == max_clients) {
      WiFiClient a_client = otgw_server.available();
      a_client.stop();
      s_println(F("SRV > Max clients exceeded. Rejecting!"));
    }
  }
  
  // handle_server_clients(esp_server, esp_clients);
  // handle_server_clients(otgw_server, otgw_clients);
  
  // ESP commands
  for (i = 0; i < max_clients; i++) {
    // https://github.com/esp8266/Arduino/issues/5257#issuecomment-430950163
   while (esp_clients[i] && esp_clients[i].available()) {
      if (esp_clients[i].available()) {
        parse_esp_cmd(esp_clients[i]);
      }
    }
  }

  // OTGW Telnet -> Serial
  for (i = 0; i < max_clients; i++) {
    // https://github.com/esp8266/Arduino/issues/5257#issuecomment-430950163
   while (otgw_clients[i] && otgw_clients[i].available()) {
      if (otgw_clients[i].available()) {
        while (otgw_clients[i].available()) {
          Serial.write(otgw_clients[i].read());
          Serial.flush(); // Wait for transmit to finish
          digitalWrite(node_led_pin, LOW);
          delay(1);
        }
      } else {
        digitalWrite(node_led_pin, HIGH);
      }
    }
  }

  // OTGW Serial -> Telnet
  if (Serial.available()) {
    size_t len = Serial.available();
    uint8_t sbuf[len];
    Serial.readBytes(sbuf, len);

    for (i = 0; i < max_clients; i++) {
    // https://github.com/esp8266/Arduino/issues/5257#issuecomment-430950163
      if (otgw_clients[i] && otgw_clients[i].connected()) {
        digitalWrite(esp_led_pin, LOW);
        otgw_clients[i].write(sbuf, len);
        delay(1);
      } else {
        digitalWrite(esp_led_pin, HIGH);
      }
    }
  }
}
