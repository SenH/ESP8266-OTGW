//
// ESP8266 serial wifi bridge
//

#include "Config.h"

#include "core_version.h"
#include "ESP8266WiFi.h"
#include "ESP8266HTTPClient.h"
#include "ESP8266httpUpdate.h"
#include "Wire.h"
#include "Ticker.h"

int rssi_to_percent(float rssi);
String get_human_uptime(uint32_t uptime);

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

#define STACK_PROTECTOR  512 // bytes

// 
// GLOBALS
// 

Ticker t_uptime;

static const byte port PROGMEM                          = 23;
static const byte max_clients PROGMEM                   = 2;
static const int baud_rate PROGMEM                      = 9600;

static const byte esp_led_pin PROGMEM                   = 2;
static const byte node_led_pin PROGMEM                  = 16;
static const byte otgw_reset_pin PROGMEM                = 14;
static const byte wd_i2c_address PROGMEM                = 38;

unsigned long uptime                                    = 0; // Counting every second until 4294967295 = 130 years
unsigned long wifi_connect_ts                           = 0;

WiFiServer otgw_server(port);
WiFiServer esp_server(port + 1);

WiFiClient otgw_clients[max_clients];
WiFiClient esp_clients[max_clients];

unsigned long wifi_connect_timer                        = 0;
static const unsigned long wifi_connect_timeout PROGMEM = 60000;

unsigned long wd2_timer                                 = 0;
static const unsigned long wd2_interval PROGMEM         = 1000;

static const char EOL[] PROGMEM                         = "\r\n";
static const char USAGE[] PROGMEM                       = "$SYS $MEM $NET $WIF $UPD $RST ESP|OTGW $EXT $HLP";
static const char ERR_SERVER_BUSY[] PROGMEM             = "Server is busy with %d active connections%s";

// 
// FUNCTIONS
// 

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
  s_println(F("WiFi > Initialize"));
  WiFi.mode(WIFI_STA);
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
    digitalWrite(esp_led_pin, !digitalRead(esp_led_pin));
    delay(500);

    // Restart ESP after timeout
    if (now - wifi_connect_timer >= wifi_connect_timeout) {
      s_printf_P(PSTR("WiFi > Failed connecting after %d seconds!%s"), (wifi_connect_timeout / 1000), FPCC(EOL));
      s_println(F("ESP > Restart now"));
      ESP.restart();
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

// https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266httpUpdate/examples/httpUpdate/httpUpdate.ino
void do_http_update(WiFiClient client) {
  WiFiClient OTAclient;
  ESPhttpUpdate.setLedPin(node_led_pin, LOW);
  t_httpUpdate_return ret = ESPhttpUpdate.update(OTAclient, FPCC(esp_update_url));
  
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

String get_human_uptime(uint32_t uptime) {
  uint32_t time;
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint16_t days;
  
  time = uptime;
  seconds = time % 60;
  time /= 60;   // now it is minutes
  minutes = time % 60;
  time /= 60;   // now it is hours
  hours = time % 24;
  time /= 24;   // now it is days
  days = time; 

  String s;
  s.reserve(32);
  if (days != 0) {
    s += days;
    if (days == 1) {
      s += F(" day ");
    } else {
      s += F(" days ");
    }
  }
  if (hours != 0) {
    s += hours;
    if (hours == 1) {
      s += F(" hour ");
    } else {
      s += F(" hours ");
    }
  }
  if (minutes != 0) {
    s += minutes;
    if (minutes == 1) {
      s += F(" minute ");
    } else {
      s += F(" minutes ");
    }
  } else {
    s += seconds;
    if (seconds == 1) {
      s += F(" second");
    } else {
      s += F(" seconds");
    }
  }
  
  return s;
}

void parse_esp_cmd(WiFiClient client) {
  String cmd;
  cmd.reserve(16);

  client.setTimeout(1); // Max waiting time for readStringUntil()
  cmd = client.readStringUntil('\r');
  cmd.toUpperCase();
  client.readStringUntil('\n'); // Discard LF

  if (cmd.length() < 4) {
    client.println(FPSTR(USAGE));
    return;
  }
  
  if (cmd.equals(F("$SYS"))) {
#ifdef SKETCH_VERSION
    client.printf_P(PSTR("Sketch:%s/"), SKETCH_VERSION);
#endif
    client.println(ESP.getFullVersion());
    client.printf_P(PSTR("    ESP uptime: %s%s"), get_human_uptime(uptime).c_str(), FPCC(EOL));
    client.printf_P(PSTR("   WiFi uptime: %s%s"), get_human_uptime(uptime - wifi_connect_ts).c_str(), FPCC(EOL));
    client.printf_P(PSTR("Restart reason: %s%s"), ESP.getResetReason().c_str(), FPCC(EOL));
  } else if (cmd.equals(F("$MEM"))) {
    client.printf_P(PSTR("Free: %d bytes Fragmentation: %d%%%s"), ESP.getFreeHeap(), ESP.getHeapFragmentation(), FPCC(EOL));
  } else if (cmd.equals(F("$NET"))) {
    client.println(get_net_info());
  } else if (cmd.equals(F("$WIF"))) {
    WiFi.printDiag(client);
    client.printf_P(PSTR("BSSID: %s%s"), WiFi.BSSIDstr().c_str(), FPCC(EOL));
    client.printf_P(PSTR("RSSI: %d dBm (%d%%)%s"), WiFi.RSSI(), rssi_to_percent(WiFi.RSSI()), FPCC(EOL));
  } else if (cmd.equals(F("$UPD"))) {
    client.printf_P(PSTR("Update ESP via %s%s"), FPCC(esp_update_url), FPCC(EOL));
    do_http_update(client);
  } else if (cmd.equals(F("$RST ESP"))) {
    client.println(F("ESP > Restart"));
    ESP.restart();
  } else if (cmd.equals(F("$RST OTGW"))) {
    client.println(F("OTGW > Reset"));
    reset_otgw();
    client.println(F("OTGW > Reset complete"));
  } else if (cmd.equals(F("$EXT"))) {
    client.println(F("Goodbye..."));
    client.stop();
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
// https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiTelnetToSerial/WiFiTelnetToSerial.ino
// 

void setup(void) {
  // Increment uptime every second
  t_uptime.attach(1.0, uptime_inc);

  Serial.begin(baud_rate);
#ifdef SKETCH_VERSION
  s_printf_P(PSTR("\nSketch:%s/"), SKETCH_VERSION);
#endif
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

  pinMode(esp_led_pin, OUTPUT);
  pinMode(node_led_pin, OUTPUT);
  // Set LEDS OFF
  digitalWrite(esp_led_pin, HIGH);
  digitalWrite(node_led_pin, HIGH);
  
#ifdef RESET_OTGW_ON_BOOT
  reset_otgw();
#endif

  connect_to_wifi();
  esp_server.begin();
  otgw_server.begin();
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
      if (!esp_clients[i]) {
        esp_clients[i] = esp_server.available();
        s_print(F("SRV > New client: "));
        s_println(i);
        break;
      }
    }
    // No free spot so reject
    if (i == max_clients) {
      WiFiClient a_client = esp_server.available();
      a_client.printf_P(FPCC(ERR_SERVER_BUSY), max_clients, FPCC(EOL));
      s_println(F("SRV > Max clients exceeded. Rejecting!"));
    }
  }
  
  // ESP commands
  for (i = 0; i < max_clients; i++) {
    while (esp_clients[i].available()) {
      parse_esp_cmd(esp_clients[i]);
    }
  }

  // Get new OTGW clients
  if (otgw_server.hasClient()) {
    for (i = 0; i < max_clients; i++) {
      // Find free spot
      if (!otgw_clients[i]) {
        otgw_clients[i] = otgw_server.available();
        s_print(F("SRV > New client: "));
        s_println(i);
        break;
      }
    }
    // No free spot so reject
    if (i == max_clients) {
      WiFiClient a_client = otgw_server.available();
      a_client.printf_P(FPCC(ERR_SERVER_BUSY), max_clients, FPCC(EOL));
      s_println(F("SRV > Max clients exceeded. Rejecting!"));
    }
  }
  
  // OTGW Telnet -> Serial
  for (i = 0; i < max_clients; i++) {
    while (otgw_clients[i].available() && Serial.availableForWrite() > 0) {
      if (otgw_clients[i].available()) {
        Serial.write(otgw_clients[i].read());
        Serial.flush(); // Wait for transmit to finish
        digitalWrite(node_led_pin, !digitalRead(node_led_pin));
        delay(1); // Increases LED ON time
      }
    }
  }

  // OTGW Serial -> Telnet
  size_t maxToTcp = 0;
  for (int i = 0; i < max_clients; i++) {
    // Determine maximum output size "fair TCP use"
    if (otgw_clients[i]) {
      size_t afw = otgw_clients[i].availableForWrite();
      if (afw) {
        if (!maxToTcp) {
          maxToTcp = afw;
        } else {
          maxToTcp = std::min(maxToTcp, afw);
        }
      } else {
        // warn but ignore congested clients
        s_println(F("SRV > One client is congested"));
      }
    }

    size_t len = std::min((size_t)Serial.available(), maxToTcp);
    len = std::min(len, (size_t)STACK_PROTECTOR);
    if (len) {
      uint8_t sbuf[len];
      size_t serial_got = Serial.readBytes(sbuf, len);
      // push UART data to all connected telnet clients
      for (int i = 0; i < max_clients; i++) {
        // if client.availableForWrite() was 0 (congested)
        // and increased since then,
        // ensure write space is sufficient:
        if (otgw_clients[i].availableForWrite() >= serial_got) {
          digitalWrite(esp_led_pin, !digitalRead(esp_led_pin));
          size_t tcp_sent = otgw_clients[i].write(sbuf, serial_got);
          if (tcp_sent != len) {
            s_printf_P(PSTR("WiFi > len mismatch: available:%zd serial-read:%zd tcp-write:%zd%s"), 
                            len, serial_got, tcp_sent, FPCC(EOL));
          }
          delay(1); // Increases LED ON time
        }
      }
    }
  }
}
