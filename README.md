# ESP8266 OTGW

## Description

Arduino application for [ESP8266](https://arduino-esp8266.readthedocs.io/en/latest/) hardware. It acts as a serial bridge for the [Nodoshop](https://www.nodo-shop.nl/en/opentherm-gateway/188-opentherm-gateway.html) [OpenTherm Gateway](http://otgw.tclcode.com).

## Installation

* Copy `Config.example.h` to `Config.h` and configure settings.

### Required libraries

* [ESP8266](https://github.com/esp8266/Arduino)
* [ESPGoodies](https://github.com/d-a-v/EspGoodies)

## Usage

### OTGW serial bridge (port 23)

`telnet OTGW_IP 23`

### ESP commands (port 24)

`telnet OTGW_IP 24` 

Command | Usage
------- | -----
`$SYS` | Display ESP Arduino version info, uptime & last restart reason
`$MEM` | Display free memory & fragmentation
`$NET` | Display IP, Subnet, Gateway, DNS and MAC address
`$WIF` | display WiFi diagnostics
`$UPD` | Update ESP firmware 
`$RST ESP` | Reset ESP
`$RST OTGW` | Reset OTGW
`$HLP` | Display usage