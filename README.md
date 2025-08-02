# Overview
- This is an ESP32 microcontroller with assorted sensors, with readings displayed on the TFT display, as well as on a web page

# Supported Microcontrollers
- Adafruit ESP32-S2 TFT
- Adafruit ESP32-S3 TFT

# Supported Sensors
- Adafruit AHT20 temperature and humidity sensor
- Adafruit BME680 temperature, humidity, altitude, barometric pressure, gas resistance
- Adafruit VL53L1X time of flight distance sensor
- Adafruit VL6180X time of flight and lux sensor
  

# Supported Actuators
- none so far
  
# Supported Build Environment
- Arduino IDE 2.x


# Web Page Output
```
ESP32 Web Server
Uptime: 0 days, 1 hours, 50 minutes, 21 seconds.
Battery: 4 V / 97%
Temperature: 23.79 C
Humidity: 47.89 % rH
Distance: 1589 mm
-------------------------
WiFi Details:
MAC address: 7C:DF:A1:96:70:D0
IP address: 10.99.99.99
Subnet mask: 255.255.255.0
Gateway: 10.99.99.1
SSID: myWifiNetwork
RSSI signal strength: -58 dBm
WiFi channel: 6
-------------------------------
Nearby WiFi Networks:
1: BigCorp (RSSI -44 dBm, Channel 6)
2: HotelGuest (RSSI -48 dBm, Channel 1)
3: StarBucks Wifi (RSSI -50 dBm, Channel 11)
4: Free Airport Wifi (RSSI -56 dBm, Channel 6)
----------------------------------------
Hardware inventory:
Microcontroller: Adafruit ESP32-S2 TFT
Sensors: AHT20,VL53L1X
Actuators: none
Software build environment: Arduino IDE 2.1.1
Software build date: 2023-07-09
Software build source: https://github.com/nickjeffrey/microcontrollers
```

# TFT Display Output
- TFT display is more space constrained (4 lines, 20 characters per line)
- TFT display shows a subset of the web page (sensor readings, wifi network, MAC address, IP address, uptime)
- Due to space constraints, TFT display cycles through multiple pages of text, pausing a few seconds on each page
<img src=images/esp32-s2-tft.png>
