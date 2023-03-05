# Temperature and humidity MQTT sensor

Based on ESP32-S2 mini and HTU21D temperature sensor

Config file should be loaded to SPIFFS as well as sketch. To load data to SPIFFS, copy tool/esp32fs.jar to ~/Documents/arduino/tools/ESP32FS/tool then open Arduino 1.x, open sketch folder and run Tools -> ESP32 Sketch Data Upload -> Select SPIFFS

Sonoff Si7021 code taken from
https://github.com/SUPLA/supla-device/blob/main/src/supla/sensor/Si7021_sonoff.cpp