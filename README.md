# RoomAutomation
* Room automation with ESP8266 in Arduino
* Cloud support: https://iothub.live/nodes/4ca6ff41-4408-11e8-94bd-3dd310e71935

# Features

* PWM output (adjustable steps and frequency) with fade-in, fade-out or fade-to (= LED strip control)
* Switch input with software based noise reduction:
  - click (pulse width < 250 ms): fade-in or fade-out
  - short push-hold-release (pulse width > 250 ms): fade-in or fade-out and stops on release, so that you can set custom PWM levels
  - long push-hold-release (pulse width > 5000 ms): relay switch on
* PIR sensor: set PWM to half power after 5 mins and fade-off after another 5 mins when no move detected.
* DS18B20 and BME280 sensor read and REST send
* Heating control via relay pin
* Ventilation control based on humidity level via PWM
* IR sender and reveiver (on PIR and PWM pins)
* OTA firmware upgrade
* Separated WiFi codes and device parameters based on the MAC address of the devices
* Deep sleep.

# Schema

![Schema](https://github.com/gaborauth/RoomAutomation/blob/master/images/schema.jpg)
