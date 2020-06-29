# Arduino People Counter

### Hardware

- 1x Arduino MKR WiFi 1010
- 2x TFMini Lidar Sensor
- 2x female to female jumper cable

### Setup

Sensor 1 is connected to pin 0 (TX, white cable) and pin 1 (RX, green cable).
Sensor 2 is connected to pin ~2 (TX, white cable) and pin ~3 (RX, green cable).

You could also connected one of the sensors to the out-of-the-box TX and RX pins (13 and 14).
I connected them this way to have a cleaner setup.

Connect the red wire of each sensor to 5v and the black wire of each sensor to GND.

![Setup](https://github.com/TAR5/people_counter/blob/master/docs/image.jpg)

### Dependencies

You can find the required libraries in the Arduino library manager.

- wiring_private.h (for giving the pins the required functionality, already included by default)
- ArduinoHttpClient.h (library to make HTTP requests, already included by default)
- WiFiNINA.h (WiFi library for the MKR WiFi 1010 board)
- Scheduler.h (Adds support for multiple operations at once)

### Secrets

```cpp
#define SECRET_SSID "yourWiFiSSid"
#define SECRET_PW "yourWifiPassword"
#define SECRET_ENDPOINT "/api/v1/your-api-endpoint"
#define SECRET_HOST "your-api-host.com"
#define SECRET_PORT 80 // Your API port, use 80 for HTTP and 443 for HTTPS
#define SECRET_AP_SSID "peopleCounter"
```

The WiFi SSID and password is changeable via the config interface (see "Configuration").

Use the `WiFiSSLClient` instead of the `WiFiClient` if your API endpoint is HTTPS only.


### Configuration

To configure the counter start the device with obscured sensors. The counter will enter into config mode and open a WiFi access point called "peopleCounter".

Connect to the access point and open `http://192.168.4.1` in your browser.

Set your WiFi SSID and password for the run mode.

