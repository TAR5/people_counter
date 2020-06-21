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
- ArduinoHttpClient.h (libary to make HTTP requests, already included by default)
- WiFiNINA.h (WiFi library for the MKR WiFi 1010 board)

### Secrets

```cpp
#define SECRET_SSID "yourWiFiSSid"
#define SECRET_PW "yourWifiPassword"
#define SECRET_ENDPOINT "/api/v1/your-api-endpoint"
#define SECRET_HOST "your-api-host.com"
#define SECRET_PORT 80 // Your API port, use 80 for HTTP and 443 for HTTPS
```

Use the `WiFiSSLClient` instead of the `WiFiClient` if your API endpoint is HTTPS only.

