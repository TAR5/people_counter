#include "wiring_private.h"
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include "arduino_secrets.h"

// Config variables.
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PW;
const char counterUrl[] = SECRET_ENDPOINT;
const char host[] = SECRET_HOST;
const int port = SECRET_PORT;

// Pin 0 als TX verwenden (Weißes Kabel hier rein).
// Pin 1 als RX verwenden (Grünes Kabel hier rein).
Uart SensorTwo(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Pin 2 als TX verwenden (Weißes Kabel hier rein).
// Pin 3 als RX verwenden (Grünes Kabel hier rein).
Uart SensorOne(&sercom0, 3, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);

typedef struct {
  int currentDistance;
  int distance;
  int strength;
  int restingDistance;
  boolean receiveComplete;
  boolean detectedEntry;
  boolean detectedLeave;
} TFmini;

typedef struct {
  int globalCount;
  int totalEntries;
  int totalExits;
  void increase()
  {
    ++globalCount;
    ++totalEntries;
  }
  void decrease()
  {
    --globalCount;
    ++totalExits;
  }
} Counter;

// Create counter object.
Counter counter = {0, 0, 0};

// Create sensor objects.
TFmini TFminiOne = {0, 0, 0, 0, false, false, false};
TFmini TFminiTwo = {0, 0, 0, 0, false, false, false};

// Counter variables.
const int threshold = 30;

// Create client.
//WiFiSSLClient client;
WiFiClient client;
HttpClient http_client = HttpClient(client, host, port);

// WiFi variables.
byte mac[6];
int status = WL_IDLE_STATUS;
String macAddress = "unknown";

// Convert bytes to hex.
const char hex[17]="0123456789ABCDEF";
String getHex(byte convertByte){
  return String(hex[(convertByte >>4) & 0x0F]) + String(hex[convertByte & 0x0F]);
}

// Mode (0 == undetermined, 1 == config, 2 == run).
int mode = 0;

void connect_to_wifi() {
  // Check WiFi firmware.
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    if (Serial) {
      Serial.println("[WiFi] Please upgrade the firmware");
    }
  }
  
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    if (Serial) {
      Serial.print("[WiFi] Connecting to: ");
      Serial.println(ssid);
    }
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(100);
  }
  
  // you're connected now, so print out the data:
  if (Serial) {
    Serial.println("[WiFi] Connected");
  }

  // Get the MAC address as a string.
  WiFi.macAddress(mac);
  macAddress = getHex(mac[5]) + ":" + getHex(mac[4]) + ":" + getHex(mac[3]) + ":" + getHex(mac[2]) + ":" + getHex(mac[1]) + ":" + getHex(mac[0]);

  // Print MAC address to monitor.
  if (Serial) {
    Serial.print("[WiFi] MAC address: ");
    Serial.println(macAddress);
  }
  
  errorBlinking();
}

void errorBlinking() {
  int i = 0;
  while (i < 10) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    i++;
  }
}

void counter_send() {
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);

  // Prepare the payload.
  String jsonPayload = "{\"Device\":\"" + macAddress + "\", \"GlobalCount\":" + counter.globalCount + ", \"TotalEntries\":" + counter.totalEntries + ", \"TotalExits\":" + counter.totalExits + "}";
  
  if (Serial) {
    Serial.println("[HTTP] Message: " + jsonPayload);
  }

  // Execute post request.
  http_client.post(counterUrl, "application/json", jsonPayload);
  http_client.contentLength();

  /**
  // read the status code and body of the response
  int statusCode = http_client.responseStatusCode();
  String response = http_client.responseBody();
  
  // Blink 
  if (statusCode != 200 && statusCode != 205) {
     errorBlinking();
  }*/
  
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
}

void getTFminiData(Uart* sensor, TFmini* tfmini) {
  static char i = 0;
  char j = 0;
  int checksum = 0; 
  static int rx[9];
  if(sensor->available()) {  
    rx[i] = sensor->read();
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        tfmini->distance = rx[2] + rx[3] * 256;
        tfmini->strength = rx[4] + rx[5] * 256;
        tfmini->receiveComplete = true;
      }
      i = 0;
    } else {
      i++;
    } 
  }  
}

void setup() {
  Serial.begin(115200);
  
  // Pin 0 als TX verwenden.
  // Pin 1 als RX verwenden.
  SensorTwo.begin(115200);
  pinPeripheral(0, PIO_SERCOM);
  pinPeripheral(1, PIO_SERCOM);
  
  // Pin 2 als TX verwenden.
  // Pin 3 als RX verwenden.
  SensorOne.begin(115200);
  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);


  // Check initial distance.
  getTFminiData(&SensorOne, &TFminiOne);
  getTFminiData(&SensorTwo, &TFminiTwo);
  delay(4000);
  while (TFminiOne.receiveComplete != true) {
    getTFminiData(&SensorOne, &TFminiOne);
  }
  while (TFminiTwo.receiveComplete != true) {
    getTFminiData(&SensorTwo, &TFminiTwo);
  }

  if (TFminiOne.distance < 10 && TFminiTwo.distance < 10) {
    if (Serial) {
      Serial.println("Starting in config mode.");
    }
    mode = 1;
  } else {
    if (Serial) {
      Serial.println("Starting in run mode.");
    }
    mode = 2;
    connect_to_wifi();
  }
}

void updateSensorState(TFmini* sensor, TFmini* otherSensor) {

  bool sensorIsResting = (sensor->distance > (sensor->restingDistance - 10) && sensor->distance < (sensor->restingDistance + 10));
  bool otherSensorIsResting = (otherSensor->distance > (otherSensor->restingDistance - 10) && otherSensor->distance < (otherSensor->restingDistance + 10));

  // Todo: Detect person only if he has cleared the sensor in the correct direction.
  // Save resting state (use millis to figure out when the sensor is resting).
  if (sensor->distance <= (sensor->restingDistance - threshold) && otherSensorIsResting && !otherSensor->detectedEntry) {
    // Mark this sensor as having detected an entry first.
    // It cannot detect an entry if the other sensor is not cleared.
    sensor->detectedEntry = true;
  } else if (sensor->detectedEntry && sensorIsResting && otherSensorIsResting) {
    // Sensor had previously detected an entry but all sensor have now returned to resting state.
    sensor->detectedEntry = false;
    sensor->detectedLeave = true;
  }
  
  if (sensor->distance >= (sensor->currentDistance + threshold) || sensor->distance <= (sensor->currentDistance - threshold)) {
    sensor->currentDistance = sensor->distance;
  }
}

int detectedEntry = 0;

void loop() {

  if (mode == 2) {
    // Run mode.
    static unsigned long lastDetectionTimestamp = 0;
  
    static unsigned long lastTime = millis();
    static unsigned int count = 0;
    static unsigned int frequency = 0;
  
    TFminiOne.detectedLeave = false;
    TFminiTwo.detectedLeave = false;
    
    getTFminiData(&SensorOne, &TFminiOne);
    getTFminiData(&SensorTwo, &TFminiTwo);
    
    if(TFminiOne.receiveComplete == true && TFminiTwo.receiveComplete == true) {
      ++count;
  
      if(millis() - lastTime > 999) {
        lastTime = millis();
        frequency = count;
        count = 0;
      }
  
      // Set initial resting distance.
      if (TFminiOne.restingDistance == 0) {
        TFminiOne.restingDistance = TFminiOne.distance;
      }
      if (TFminiTwo.restingDistance == 0) {
        TFminiTwo.restingDistance = TFminiTwo.distance;
      }
    
      updateSensorState(&TFminiOne, &TFminiTwo);
      updateSensorState(&TFminiTwo, &TFminiOne);
  
      //Serial.println(lastDetectionTimestamp);
  
      bool lastDetectionOldEnough = millis() > (lastDetectionTimestamp + 1000);
    
      if (TFminiOne.detectedLeave && lastDetectionOldEnough) {
        lastDetectionTimestamp = millis();
        if (Serial) {
          Serial.println("In");
        }
        counter.increase();
        counter_send();
      } else if (TFminiTwo.detectedLeave && lastDetectionOldEnough) {
        lastDetectionTimestamp = millis();
        if (Serial) {
          Serial.println("Out");
        }
        counter.decrease();
        counter_send();
      }
      
      TFminiOne.receiveComplete = false;
      TFminiTwo.receiveComplete = false;
    }
  } else if (mode == 1) {
    // Config mode.
  }
  

}

void SERCOM3_Handler()
{
  SensorTwo.IrqHandler();
}

void SERCOM0_Handler()
{
  SensorOne.IrqHandler();
}
