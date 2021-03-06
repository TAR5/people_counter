#include "wiring_private.h"
#include <WString.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include <FlashStorage.h>
#include "arduino_secrets.h"
#include <Scheduler.h>

// Config variables.
char ssid[255] = SECRET_SSID;
char pass[255] = SECRET_PW;
const char counterUrl[] = SECRET_ENDPOINT;
const char host[] = SECRET_HOST;
const int port = SECRET_PORT;
const char APssid[] = SECRET_AP_SSID;

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

// Create a structure that is big enough to contain a name
// and a surname. The "valid" variable is set to "true" once
// the structure is filled with actual data for the first time.
typedef struct {
  boolean valid;
  char ssid[255];
  char pw[255];
} Config;

// Reserve a portion of flash memory to store a "Config" and call it "configStore".
FlashStorage(configStore, Config);

Config storedConfig;

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

// Create server.
WiFiServer server(80);

// Convert bytes to hex.
const char hex[17]="0123456789ABCDEF";
String getHex(byte convertByte){
  return String(hex[(convertByte >>4) & 0x0F]) + String(hex[convertByte & 0x0F]);
}

// Mode (0 == undetermined, 1 == config, 2 == run).
int mode = 0;
boolean serialAvailable = false;
boolean sendCount = false;
boolean sendingCount = false;

void connectToWifi() {
  // Check WiFi firmware.
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    if (serialAvailable) {
      Serial.println("[WiFi] Please upgrade the firmware");
    }
  }
  
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    if (serialAvailable) {
      Serial.print("[WiFi] Connecting to: ");
      Serial.println(ssid);
    }
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(100);
  }
  
  // you're connected now, so print out the data:
  if (serialAvailable) {
    Serial.println("[WiFi] Connected");
  }

  // Get the MAC address as a string.
  WiFi.macAddress(mac);
  macAddress = getHex(mac[5]) + ":" + getHex(mac[4]) + ":" + getHex(mac[3]) + ":" + getHex(mac[2]) + ":" + getHex(mac[1]) + ":" + getHex(mac[0]);

  // Print MAC address to monitor.
  if (serialAvailable) {
    Serial.print("[WiFi] MAC address: ");
    Serial.println(macAddress);
  }
  
  errorBlinking();
}

void openServer() {
  if (serialAvailable) {
    Serial.println("[WiFi] Access Point Web Server");
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    if (serialAvailable) {
      Serial.println("[WiFi] Communication with WiFi module failed!");
    }
    // don't continue
    while (true);
  }

  // Check WiFi firmware.
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    if (serialAvailable) {
      Serial.println("[WiFi] Please upgrade the firmware");
    }
  }

  // by default the local IP address of will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  if (serialAvailable) {
    Serial.print("[WiFi] Creating access point named: ");
    Serial.println(APssid);
  }
  
  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(APssid);
  if (status != WL_AP_LISTENING) {
    if (serialAvailable) {
      Serial.println("[WiFi] Creating access point failed");
    }
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}

void printWiFiStatus() {
  if (serialAvailable) {
    // print the SSID of the network you're attached to:
    Serial.print("[WiFi] SSID: ");
    Serial.println(WiFi.SSID());
  
    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("[WiFi] IP Address: ");
    Serial.println(ip);
  
    // print where to go in a browser:
    Serial.print("[WiFi] Configure the counter by opening a browser to http://");
    Serial.println(ip);
  }
}

void errorBlinking() {
  digitalWrite(LED_BUILTIN, LOW);
  int i = 0;
  while (i < 10) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    i++;
  }
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

void loadConfig() {
  storedConfig = configStore.read();
  if (storedConfig.valid) {
    // Use credentials from store instead of secrets.
    String userSsid = storedConfig.ssid;
    String userPw = storedConfig.pw;
    userSsid.toCharArray(ssid, 255);
    userPw.toCharArray(pass, 255);
  }
  if (serialAvailable) {
    Serial.print("[Loaded config] SSID:");
    Serial.println(ssid);
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

  delay(3000);
  if (Serial) {
    serialAvailable = true;
  }

  // Read SSID and PW from saved config.
  loadConfig();

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
    if (serialAvailable) {
      Serial.println("Starting in config mode.");
    }
    mode = 1;
    openServer();
  } else {
    if (serialAvailable) {
      Serial.println("Starting in run mode.");
    }
    mode = 2;
    connectToWifi();
    // Set initial resting distance.
    if (TFminiOne.restingDistance == 0) {
      TFminiOne.restingDistance = TFminiOne.distance;
    }
    if (TFminiTwo.restingDistance == 0) {
      TFminiTwo.restingDistance = TFminiTwo.distance;
    }
  }

  Scheduler.startLoop(loop2);
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

unsigned char h2int(char c)
{
    if (c >= '0' && c <='9'){
        return((unsigned char)c - '0');
    }
    if (c >= 'a' && c <='f'){
        return((unsigned char)c - 'a' + 10);
    }
    if (c >= 'A' && c <='F'){
        return((unsigned char)c - 'A' + 10);
    }
    return(0);
}

String urldecode(String str)
{
    
    String encodedString="";
    char c;
    char code0;
    char code1;
    for (int i =0; i < str.length(); i++){
        c=str.charAt(i);
      if (c == '+'){
        encodedString+=' ';  
      }else if (c == '%') {
        i++;
        code0=str.charAt(i);
        i++;
        code1=str.charAt(i);
        c = (h2int(code0) << 4) | h2int(code1);
        encodedString+=c;
      } else{
        
        encodedString+=c;  
      }
      
      yield();
    }
    
   return encodedString;
}

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
      
      updateSensorState(&TFminiOne, &TFminiTwo);
      updateSensorState(&TFminiTwo, &TFminiOne);
  
      //Serial.println(lastDetectionTimestamp);
  
      bool lastDetectionOldEnough = millis() > (lastDetectionTimestamp + 200);
    
      if (TFminiOne.detectedLeave && lastDetectionOldEnough) {
        lastDetectionTimestamp = millis();
        if (serialAvailable) {
          Serial.println("In");
        }
        counter.increase();
        sendCount = true;
        // Add a tiny delay so loop 2 gets executed.
        digitalWrite(LED_BUILTIN, HIGH);
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);
      } else if (TFminiTwo.detectedLeave && lastDetectionOldEnough) {
        lastDetectionTimestamp = millis();
        if (serialAvailable) {
          Serial.println("Out");
        }
        counter.decrease();
        sendCount = true;
        // Add a tiny delay so loop 2 gets executed.
        digitalWrite(LED_BUILTIN, HIGH);
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);
      }
      
      TFminiOne.receiveComplete = false;
      TFminiTwo.receiveComplete = false;
    }
  } else {
    // Compare the previous status to the current status.
    if (status != WiFi.status()) {
      // It has changed, update the variable.
      status = WiFi.status();
      if (serialAvailable) {
        if (status == WL_AP_CONNECTED) {
          // A device has connected to the AP.
          Serial.println("[Config] Device connected to AP");
        } else {
          // A device has disconnected from the AP, and we are back in listening mode.
          Serial.println("[Config] Device disconnected from AP");
        }
      }
    }
    
    WiFiClient client = server.available();
  
    if (client) {
      if (serialAvailable) {
        Serial.println("[Config] New client");
      }
      
      String currentLine = "";
      String request = "";
      String urlPath = "";
      String inputSSID = ssid;
      String inputPW = pass;
      
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();

          // Print the request to the monitor.
          //if (serialAvailable) {
          //  Serial.write(c);
          //}

          request += c;
          
          if (c == '\n') {
            // If the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // Get the URL from the path and the query paramters.
              // GET /?ssid=SSID&pw=PW HTTP/1.1
              if(request.indexOf("GET /save") != -1) {
                int startOfPath = request.indexOf("GET /save?");
                int endOfPath = request.indexOf(" HTTP");
                urlPath = request.substring(startOfPath+10, endOfPath);
                // Get parameters from path.
                int split = urlPath.indexOf("&");
                inputSSID = urlPath.substring(5, split);
                inputPW = urlPath.substring(split+4);
                // Url decode special chars.
                inputSSID = urldecode(inputSSID);
                inputPW = urldecode(inputPW);
                if (serialAvailable) {
                  Serial.print("[Config] SSID: ");
                  Serial.println(inputSSID);
                  Serial.print("[Config] PW: ");
                  Serial.println(inputPW);
                }
                // Save new SSID and PW to config store.
                inputSSID.toCharArray(storedConfig.ssid, 255);
                inputPW.toCharArray(storedConfig.pw, 255);
                storedConfig.valid = true;
                configStore.write(storedConfig);
                // Save the new SSID and PW to memory.
                inputSSID.toCharArray(ssid, 255);
                inputPW.toCharArray(pass, 255);

                // Display saved message.
                // Print the HTTP header.
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:text/html");
                client.println();
    
                // Print the HTTP content.
                client.print("<!doctype html><html><head></head><body><h4>Saved!</h4><script>setTimeout(() => { window.location.replace('http://192.168.4.1/'); }, 1000);</script>");
    
              } else {
                // Print the HTTP header.
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:text/html");
                client.println();
    
                // Print the HTTP content.
                client.print("<!doctype html><html><head></head><body><h4>People Counter Configuration</h4><form action='/save' method='get'><label>SSID:</label> <input name='ssid' type='text' maxlength='200' value='"+inputSSID+"'><br><label>Password:</label> <input name='pw' type='text' maxlength='200' value='"+inputPW+"'><br><button type='submit'>Save</button></form>");
              }

              
              // The HTTP response ends with another blank line.
              client.println();
              break;
            }
            else {
              currentLine = "";
            }
          }
          else if (c != '\r') {
            currentLine += c;
          }
        }
      }
      
      // Close the connection:
      client.stop();
      if (serialAvailable) {
        Serial.println("[Config] client disconnected");
      }
    }
  }

}

// This loop sends the new count without blocking the sensors from collecting more entries and exits.
void loop2() {
  if (sendCount && sendingCount == false) {
    sendingCount = true;
  
    // Prepare the payload.
    String jsonPayload = "{\"Device\":\"" + macAddress + "\", \"GlobalCount\":" + counter.globalCount + ", \"TotalEntries\":" + counter.totalEntries + ", \"TotalExits\":" + counter.totalExits + "}";
    
    if (serialAvailable) {
      Serial.println("[HTTP] Message: " + jsonPayload);
    }
  
    // Execute post request.
    http_client.post(counterUrl, "application/json", jsonPayload);
    http_client.contentLength();
  
    sendingCount = false;
  }
  yield();
}

void SERCOM3_Handler()
{
  SensorTwo.IrqHandler();
}

void SERCOM0_Handler()
{
  SensorOne.IrqHandler();
}
