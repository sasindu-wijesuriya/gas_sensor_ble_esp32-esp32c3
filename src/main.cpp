#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <ElegantOTA.h>

// gateway firmware version
String gatewayFWVersion = "C3-1.0.2";

bool inAPMode = false;
bool bluetooth_sending_status = false;
bool inSensorSearchingMode = false;
unsigned long previousMillis = 0;
unsigned long previousMillisForAPMode = 0;
unsigned long ota_progress_millis = 0;
int led_state = 0;
int number_of_failed_attempts_to_connect_to_server = 0;
int max_number_of_failed_attempts = 4;
bool automatically_put_to_AP_mode = false;

String tankSize = "NA";
String timeZone = "NA";
String longitude = "NA";
String latitude = "NA";
String loadedHeight = "NA";
String selected_sensor_mac_address = "NA";

const int scanTimeSeconds = 1;
const int BOOT_PIN = 9;
const int SSID_ADDR = 0;
const int PASS_ADDR = 50;
const int TANKSIZE_ADDR = 100;
const int TIMEZONE_ADDR = 150;
const int LONGITUDE_ADDR = 200;
const int LATITUDE_ADDR = 250;
const int LOADED_HEIGHT_ADDR = 300;
const int SENSOR_MAC_ADDR = 350;
const int EEPROM_SIZE = 512;
const int httpsPort = 443;
const int sound_speed = 757;
const long blink_interval = 500;
const long wifi_search_interval = 120000;
const char *ap_ssid = "Gateway";
const char *ap_password = "123456789";
const char *serverHost = "elysiumapi.overleap.lk";
const char *apiPath = "/api/v2/gas/stream/esp32_que"; // API endpoint

static unsigned long lastSendTime = 0;

BLEScan *pBLEScan;

WebServer server(80);
WiFiClientSecure client;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);

String postData;

void indicateSuccessfulConnection();
void indicateSuccessfulDataSendToServer();
void blinkLEDinErrorPattern(int number_of_blinks);
bool tryConnectToSavedWiFi();
bool handleButtonPress();

std::string string_to_hex(const std::string &input)
{
  static const char hex_digits[] = "0123456789abcdef";
  std::string output;
  output.reserve(input.length() * 2);
  for (unsigned char c : input)
  {
    output.push_back(hex_digits[c >> 4]);
    output.push_back(hex_digits[c & 15]);
  }
  return output;
}

std::string format_hex_string(const std::string &hexString)
{
  std::string formattedString;
  for (size_t i = 0; i < hexString.length(); i += 2)
  {
    formattedString += hexString.substr(i, 2);
    if (i + 2 < hexString.length())
    {
      formattedString += " ";
    }
  }
  return formattedString;
}

void onOTAStart()
{
  Serial.println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final)
{
  if (millis() - ota_progress_millis > 1000)
  {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success)
{
  if (success)
  {
    Serial.println("OTA update finished successfully!");
  }
  else
  {
    Serial.println("There was an error during OTA update!");
  }
  ESP.restart();
}

int hex_to_int(const std::string &hexString)
{
  return strtol(hexString.c_str(), nullptr, 16);
}

void sendDataToServer(void *param)
{
  Serial.println("\n****************************************************************************************************");

  if (client.connect("www.google.com", 443))
  {
    Serial.println("Internet connection: OK");
  }
  else
  {
    Serial.println("Internet connection: FAILED!!!!!");
  }
  client.stop();

  if (client.connect(serverHost, httpsPort))
  {
    Serial.println("Connected to server. Sending data... ");
    Serial.println("JSON Data:");
    Serial.println(postData);

    client.println(String("POST ") + apiPath + " HTTP/1.1");
    client.println(String("Host: ") + serverHost);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(postData.length());
    client.println();
    client.println(postData);

    unsigned long timeout = millis();
    bool isHeader = true;
    int responseCode = -1;
    String responseBody = "";

    int freeHeap = ESP.getFreeHeap();
    Serial.println("Free heap memory: " + String(freeHeap));

    if (freeHeap < 20000)
    {
      Serial.println("Free heap memory is low. Restarting ESP");
      blinkLEDinErrorPattern(4); // Blink LED 4 times in error pattern
      ESP.restart();
    }

    while (client.connected() && (millis() - timeout) < 5000)
    {
      String response = client.readString();
      responseCode = response.substring(9, 12).toInt();
      Serial.print("Server responseCode: ");
      Serial.println(responseCode);
      if (responseCode == 200)
      {
        Serial.println("Data sent successfully");
        indicateSuccessfulDataSendToServer();
        number_of_failed_attempts_to_connect_to_server = 0;
        break;
      }
      else
      {
        number_of_failed_attempts_to_connect_to_server++;
      }
    }
    client.stop();
  }
  else
  {
    Serial.println("Connection to server failed");
    number_of_failed_attempts_to_connect_to_server++;
    Serial.println("Number of failed attempts: " + String(number_of_failed_attempts_to_connect_to_server));
  }
  if (number_of_failed_attempts_to_connect_to_server >= max_number_of_failed_attempts)
  {
    Serial.println("Restarting ESP");
    blinkLEDinErrorPattern(2); // Blink LED 2 times in error pattern
    ESP.restart();
  }

  vTaskDelete(NULL); // Delete the task once the HTTPS request is done
}

void createHTTPSTask()
{
  xTaskCreate(
      sendDataToServer, // Task function
      "HTTPS Task",     // Task name
      8192,             // Stack size
      NULL,             // Task input parameter
      1,                // Priority
      NULL              // Task handle
  );
}

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{

  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (strcmp(advertisedDevice.getName().c_str(), "") == 0)
    {
      std::string hexAdvData = string_to_hex(advertisedDevice.getManufacturerData());
      if (hexAdvData.rfind("544e", 0) == 0) // Check for a valid prefix
      {
        std::string frameHead1 = hexAdvData.substr(0, 4);
        std::string type = hexAdvData.substr(4, 2);
        std::string cmd = hexAdvData.substr(6, 2);
        std::string frameHead2 = hexAdvData.substr(8, 4);
        std::string measurementHex = hexAdvData.substr(12, 4);
        std::string batteryHex = hexAdvData.substr(16, 2);
        std::string macAddress = advertisedDevice.getAddress().toString().c_str();

        if (inSensorSearchingMode)
        {
          if (type == "21")
          {
            Serial.println("\nFound a SYNCed sensor");
            inSensorSearchingMode = false;
            selected_sensor_mac_address = macAddress.c_str();
            Serial.print("MAC Address: ");
            Serial.println(selected_sensor_mac_address);
          }
        }
        else
        {
          if (strcmp(macAddress.c_str(), selected_sensor_mac_address.c_str()) == 0)
          {
            float measurement = (hex_to_int(measurementHex)) * sound_speed / 2000;

            int battery = hex_to_int(batteryHex);

            // Serial.print("Received Payload: ");
            // Serial.println(hexAdvData.c_str());

            timeClient.update();
            unsigned long epochTime = timeClient.getEpochTime();

            postData = String("{\"DATETIME\":") + String(epochTime) +
                       ",\"IMEI\":\"" + String(advertisedDevice.getAddress().toString().c_str()) + "\"," +
                       "\"NCU_FW_VER\":\"" + String(gatewayFWVersion) + "\"," +
                       "\"GAS_METER\":" + String(measurement / 10) + "," +
                       "\"CSQ\":104," +
                       "\"MCU_TEMP\":30," +
                       "\"BAT_VOL\":" + String(battery) + "," +
                       "\"METER_TYPE\":4," +
                       "\"TIME_ZONE\":\"" + String(timeZone) + "\"," +
                       "\"TANK_SIZE\":\"" + String(tankSize) + "\"," +
                       "\"GAS_PERCENT\":" + String(measurement * 100 / loadedHeight.toFloat()) + "," +
                       "\"LONGITUDE\":\"" + String(longitude) + "\"," +
                       "\"LATITUDE\":\"" + String(latitude) + "\"," +
                       "\"LOADED_HEIGHT\":" + String(loadedHeight.toFloat()) + "," +
                       "\"RSSI\":" + String(advertisedDevice.getRSSI()) + "}";

            // Ensure there's a delay between transmissions
            if (millis() - lastSendTime > 5000)
            {
              createHTTPSTask();
              lastSendTime = millis();
            }
          }
        }
      }
    }
  }
};

void switchToAPMode()
{
  Serial.println("\n\n****************************************************************************************************\n****************************************************************************************************");
  Serial.println("\nSwitching to AP mode...");
  bluetooth_sending_status = false;
  WiFi.mode(WIFI_AP);
  inAPMode = true;
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("\nAccess Point Started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void blinkLEDInAPMode()
{
  if (automatically_put_to_AP_mode)
  {
    Serial.println("Trying to connect to the last saved Wi-Fi network...");
    if (tryConnectToSavedWiFi())
    {
      inAPMode = false;
      WiFi.mode(WIFI_STA);
      bluetooth_sending_status = true;
      Serial.println("Connected to the last saved Wi-Fi network... Restarting the gateway");
      delay(500);
      ESP.restart();
    }
    else
    {
      Serial.println("Failed to connect to the last saved Wi-Fi network.\nRetrying in 2 second...");
      delay(2000);
      WiFi.mode(WIFI_AP_STA);
      handleButtonPress();
    }
  }
  else
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= blink_interval)
    {
      previousMillis = currentMillis;
      led_state = !led_state;
      digitalWrite(BUILTIN_LED, led_state);
    }
  }
}

bool handleButtonPress()
{
  if (digitalRead(BOOT_PIN) == LOW)
  {
    delay(100);
    if (digitalRead(BOOT_PIN) == LOW)
    {
      Serial.println("\nBoot Button press recognized");
      automatically_put_to_AP_mode = false;
      switchToAPMode();
      return true;
    }
    return false;
  }
  return false;
}

void saveWiFiCredentials(const String &ssid, const String &password)
{
  EEPROM.begin(EEPROM_SIZE);

  for (int i = SSID_ADDR; i < SSID_ADDR + 50; i++)
    EEPROM.write(i, 0);
  for (int i = PASS_ADDR; i < PASS_ADDR + 50; i++)
    EEPROM.write(i, 0);

  for (int i = 0; i < ssid.length(); i++)
    EEPROM.write(SSID_ADDR + i, ssid[i]);
  for (int i = 0; i < password.length(); i++)
    EEPROM.write(PASS_ADDR + i, password[i]);
  Serial.println("Saved Wi-Fi credentials to EEPROM");

  EEPROM.commit();
}

void saveOtherConfigDataToEEPROM(const String &tankSize, const String &timeZone, const String &longitude, const String &latitude, const String &loadedHeight)
{
  EEPROM.begin(EEPROM_SIZE);

  for (int i = TANKSIZE_ADDR; i < TANKSIZE_ADDR + 50; i++)
    EEPROM.write(i, 0);
  for (int i = TIMEZONE_ADDR; i < TIMEZONE_ADDR + 50; i++)
    EEPROM.write(i, 0);
  for (int i = LONGITUDE_ADDR; i < LONGITUDE_ADDR + 50; i++)
    EEPROM.write(i, 0);
  for (int i = LATITUDE_ADDR; i < LATITUDE_ADDR + 50; i++)
    EEPROM.write(i, 0);
  for (int i = LOADED_HEIGHT_ADDR; i < LOADED_HEIGHT_ADDR + 50; i++)
    EEPROM.write(i, 0);

  for (int i = 0; i < tankSize.length(); i++)
    EEPROM.write(TANKSIZE_ADDR + i, tankSize[i]);
  for (int i = 0; i < timeZone.length(); i++)
    EEPROM.write(TIMEZONE_ADDR + i, timeZone[i]);
  for (int i = 0; i < longitude.length(); i++)
    EEPROM.write(LONGITUDE_ADDR + i, longitude[i]);
  for (int i = 0; i < latitude.length(); i++)
    EEPROM.write(LATITUDE_ADDR + i, latitude[i]);
  for (int i = 0; i < loadedHeight.length(); i++)
    EEPROM.write(LOADED_HEIGHT_ADDR + i, loadedHeight[i]);

  EEPROM.commit();
  Serial.println("Saved other configuration data to EEPROM");
}

void loadWiFiCredentials(String &ssid, String &password)
{
  EEPROM.begin(EEPROM_SIZE);

  char ssidBuff[50];
  char passBuff[50];

  for (int i = 0; i < 50; i++)
    ssidBuff[i] = EEPROM.read(SSID_ADDR + i);
  for (int i = 0; i < 50; i++)
    passBuff[i] = EEPROM.read(PASS_ADDR + i);

  ssid = String(ssidBuff);
  password = String(passBuff);
  Serial.println("Loaded Wi-Fi credentials from EEPROM");

  char tankSizeBuff[50];
  char timeZoneBuff[50];
  char longitudeBuff[50];
  char latitudeBuff[50];
  char loadedHeightBuff[50];
  char selectedSensorMacBuff[50];

  for (int i = 0; i < 50; i++)
    tankSizeBuff[i] = EEPROM.read(TANKSIZE_ADDR + i);
  for (int i = 0; i < 50; i++)
    timeZoneBuff[i] = EEPROM.read(TIMEZONE_ADDR + i);
  for (int i = 0; i < 50; i++)
    longitudeBuff[i] = EEPROM.read(LONGITUDE_ADDR + i);
  for (int i = 0; i < 50; i++)
    latitudeBuff[i] = EEPROM.read(LATITUDE_ADDR + i);
  for (int i = 0; i < 50; i++)
    loadedHeightBuff[i] = EEPROM.read(LOADED_HEIGHT_ADDR + i);
  for (int i = 0; i < 50; i++)
    selectedSensorMacBuff[i] = EEPROM.read(SENSOR_MAC_ADDR + i);

  tankSize = String(tankSizeBuff);
  timeZone = String(timeZoneBuff);
  longitude = String(longitudeBuff);
  latitude = String(latitudeBuff);
  loadedHeight = String(loadedHeightBuff);
  selected_sensor_mac_address = String(selectedSensorMacBuff);

  Serial.println("Loaded other configuration data from EEPROM");
}

bool tryConnectToSavedWiFi()
{
  WiFi.mode(WIFI_AP_STA);
  String savedSSID, savedPassword;
  loadWiFiCredentials(savedSSID, savedPassword);

  if (savedSSID.length() > 0 && savedPassword.length() > 0)
  {
    Serial.print("Trying to connect to saved SSID: ");
    Serial.print(savedSSID);

    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

    int maxRetries = 10;
    int retries = 0;

    while (WiFi.status() != WL_CONNECTED && retries < maxRetries)
    {
      delay(1000);
      Serial.print(".");
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nSuccessfully connected to saved Wi-Fi");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      inAPMode = false;
      return true;
    }
  }
  Serial.println("Failed to connect to saved Wi-Fi");
  WiFi.mode(WIFI_AP_STA);
  return false;
}

void handle_check_internet_connection()
{
  Serial.println("\nChecking Internet connection...");
  if (server.method() == HTTP_GET)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (client.connect("www.google.com", 443))
      {
        Serial.println("Connected to the Internet");
        Serial.println("Restart the gateway to start sending data to the server");
        server.send(200, "application/json", "{\"internet_connected\": 1, \"wifi_connected\": 1}");
      }
      else
      {
        Serial.println("No Internet access");
        server.send(200, "application/json", "{\"internet_connected\": 0, \"wifi_connected\": 1}");
      }

      client.stop();
    }
    else
    {
      Serial.println("Not connected to Wi-Fi");
      server.send(200, "application/json", "{\"internet_connected\": 0, \"wifi_connected\": 0}");
    }
  }
}

void indicateSuccessfulDataSendToServer()
{
  digitalWrite(BUILTIN_LED, HIGH);
  delay(30);
  digitalWrite(BUILTIN_LED, LOW);
  delay(30);
  digitalWrite(BUILTIN_LED, HIGH);
  delay(30);
  digitalWrite(BUILTIN_LED, LOW);
}

void indicateSuccessfulConnection()
{
  digitalWrite(BUILTIN_LED, HIGH); // Turn the LED on
  delay(3000);                     // Keep the LED on for 3 seconds
  digitalWrite(BUILTIN_LED, LOW);  // Turn the LED off
}

void blinkLEDinErrorPattern(int number_of_blinks)
{
  for (int i = 0; i < number_of_blinks; i++)
  {
    digitalWrite(BUILTIN_LED, HIGH);
    delay(30);
    digitalWrite(BUILTIN_LED, LOW);
    delay(30);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(500);
    digitalWrite(BUILTIN_LED, LOW);
    delay(500);
  }
}

void handle_other_config()
{
  if (server.method() == HTTP_POST && server.uri() == "/configuration/v1/other-config")
  {
    Serial.println("\nReceiving other configuration data");

    JsonDocument doc;
    String requestBody = server.arg("plain");
    DeserializationError error = deserializeJson(doc, requestBody);

    if (error)
    {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      server.send(400, "text/plain", "Invalid JSON");
      return;
    }

    tankSize = doc["tankSize"].as<String>();
    timeZone = doc["timeZone"].as<String>();
    longitude = doc["longitude"].as<String>();
    latitude = doc["latitude"].as<String>();
    loadedHeight = doc["loadedHeight"].as<String>();

    Serial.print("Received TankSize: ");
    Serial.print(tankSize);
    Serial.print("\tTimeZone: ");
    Serial.print(timeZone);
    Serial.print("\tLongitude: ");
    Serial.print(longitude);
    Serial.print("\tLatitude: ");
    Serial.println(latitude);
    Serial.print("\tLoaded Height: ");
    Serial.println(loadedHeight);

    server.send(200, "application/json", "{\"status\": 1}");

    saveOtherConfigDataToEEPROM(tankSize, timeZone, longitude, latitude, loadedHeight);
    Serial.println("Restart the gateway to start sending data to the server");
  }
  else
  {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void handle_connect_to_new_wifi()
{
  if (server.method() == HTTP_POST)
  {
    JsonDocument doc;

    String requestBody = server.arg("plain");
    DeserializationError error = deserializeJson(doc, requestBody);

    if (error)
    {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      server.send(400, "text/plain", "Invalid JSON");
      return;
    }

    String ssid = doc["ssid"].as<String>();
    String password = doc["password"].as<String>();

    Serial.println("\nReceiving Wi-Fi credentials...");
    Serial.print("Received SSID: ");
    Serial.println(ssid);

    Serial.print("Trying to connect to the new Wi-Fi network");

    WiFi.mode(WIFI_AP_STA); // Set mode to both AP and STA
    WiFi.begin(ssid.c_str(), password.c_str());

    int maxRetries = 10;
    int retries = 0;

    while (WiFi.status() != WL_CONNECTED && retries < maxRetries)
    {
      delay(1000);
      Serial.print(".");
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nSuccessfully connected to Wi-Fi");
      server.send(200, "application/json", "{\"status\": 1, \"ssid\": \"" + ssid + "\"}");

      delay(1000);

      saveWiFiCredentials(ssid, password);
      Serial.print("Wi-Fi client IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Wi-Fi server (AP) IP Address: ");
      Serial.println(WiFi.softAPIP());

      indicateSuccessfulConnection();
    }
    else
    {
      Serial.println("\nFailed to connect to Wi-Fi");
      server.send(200, "application/json", "{\"status\": 0, \"ssid\": \"" + ssid + "\"}");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid, ap_password);
      Serial.println("Re-enabled AP mode");
    }
  }
  else
  {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void handle_sync_sensor()
{
  if (server.method() == HTTP_GET)
  {
    Serial.println("\nWaiting for user to press SYNC button on sensor...");
    inSensorSearchingMode = true;
    selected_sensor_mac_address = "NA";
    while (inSensorSearchingMode && inAPMode)
    {
      BLEScanResults foundDevices = pBLEScan->start(scanTimeSeconds, false);
      pBLEScan->clearResults();
      if (selected_sensor_mac_address != "NA")
      {
        server.send(200, "application/json", "{\"status\": 1, \"sync_mac\": \"" + selected_sensor_mac_address + "\"}");
        Serial.println("SYNCed sensor mac sent to the app");
        inSensorSearchingMode = false;
        break;
      }
    }
  }
}

void handle_confirm_synced_sensor()
{
  if (server.method() == HTTP_GET)
  {
    if (selected_sensor_mac_address != "NA")
    {
      Serial.println("Confirmed synced sensor. Writing to EEPROM...");
      bluetooth_sending_status = false;
      EEPROM.begin(EEPROM_SIZE);
      for (int i = SENSOR_MAC_ADDR; i < SENSOR_MAC_ADDR + 50; i++)
        EEPROM.write(i, 0);
      for (int i = 0; i < selected_sensor_mac_address.length(); i++)
        EEPROM.write(SENSOR_MAC_ADDR + i, selected_sensor_mac_address[i]);
      EEPROM.commit();
      Serial.println("Saved mac address of the sensor to the EEPROM");
      server.send(200, "application/json", "{\"status\": 1, \"confirmed_mac\": \"" + selected_sensor_mac_address + "\"}");
    }
  }
}

void setup()
{
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);

  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  server.on("/configuration/v1/wifi-config", HTTP_POST, handle_connect_to_new_wifi);
  server.on("/configuration/v1/other-config", HTTP_POST, handle_other_config);
  server.on("/check/v1/check-internet", HTTP_GET, handle_check_internet_connection);
  server.on("/check/v1/confirm-synced-sensor", HTTP_GET, handle_confirm_synced_sensor);
  server.on("/check/v1/sync-sensor", HTTP_GET, handle_sync_sensor);
  server.on("/", []()
            { server.send(200, "text/plain", "Hi! This is ElegantOTA Demo."); });
  bluetooth_sending_status = false;

  // Try to connect to saved Wi-Fi credentials and load other configuration data
  if (!tryConnectToSavedWiFi() || timeZone == "NA" || tankSize == "NA" || longitude == "NA" || latitude == "NA" || loadedHeight == "NA" || selected_sensor_mac_address == "NA")
  {

    if (!handleButtonPress())
    {
      Serial.println("Automatically trying to connect to the last saved Wi-Fi network.\nPlease press BOOT button to enter AP mode.");
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(ap_ssid, ap_password);
      // Serial.println("Access Point Started");
      // Serial.print("AP IP Address: ");
      // Serial.println(WiFi.softAPIP());
      automatically_put_to_AP_mode = true;
      inAPMode = true;
    }
  }
  else
  {
    inAPMode = false;
    WiFi.mode(WIFI_STA);
    indicateSuccessfulConnection();
    bluetooth_sending_status = true;
    Serial.println("Data loaded from EEPROM.");
    Serial.println("Scanning for Gas sensor of MAC address: " + selected_sensor_mac_address);
  }

  server.begin();

  client.setInsecure(); // For development purposes, skip certificate validation

  timeClient.begin();

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
}

void loop()
{
  server.handleClient();

  if (inAPMode)
  {
    blinkLEDInAPMode();
  }
  else
  {
    handleButtonPress();
    BLEScanResults foundDevices = pBLEScan->start(scanTimeSeconds, false);
    pBLEScan->clearResults();
  }
}
