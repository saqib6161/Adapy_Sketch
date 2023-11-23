
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <cmath>
#include <string>
#include <FastLED.h>
#include <Adafruit_INA219.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <Temperature_LM75_Derived.h>


// Created By Muhammad Sa QIB
// current version
#define FIRMWARE_VERSION "2.0.2"


unsigned long lastActivityTime = 0;
const unsigned long resetInterval = 5 * 60 * 1000;  // 5 minutes in milliseconds

#define SCL_1 16
#define SDA_1 17
// Invalid temperature constant
#define INVALID_TEMPERATURE -1000.0
Generic_LM75 temperature;


Adafruit_INA219 ina219;

float tempC = 0.0;  // temperature in Celsius
float tempF = 0.0;  // temperature in Fahrenheit

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
float perc = 0;
int buttonState = 0;
String lastReceivedMessage = "0";
bool readI2C = true;


#define NUM_LEDS 1
// For led chips like WS2812, which have a data line, ground, and power, you just
// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];

#define URL_fIRMWARE_VERSION "https://raw.githubusercontent.com/saqib6161/Adapy_PCB_Firmware/main/version.txt"
#define URL_FIREMWARE_BIN "https://raw.githubusercontent.com/saqib6161/Adapy_PCB_Firmware/main/Adapy_sketch.ino.esp32.bin"

BLEServer *pServer = NULL;  // added
BLECharacteristic *pCharacteristic;
BLECharacteristic *pCharUpdate;        // for update stuff only
BLECharacteristic *pCharBatteryStuff;  // for battery stuff only

bool batteryEnabled = false;
bool deviceConnected = false;
bool oldDeviceConnected = false;  // added
bool wifiConnect = false;
bool stopAllStuffs = false;
bool updateStarted = false;
float txValue = 0;
const int readPin = 32;  // Use GPIO number. See ESP32 board pinouts
const int LED = 2;       // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

String receivedData = "Wait For Me";
String macAddress = "";

bool readySend = true;
uint32_t value = 0;

int incomingByte;

bool startUpdate = false;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_UPDATE "ce30f22b-5c27-4b2b-987c-fc0b80ec7415"  // to handle only OTA or Other update TX + RX

#define CHARACTERISTIC_UUID_BATTERY_STUFF "c8afdebb-16e6-4881-99f5-b6ec337f5b6b"  // to handle only battery releated data being transffered to app
// =================================== RIGHT ================================
const int pinD15 = 15;  // LED is connected to GPIO15
const int pinD2 = 2;    //  LED is  connected to GPI02
const int pinD4 = 4;    //  LED is  connected to GPI04

const int pinD5 = 5;    //  LED is  connected to GPI05
const int pinD18 = 18;  //  LED is  connected to GPI018
const int pinD19 = 19;  //  LED is  connected to GPI019

const int pinD13 = 13;  //  LED is  connected to GPI013

const int pinD14 = 14;  //  LED is  connected to GPI14
const int pinD12 = 12;  //  LED is  connected to GPI12

const int pinD16 = 16;  //  LED is  connected to GPI16
const int pinD17 = 17;  //  LED is  connected to GPI17

const int pinD21 = 21;  //  LED is  connected to GPI021
const int pinD22 = 22;  //  LED is  connected to GPI022
const int pinD23 = 23;  //  LED is  connected to GPI023

// =================================== RIGHT ================================

// =================================== LEFT  ================================

const int pinD27 = 27;  //  LED is  connected to GPI027
const int pinD26 = 26;  //  LED is  connected to GPI026
const int pinD25 = 25;  //  LED is  connected to GPI025
const int pinD33 = 33;  //  LED is  connected to GPI033
const int pinD32 = 32;  //  LED is  connected to GPI032
const int pinD35 = 35;  //  LED is  connected to GPI035 (only input)
const int pinD34 = 34;  //  LED is  connected to GPI034 (only input)

// =================================== LEFT  ================================


void PinSetup() {

  pinMode(pinD14, OUTPUT);
  pinMode(pinD2, OUTPUT);
  pinMode(pinD12, OUTPUT);
  pinMode(pinD4, OUTPUT);
  pinMode(pinD5, OUTPUT);
  pinMode(pinD15, OUTPUT);
  pinMode(pinD18, OUTPUT);
  pinMode(pinD19, OUTPUT);
  pinMode(pinD21, OUTPUT);
  pinMode(pinD22, OUTPUT);
  pinMode(pinD23, OUTPUT);
  pinMode(pinD25, OUTPUT);
  pinMode(pinD26, OUTPUT);
  pinMode(pinD27, OUTPUT);
  pinMode(pinD32, OUTPUT);
  pinMode(pinD33, OUTPUT);
}

void putAllOutputFor0() {

  digitalWrite(pinD12, LOW);
  digitalWrite(pinD2, LOW);
  digitalWrite(pinD14, LOW);
  //digitalWrite(pinD13, HIGH);
  digitalWrite(pinD4, LOW);
  digitalWrite(pinD5, LOW);
  digitalWrite(pinD15, LOW);
  //digitalWrite(pinD18, LOW); // constant
  digitalWrite(pinD19, LOW);
  digitalWrite(pinD21, LOW);  // added
  digitalWrite(pinD22, LOW);  // added
  digitalWrite(pinD23, LOW);
  digitalWrite(pinD25, LOW);
  digitalWrite(pinD26, HIGH);  // constant
  digitalWrite(pinD27, HIGH);  // constant
  digitalWrite(pinD32, LOW);
  digitalWrite(pinD33, LOW);
}

void putHighSpecificPin(int pin) {
  digitalWrite(pin, HIGH);
  if (pin == 18) {
    return;
  }
  Serial.print("Turn Into High PIN Number : ");
  Serial.println(pin);
}

void putLowSpecificPin(int pin) {
  digitalWrite(pin, LOW);
}

void changePinByValue(std::string receivedSignal) {

  lastReceivedMessage = receivedSignal.c_str();

  // from here it is J2 --- J3 Harness Configuration 8 PINS

  if (receivedSignal == "2") {

    //Winch Down
    putHighSpecificPin(pinD2);
  }
  if (receivedSignal == "15") {
    //winch Up
    putHighSpecificPin(pinD15);
  }

  if (receivedSignal == "14") {
    // OUT
    putHighSpecificPin(pinD14);
  }

  if (receivedSignal == "12") {
    // Crane UP
    putHighSpecificPin(pinD12);
  }
  if (receivedSignal == "19") {
    //Down
    putHighSpecificPin(pinD19);
  }

  // Crane IN
  if (receivedSignal == "4") {
    putHighSpecificPin(pinD4);
  }


  //always high constant 12V
  putHighSpecificPin(pinD18);



  // 6 PIN MOLEX NEW SETUP
  if (receivedSignal == "32") {
    putHighSpecificPin(pinD32);
  }

  if (receivedSignal == "21") {  // this oe
    putHighSpecificPin(pinD21);
  }

  if (receivedSignal == "22") {  // trhis ine
    putHighSpecificPin(pinD22);
  }

  if (receivedSignal == "23") {
    putHighSpecificPin(pinD23);
  }


  /*if (receivedSignal == "061") {
    buttonState = digitalRead(pinD23);

    Serial.println(buttonState);
    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH
    if (buttonState == 0) {
      // turn LED on
      putHighSpecificPin(pinD23);
    } else {
      // turn LED off
      digitalWrite(pinD23, LOW);
    }
  }*/


  // 5 MIDI MOLEX SETUP

  if (receivedSignal == "26") {
    putLowSpecificPin(pinD26);
  }

  if (receivedSignal == "27") {
    putLowSpecificPin(pinD27);
  }

  // 3 PIN MOLEX SETUP

  if (receivedSignal == "25") {
    putHighSpecificPin(pinD25);
  }

  if (receivedSignal == "33") {
    putHighSpecificPin(pinD33);
  }

  if (receivedSignal == "0") {
    putAllOutputFor0();
  }
}
String getWifiStatus(int code) {
  if (code == 0) {

    return "WL_IDLE_STATUS";
  } else if (code == 1) {

    return "WL_NO_SSID_AVAIL";
  } else if (code == 2) {

    return "WL_SCAN_COMPLETED";
  } else if (code == 3) {

    return "WL_CONNECTED";
  } else if (code == 4) {

    return "WL_CONNECT_FAILED";
  } else if (code == 5) {

    return "WL_CONNECTION_LOST";
  } else if (code == 6) {

    return "WL_DISCONNECTED";
  } else {
    return "Not Specified";
  }
}


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.print("Server Connected ");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Server Disconnected ");
    putAllOutputFor0();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");

      for (int i = 0; i < rxValue.length(); i++) {
        // Serial.print(rxValue[i]);
      }

      // Serial.println();
      receivedData = rxValue.c_str();
      Serial.print(receivedData);

      // Handle data received from the Bluetooth device
      lastActivityTime = millis();

      Serial.println();
      Serial.println("*********");
      changePinByValue(rxValue.c_str());
      // readySend = true;
    }
  }
};

char receivedUpdateData[50];  //store received data
char wifiSSID[25];
char wifiPassword[50];

// To handlee only update or OTA releated CallBacks
class MyUpdateCallBack : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value in Update: ");

      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
      }

      String updateData = rxValue.c_str();
      strcpy(receivedUpdateData, rxValue.c_str());

      if (strstr(receivedUpdateData, "SSID")) {  //if data received include the word "" (received the latitude)
        strcpy(wifiSSID, (char *)(strstr(receivedUpdateData, "SSID") + 5));
      }

      if (strstr(receivedUpdateData, "PASS")) {  //if data received include the word "" (received the latitude)
        strcpy(wifiPassword, (char *)(strstr(receivedUpdateData, "PASS") + 5));
      }


      Serial.println();
      Serial.print(wifiSSID);
      Serial.println("*********");
      Serial.print(wifiPassword);
      Serial.println("*********");


      if (updateData == "wifi") {
        wifiConnect = true;
      }
      if (updateData == "stop") {
        wifiConnect = false;
      }

      if (updateData == "off") {

        wifiConnect = false;
        WiFi.disconnect();
        Serial.println();
        Serial.print("Wifi diconnected....");
      }

      if (updateData == "start") {


        strcpy(wifiSSID, "Adapy Server");
        strcpy(wifiPassword, "iamsaqib12345");
      }
      if (updateData == "update") {

        // stream method

        startUpdate = true;

        //firmwareUpdate();
      }
      if (updateData == "v") {

        // get version
        FirmwareVersionCheck();
        Serial.println();
        Serial.println("Called check version method... v");
      }
    }
  }
};

void initBle() {
  // Create the BLE Device
  BLEDevice::init("Adapy Mobility Device");  // Give it a name

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  BLECharacteristic *pChar = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);

  pChar->setCallbacks(new MyCallbacks());

  pCharUpdate = pService->createCharacteristic(
    CHARACTERISTIC_UUID_UPDATE,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pCharUpdate->setCallbacks(new MyUpdateCallBack());
  pCharUpdate->addDescriptor(new BLE2902());


  // Create a BLE Characteristic using for sending version and MAC address
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());


  // Create a BLE Characteristic using for pCharBatteryStuff
  pCharBatteryStuff = pService->createCharacteristic(
    CHARACTERISTIC_UUID_BATTERY_STUFF,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  // Create a BLE Descriptor
  pCharBatteryStuff->addDescriptor(new BLE2902());


  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client for BLE connection to notify...");
}


unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long previousMillis_2 = 0;
const long interval = 60000;
const long mini_interval = 4000;

void repeatedCall() {
  //Serial.println();
  //Serial.println("Called Reepeated call meethod...");
  static int num = 0;
  unsigned long currentMillis = millis();

  /*if ((currentMillis - previousMillis) >= interval)
    {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    //updateFirmware();

    if (FirmwareVersionCheck())
    {
      //firmwareUpdate();
    }
    }*/
  static int count = 0;
  if ((currentMillis - previousMillis_2) >= mini_interval) {
    previousMillis_2 = currentMillis;
    Serial.print("idle loop is running at ...");
    Serial.print(num++);
    Serial.print(" Active firmware version:");
    Serial.println(FIRMWARE_VERSION);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("wifi already connected");

      Serial.print(count);

      //wifiConnect = false;
      //ServerStart();
      if (startUpdate) {
        count++;
        if (count == 2) {
          count = 0;
          //delay(5000);
          firmwareUpdate();
          startUpdate = false;
        }
      }

      if (updateStarted) {
        firmwareUpdate();
      }

    } else {
      connect_wifi();
    }
  }
}

int FirmwareVersionCheck(void) {

  Serial.println();
  Serial.println("Called Firmware version check meethod...");

  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fIRMWARE_VERSION;

  Serial.println(fwurl);
  WiFiClientSecure *client = new WiFiClientSecure;

  // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
  HTTPClient https;

  if (https.begin(fwurl)) {  // HTTPS
    Serial.print("[HTTPS] GET...\n");
    // start connection and send HTTP header
    delay(100);
    httpCode = https.GET();
    delay(100);
    if (httpCode == HTTP_CODE_OK)  // if version received
    {
      payload = https.getString();  // save received version
    } else {
      Serial.print("error in downloading version file:");
      Serial.println(httpCode);
    }
    https.end();
  }
  delete client;

  if (httpCode == HTTP_CODE_OK)  // if version received
  {
    payload.trim();
    Serial.print("Payload get :  ");
    Serial.println(payload);
    if (payload.equals(FIRMWARE_VERSION)) {
      Serial.printf("\nDevice already on latest firmware version: %s\n", FIRMWARE_VERSION);
      return 0;
    } else {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  }
  return 0;
}

//char *ssid = "Adapy Server";
//char *password = "iamsaqib12345";

int wifiAttempt = 0;
String hostname = "Adapy Hub";
void connect_wifi() {
  Serial.println();
  Serial.println("Called Wifi connect call meethod...");

  Serial.println(wifiSSID);
  Serial.println("*********");
  Serial.println(wifiPassword);
  Serial.println("Waiting for WiFi Connection ");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    wifiAttempt++;
    delay(500);
    Serial.print(".");
    Serial.println(getWifiStatus(WiFi.status()));

    delay(5);
    int status = WiFi.status();
    char buffer[20];
    dtostrf(status, 1, 0, buffer);
    // customCharacteristic.setValue((char*)&buffer);
    pCharUpdate->setValue((char *)&buffer);
    pCharUpdate->notify();
    delay(5);

    if (wifiAttempt > 20) {
      wifiAttempt = 0;
      wifiConnect = false;

      // to send -1 which is called failed to connect
      int status = -1;
      char buffer[20];
      dtostrf(status, 1, 0, buffer);
      // customCharacteristic.setValue((char*)&buffer);
      pCharUpdate->setValue((char *)&buffer);
      pCharUpdate->notify();
      delay(5);

      WiFi.disconnect();
      Serial.print("Connect 1 FAILED");
      return;
    }
  }

  int status = WiFi.status();
  ;
  char buffer[20];
  dtostrf(status, 1, 0, buffer);
  // customCharacteristic.setValue((char*)&buffer);
  pCharUpdate->setValue((char *)&buffer);
  pCharUpdate->notify();
  delay(5);


  Serial.println("");
  Serial.println("WiFi connected Now");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  //FirmwareVersionCheck();
}

double totalLength;
double currentLength;

// Function to update firmware incrementally
// Buffer is declared to be 128 so chunks of 128 bytes
// from firmware is written to device until server closes

bool bleEnable = true;
void updateFirmware(uint8_t *data, size_t len) {
  Update.write(data, len);

  currentLength += len;
  // Print dots while waiting for update to finish

  double per = (currentLength / totalLength) * 100;
  if (bleEnable) {
    if (round(per) > 10.00) {
      Serial.println("per 100 and ble de init true called");

      bleEnable = false;
    }
  }
  Serial.print("Progress : ");
  Serial.print(round(per));
  Serial.println("%");

  // if current length of written firmware is not equal to total firmware size, repeated
  if (currentLength != totalLength) return;

  updateStarted = false;
  Update.end(true);


  Serial.printf("\nUpdate Success, Total Size: %u\nRebooting...\n", currentLength);

  // Restart ESP32 to see changes
  ESP.restart();
}


void firmwareUpdate(void) {

  totalLength = 0;
  currentLength = 0;

  Serial.println();
  Serial.println("Called updateFirmware with stream reader method...");

  WiFiClientSecure client;

  HTTPClient https;
  int httpCode;
  String payload;
  int totalSize;

  if (https.begin(URL_FIREMWARE_BIN)) {
    // HTTPS

    Serial.print("[HTTPS] GET...\n");
    // start connection and send HTTP header
    delay(100);
    httpCode = https.GET();
    delay(100);
    if (httpCode == HTTP_CODE_OK)  // if version received
    {
      payload = https.getString();  // save received version
      totalSize = https.getSize();
      Serial.print("get this from server via request ");
      Serial.println(payload);

      Serial.printf("FW Size: %u\n", totalSize);
      Serial.println();

      totalLength = https.getSize();
      // transfer to local variable
      int len = totalLength;
      // this is required to start firmware update process
      Update.begin(UPDATE_SIZE_UNKNOWN);
      Serial.printf("FW Size: %u\n", totalLength);

      // create buffer for read
      uint8_t buff[128] = { 0 };
      // get tcp stream
      WiFiClient *stream = https.getStreamPtr();
      // read all data from server
      Serial.println("Updating firmware...");

      //stopAllStuffs = true;
      //wifiConnect = false;

      int status = 200;
      char buffer[20];
      dtostrf(status, 1, 0, buffer);
      // customCharacteristic.setValue((char*)&buffer);
      //pCharUpdate->setValue((char *)&buffer);
      //pCharUpdate->notify();
      delay(10);


      while (https.connected() && (len > 0 || len == -1)) {
        // get available data size

        size_t size = stream->available();
        if (size) {
          // read up to 128 byte
          int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));

          // pass to function
          updateFirmware(buff, c);
          if (len > 0) {
            len -= c;
          }
        }
        delay(1);
      }
      if (!https.connected()) {
        updateStarted = true;
        // if the server's disconnected, stop the client:
        Serial.println("http disconnected between");
        Update.end(true);
        delay(200);
      }
    } else {
      Serial.print("error in downloading version file:");
      Serial.println(httpCode);
    }
  } else {
    Serial.print("unable to connect http request");
  }
  https.end();
  // delete client;
}

void begainI2C() {
  bool status = Wire.begin(SDA_1, SCL_1);
  Serial.print("Wire Configuration Status is :");
  Serial.println(status);
}



void setup() {
  Serial.begin(115200);
  // Start i2c with 17, 16 pins
  begainI2C();

  PinSetup();
  putAllOutputFor0();

  FastLED.addLeds<NEOPIXEL, pinD13>(leds, NUM_LEDS);
  //FastLED.addLeds<WS2812, pinD13, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(1);

  // to init ble call function
  initBle();

  Serial.print("Current Firmware version is : ");
  Serial.println(FIRMWARE_VERSION);

  macAddress = WiFi.macAddress();
  Serial.print("ESP Board MAC Address Now :  ");
  Serial.println(macAddress);


  //batterySetup();
  //tempSensorSetup();
}

void checkToReconnect()  // added
{
  // disconnected so advertise
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Disconnected: start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connected so reset boolean control
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    Serial.println("Reconnected");
    oldDeviceConnected = deviceConnected;
  }
}



void sendSensorDataToApp(String data) {
  char buffer[data.length() + 1];
  data.toCharArray(buffer, macAddress.length() + 1);
  pCharBatteryStuff->setValue((char *)&buffer);
  pCharBatteryStuff->notify();
  //Serial.print("Data Sended To App : ");
  //Serial.println(data);
}
void selectI2CIndex(uint8_t bus) {

  if (bus > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}


void readI2ConnectedDevice() {

  selectI2CIndex(0);

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    busvoltage = 0;
  } else {
    Serial.println("Measuring voltage and current with INA219 ... Started");
    busvoltage = ina219.getBusVoltage_V();
    Serial.print("Bus Voltage:   ");
    Serial.print(busvoltage);
    Serial.println(" V");
  }

  delay(100);
  selectI2CIndex(1);

  float tempCHere = temperature.readTemperatureC();

  if (isnan(tempCHere)) {
    Serial.println("Error reading temperature from LM75 sensor!");
    tempC = 0.0;
  } else if (tempCHere == INVALID_TEMPERATURE) {
    Serial.println("Invalid temperature reading from LM75 sensor!");
    tempC = 0.0;
  } else if (tempCHere == -0.50) {
    Serial.println("Not Connected Sensor yet");
    tempC = 0.0;
  } else {
    // Check for any out-of-range values or anomalies
    if (tempCHere < -50 || tempCHere > 150) {
      Serial.println("Temperature reading out of range!");
      tempC = 0.0;
    } else {
      tempC = tempCHere;
      // Print the valid temperature value
      Serial.print("Temperature: ");
      Serial.print(tempC);
      Serial.println(" °C");
    }
  }
}
unsigned long previousMillis_3 = 0;  // will store last time was updated
const long interval_3 = 2000;

void startTimerAndPerfomI2C() {
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis_3) >= interval_3) {
    // save the last time you blinked the LED
    previousMillis_3 = currentMillis;

    Serial.print("timer ... ");
    Serial.println(currentMillis);

    //getTempFromSensor();
    readI2ConnectedDevice();

    String data;
    data = String(busvoltage) + "," + String(tempC);
    //  to send combine battery and temp sensor to app from here
    sendSensorDataToApp(data);
    //delay(2000);
  }
}

void loop() {

  // Check if it's time to reset the ESP32
  if (millis() - lastActivityTime >= resetInterval && !updateStarted) {
    Serial.println("Called Restart method now ");
    BLEDevice::deinit();
    if (!deviceConnected) {
      leds[0] = CRGB::Red;
      // Show the leds (only one of which is set to white, from above)
      FastLED.show();
    }
    delay(2000);
    // Perform a reset
    ESP.restart();
  }

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'U') {
      Serial.println("Firmware Update In Progress..");
      //firmwareUpdate();
      firmwareUpdate();
    }
    if (incomingByte == 'W') {
      Serial.println("Firmware Update In Progress..");
      strcpy(wifiSSID, "New Server");
      strcpy(wifiPassword, "iamsaqib12345");
      wifiConnect = true;
    }
  }

  /* if (lastReceivedMessage == "0") {
    putAllOutputFor0();
  }*/

  if (deviceConnected) {
    leds[0] = CRGB::Green;
    // Show the leds (only one of which is set to white, from above)
    FastLED.show();

    if (readySend) {
      macAddress += ",";
      macAddress += FIRMWARE_VERSION;
      // to send MAC address + current version to service
      char buffer[macAddress.length() + 1];
      macAddress.toCharArray(buffer, macAddress.length() + 1);
      // customCharacteristic.setValue((char*)&buffer);
      pCharacteristic->setValue((char *)&buffer);
      pCharacteristic->notify();
      Serial.println("MAC AND VERSION SEND TO APP ...");
      delay(5);
      readySend = false;
    }

    // to get the temp sensor reading
    if (readI2C) {
      //readI2C = false;
      //Serial.println("timer ... ok for run ");
      startTimerAndPerfomI2C();
    }


  } else {
    readySend = true;

    leds[0] = CRGB::Red;
    // Show the leds (only one of which is set to white, from above)
    FastLED.show();
  }


  checkToReconnect();
  if (wifiConnect) {
    repeatedCall();
  } else {
  }
  delay(1);
}
