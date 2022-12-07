
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


#define NUM_LEDS 1
// For led chips like WS2812, which have a data line, ground, and power, you just
// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];

// Created By Muhammad Sa QIB
// current version
#define FIRMWARE_VERSION "1.1.3"


#define URL_fIRMWARE_VERSION "https://raw.githubusercontent.com/saqib6161/Adapy_PCB_Firmware/main/version.txt"
#define URL_FIREMWARE_BIN "https://raw.githubusercontent.com/saqib6161/Adapy_PCB_Firmware/main/Adapy_sketch.ino.esp32.bin"

BLEServer *pServer = NULL; // added
BLECharacteristic *pCharacteristic;
BLECharacteristic *pCharUpdate; // for update stuff
bool deviceConnected = false;
bool oldDeviceConnected = false; // added
bool wifiConnect = false;
bool stopAllStuffs = false;
bool updateStarted = false;
float txValue = 0;
const int readPin = 32; // Use GPIO number. See ESP32 board pinouts
const int LED = 2;      // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

String receivedData = "Wait For Me";
String macAddress = "";

bool readySend = true;
uint32_t value = 0;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UART service UUID
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_UPDATE "ce30f22b-5c27-4b2b-987c-fc0b80ec7415"

// =================================== RIGHT ================================
const int pinD15 = 15; // LED is connected to GPIO15
const int pinD2 = 2;   //  LED is  connected to GPI02
const int pinD4 = 4;   //  LED is  connected to GPI04

const int pinD5 = 5;   //  LED is  connected to GPI05
const int pinD18 = 18; //  LED is  connected to GPI018
const int pinD19 = 19; //  LED is  connected to GPI019

const int pinD13 = 13; //  LED is  connected to GPI013

const int pinD14 = 14; //  LED is  connected to GPI14
const int pinD12 = 12; //  LED is  connected to GPI12

const int pinD21 = 21; //  LED is  connected to GPI021
const int pinD22 = 22; //  LED is  connected to GPI022
const int pinD23 = 23; //  LED is  connected to GPI023

// =================================== RIGHT ================================

// =================================== LEFT  ================================

const int pinD27 = 27; //  LED is  connected to GPI027
const int pinD26 = 26; //  LED is  connected to GPI026
const int pinD25 = 25; //  LED is  connected to GPI025
const int pinD33 = 33; //  LED is  connected to GPI033
const int pinD32 = 32; //  LED is  connected to GPI032
const int pinD35 = 35; //  LED is  connected to GPI035
const int pinD34 = 34; //  LED is  connected to GPI034

// =================================== LEFT  ================================


void PinSetup()
{

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
  pinMode(pinD34, OUTPUT);
  pinMode(pinD35, OUTPUT);


}

void putAllOutputFor0()
{

  digitalWrite(pinD12, LOW);
  digitalWrite(pinD2, LOW);
  digitalWrite(pinD14, LOW);
  //digitalWrite(pinD13, HIGH);
  digitalWrite(pinD4, LOW);
  digitalWrite(pinD5, LOW);
  digitalWrite(pinD15, LOW);
  //digitalWrite(pinD18, LOW); // constant
  digitalWrite(pinD19, LOW);
  digitalWrite(pinD21, LOW); // added
  digitalWrite(pinD22, LOW);  // added
  digitalWrite(pinD23, LOW);
  digitalWrite(pinD25, LOW);
  digitalWrite(pinD26, LOW); // constant
  digitalWrite(pinD27, LOW); // constant
  digitalWrite(pinD32, LOW);
  digitalWrite(pinD33, LOW);
  digitalWrite(pinD34, LOW);
  digitalWrite(pinD35, LOW);
}

void putHighSpecificPin(int pin)
{
  digitalWrite(pin, HIGH);
  if (pin == 18) {
    return;
  }
  Serial.print("Turn Into High PIN Number : ");
  Serial.println(pin);
}

void putLowSpecificPin(int pin)
{
  digitalWrite(pin, LOW);
}

void changePinByValue(std::string receivedSignal) {

  // from here it is J2 --- J3 Harness Configuration 8 PINS

  if (receivedSignal == "2") {

    //Winch Down
    putHighSpecificPin(pinD2);
  }
  if (receivedSignal == "15")
  {
    //winch Up
    putHighSpecificPin(pinD15);
  }

  if (receivedSignal == "14")
  {
    // OUT
    putHighSpecificPin(pinD14);
  }

  if (receivedSignal == "12")
  {
    // Crane UP
    putHighSpecificPin(pinD12);
  }
  if (receivedSignal == "19")
  {
    //Down
    putHighSpecificPin(pinD19);
  }

    // Crane IN
  if (receivedSignal == "4")
  {
    putHighSpecificPin(pinD4);
  }

  
  //always high constant 12V
  putHighSpecificPin(pinD18);



  // 6 PIN MOLEX SETUP
   if (receivedSignal == "32")
  {
    // Changed this pin from 35 to 22
    putHighSpecificPin(pinD22);
  }

  if (receivedSignal == "35")
  {
    // Changed this pin from 35 to 32
    putHighSpecificPin(pinD32);
  }
  
   if (receivedSignal == "23")
  {
    putHighSpecificPin(pinD23);
  }

  if (receivedSignal == "34")
  {
    // Changed this pin from 34 to 21
    putHighSpecificPin(pinD21);
  }

  // 3 PIN MOLEX SETUP 
   if (receivedSignal == "25")
  {
    putHighSpecificPin(pinD25);
  }

  if (receivedSignal == "33")
  {
    putHighSpecificPin(pinD33);
  }

  if (receivedSignal == "0")
  {
    putAllOutputFor0();
  }
}
String getWifiStatus(int code)
{
  if (code == 0)
  {

    return "WL_IDLE_STATUS";
  }
  else if (code == 1)
  {

    return "WL_NO_SSID_AVAIL";
  }
  else if (code == 2)
  {

    return "WL_SCAN_COMPLETED";
  }
  else if (code == 3)
  {

    return "WL_CONNECTED";
  }
  else if (code == 4)
  {

    return "WL_CONNECT_FAILED";
  }
  else if (code == 5)
  {

    return "WL_CONNECTION_LOST";
  }
  else if (code == 6)
  {

    return "WL_DISCONNECTED";
  }
  else
  {
    return "Not Specified";
  }
}


class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
      deviceConnected = true;
      Serial.print("Server Connected ");
    };

    void onDisconnect(BLEServer *pServer)
    {
      deviceConnected = false;
      Serial.print("Server Disconnected ");
      putAllOutputFor0();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0)
      {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++)
        {
          // Serial.print(rxValue[i]);
        }

        // Serial.println();
        receivedData = rxValue.c_str();
        Serial.print(receivedData);

        Serial.println();
        Serial.println("*********");
        changePinByValue(rxValue.c_str());


        // readySend = true;
      }
    }
};

char receivedUpdateData[50]; //store received data
char wifiSSID[25];
char wifiPassword[50];

class MyUpdateCallBack : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0)
      {
        Serial.println("*********");
        Serial.print("Received Value in Update: ");

        for (int i = 0; i < rxValue.length(); i++)
        {
          Serial.print(rxValue[i]);
        }

        String updateData = rxValue.c_str();
        strcpy(receivedUpdateData, rxValue.c_str());

        if (strstr(receivedUpdateData, "SSID")) { //if data received include the word "" (received the latitude)
          strcpy(wifiSSID, (char *)(strstr(receivedUpdateData, "SSID") + 5));
        }

        if (strstr(receivedUpdateData, "PASS")) { //if data received include the word "" (received the latitude)
          strcpy(wifiPassword, (char *)(strstr(receivedUpdateData, "PASS") + 5));
        }


        Serial.println();
        Serial.print(wifiSSID);
        Serial.println("*********");
        Serial.print(wifiPassword);
        Serial.println("*********");


        if (updateData == "wifi")
        {
          wifiConnect = true;
        }
        if (updateData == "stop")
        {
          wifiConnect = false;
        }

        if (updateData == "off")
        {

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

          firmwareUpdate();

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

void initBle()
{
  // Create the BLE Device
  BLEDevice::init("Adapy Mobility Device"); // Give it a name

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  BLECharacteristic *pChar = pService->createCharacteristic(
                               CHARACTERISTIC_UUID_RX,
                               BLECharacteristic::PROPERTY_WRITE |
                               BLECharacteristic::PROPERTY_READ);

  pChar->setCallbacks(new MyCallbacks());

  pCharUpdate = pService->createCharacteristic(
                  CHARACTERISTIC_UUID_UPDATE,
                  BLECharacteristic::PROPERTY_WRITE |
                  BLECharacteristic::PROPERTY_READ |
                  BLECharacteristic::PROPERTY_NOTIFY);

  pCharUpdate->setCallbacks(new MyUpdateCallBack());
  pCharUpdate->addDescriptor(new BLE2902());


  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client for BLE connection to notify...");

}


unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long previousMillis_2 = 0;
const long interval = 60000;
const long mini_interval = 4000;

void repeatedCall()
{
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

  if ((currentMillis - previousMillis_2) >= mini_interval)
  {
    previousMillis_2 = currentMillis;
    Serial.print("idle loop is running at ...");
    Serial.print(num++);
    Serial.print(" Active firmware version:");
    Serial.println(FIRMWARE_VERSION);
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("wifi already connected");


      if (updateStarted) {
        firmwareUpdate();
      }

    }
    else
    {
      connect_wifi();
    }

  }
}

int FirmwareVersionCheck(void)
{

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

  if (https.begin(fwurl))
  { // HTTPS
    Serial.print("[HTTPS] GET...\n");
    // start connection and send HTTP header
    delay(100);
    httpCode = https.GET();
    delay(100);
    if (httpCode == HTTP_CODE_OK) // if version received
    {
      payload = https.getString(); // save received version
    }
    else
    {
      Serial.print("error in downloading version file:");
      Serial.println(httpCode);
    }
    https.end();
  }
  delete client;

  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    Serial.print("Payload get :  ");
    Serial.println(payload);
    if (payload.equals(FIRMWARE_VERSION))
    {
      Serial.printf("\nDevice already on latest firmware version: %s\n", FIRMWARE_VERSION);
      return 0;
    }
    else
    {
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
void connect_wifi()
{
  Serial.println();
  Serial.println("Called Wifi connect call meethod...");

  Serial.println(wifiSSID);
  Serial.println("*********");
  Serial.println(wifiPassword);
  Serial.println("Waiting for WiFi Connection ");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED)
  {
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

  int status = WiFi.status();;
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
void updateFirmware(uint8_t *data, size_t len) {
  Update.write(data, len);
  currentLength += len;
  // Print dots while waiting for update to finish

  double per = (currentLength / totalLength) * 100;

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


void firmwareUpdate(void)
{

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
    if (httpCode == HTTP_CODE_OK) // if version received
    {
      payload = https.getString(); // save received version
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
      uint8_t buff[128] = {0};
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
      pCharUpdate->setValue((char *)&buffer);
      pCharUpdate->notify();
      delay(10);

      while (https.connected() && (len > 0 || len == -1))
      {
        // get available data size

        size_t size = stream->available();
        if (size)
        {
          // read up to 128 byte
          int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));

          // pass to function
          updateFirmware(buff, c);
          if (len > 0)
          {
            len -= c;
          }
        }
        delay(1);
      }
      if (!https.connected())
      {
        updateStarted = true;
        // if the server's disconnected, stop the client:
        Serial.println("http disconnected between");
        delay(200);
      }
    }
    else
    {
      Serial.print("error in downloading version file:");
      Serial.println(httpCode);
    }
  }
  else {
    Serial.print("unable to connect http request");
  }
  https.end();
  // delete client;
}

void setup()
{
  Serial.begin(115200);

  PinSetup();
  FastLED.addLeds<NEOPIXEL, pinD13>(leds, NUM_LEDS);
  //FastLED.addLeds<WS2812, pinD13, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(1);
  putAllOutputFor0();

  // to init ble call function
  initBle();

  Serial.print("Current Firmware version is : ");
  Serial.println(FIRMWARE_VERSION);

  macAddress = WiFi.macAddress();
  Serial.print("ESP Board MAC Address Now :  ");
  Serial.println(macAddress);

}

void checkToReconnect() // added
{
  // disconnected so advertise
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Disconnected: start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connected so reset boolean control
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    Serial.println("Reconnected");
    oldDeviceConnected = deviceConnected;
  }
}

void loop()
{

  delay(1);


  if (deviceConnected)
  {
    if (readySend)
    {
      macAddress += ",";
      macAddress += FIRMWARE_VERSION;
      // to send MAC address + current version to service
      char buffer[macAddress.length() + 1];
      macAddress.toCharArray(buffer, macAddress.length() + 1);
      // customCharacteristic.setValue((char*)&buffer);
      pCharacteristic->setValue((char *)&buffer);
      pCharacteristic->notify();
      delay(5);
      readySend = false;
    }
  }

  checkToReconnect();
  if (wifiConnect)
  {
    repeatedCall();
  }
  else {

  }

  //delay(1000);
  leds[0] = CRGB::Green;
  // Show the leds (only one of which is set to white, from above)
  FastLED.show();

  // delay(1000);
}
