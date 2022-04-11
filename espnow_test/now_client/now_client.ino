// Simple echo test with ESPNow
// There will be two modules:
// 1) Barebones ESP32 will the the client, will have a readStringUntil("\r\n") which will let you change the hue of the remote boi
// 2) ESP32 with all the electronics will be the server, and it will change its singular WS2812B LED strip color
// while having an callback for recieving data that it will send soundncolorsremote back

#include <WiFi.h>
#include <FastLED.h>
#include <esp_now.h>
#define BUTTON_COUNT 1
#define REMOTE_BUTTON_COUNT 1
#define MAX_ESPNOW_FAILURES 10
#define CHANNEL 1  // Both sender and receiever has to be on the same channel
#define DEBUG_PAIR 0  // if true, then esp_now_is_peer_exist will be called as an additional check

void OnDataRecv();
const uint8_t broadcastAddress[6] = {0x9C, 0x9C, 0x1F, 0xDD, 0x52, 0xF0};  // ESP32 with full electronics
esp_now_peer_info_t remote;  // set up remote peer info
int ESPNow_Fail_Count = 0;

// Cos I keep forgetting struct syntax:
// struct __somename__ {}; will declare a struct type of __somename__
// 
// typedef struct __somename__ __alias__ will create an alias for struct __somename__ as __alias__
// This lets me do some lazy declaration
//
// Mixing the two: 
// typedef struct __somename__ {} __alias__; 
// ^ if you look carefully you just replaced __somename__ in the typedef line with the declaration of __somename__{}

struct soundncolorslocal {
  float hue;
  bool *buttonState[BUTTON_COUNT];
};  // local is the current device 

struct soundncolorsremote{
  float hue;
  bool *buttonState[REMOTE_BUTTON_COUNT];
};  // remote is the other device

struct soundncolorslocal localData = {{128}, {false}};  // localData is what is sent over
struct soundncolorsremote remoteData = {{128}, {false}};  // remoteData is what is to be sent out


void initESPNow() {
  // Init ESP-NOW
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow has been initialized.");
    return;
  }
  else {
    ESPNow_Fail_Count++;
    Serial.println("ESPNow initialization failed... Retrying...");
    if (ESPNow_Fail_Count > MAX_ESPNOW_FAILURES) {
      Serial.println("Too many failures. Restarting board.");
      ESP.restart();
  }
}


void sendData() {
  // Used by barebones ESPNow to send soundncolorsremote to broadcastAddress
  Serial.println();
  Serial.println("Sending data to remote:"); 
  Serial.println("Sliders --> ");
  Serial.print(remoteData.hue);
  Serial.println();
  Serial.println("Buttons --> ");
  for (int i = 0; i < REMOTE_BUTTON_COUNT; i++) {
    Serial.print(remoteData.buttonState[i]);
  }
  Serial.println();
  esp_err_t sendStatus = esp_now_send(broadcastAddress, &remoteData, sizeof(soundncolorsremote));

  if (sendStatus == ESP_OK) {
    Serial.println("Sent data to remote successfully.");
  } else {
    Serial.println("Could not send data to remote.");
  }
}

void initBroadcastPeer() {
  memset(&remote, 0, sizeof(esp_now_peer_info_t));
  memcpy(remote.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  remote.channel = CHANNEL;
  remote.encrypt = false;
  
  if (esp_now_add_peer(&remote) != ESP_OK) {
    Serial.println("Failed to add peer");
  }

  if (DEBUG_PAIR) {
    if (esp_now_is_peer_exist(peer_addr)) {
      Serial.println("LOL you did not sucessfully pair with remote.");
    } else {
      Serial.println("Remote has been paired with successfully");
    }
  }
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  // If received button and sensor data, update struct of button and sensor data
  // I know it sounds counter intutive but localData is data that is destined for the current device
  Serial.println();
  Serial.println("Data received!");
  memcpy(&localData, incomingData, sizeof(soundncolorslocal));

  Serial.println("Data from remote:"); 
  Serial.println("Sliders --> ");
  Serial.print(localData.hue);
  Serial.println();
  Serial.println("Buttons --> ");
  for (int i = 0; i < BUTTON_COUNT; i++) {
    Serial.print(localData.buttonState[i]);
  }
  Serial.println();
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  initESPNow();
  esp_now_register_recv_cb(OnDataRecv);
  initBroadcastPeer();
}


void loop() {
  if (Serial.available()) {
    if (Serial.readStringUntil("\n") == "Send") {
      Serial.println("Send Hue below as an integer from 0-255");
      int soundncolorsremote.hue[0] = Serial.readStringUntil("\n").toFloat();
      sendData();
    }
  }
}
