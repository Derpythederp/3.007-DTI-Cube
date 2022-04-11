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
#define NUM_LEDS 1
#define CHANNEL 1  // Both sender and receiever has to be on the same channel
#define DEBUG_PAIR 0  // if true, then esp_now_is_peer_exist will be called as an additional check

void OnDataRecv();
const int ledPin = 27;
const uint8_t broadcastAddress[6] = {0x58, 0xBF, 0x25, 0x14, 0x50, 0xA0};  // Barebones ESP32 
esp_now_peer_info_t remote;  // set up remote peer info
int ESPNow_Fail_Count = 0;

struct soundncolorslocal {
  float hue;
  bool *buttonState[BUTTON_COUNT];
};  // local is the current device 

struct soundncolorsremote{
  float hue;
  bool *buttonState[REMOTE_BUTTON_COUNT];
};  // remote is the other device


struct soundncolorslocal localData = {{128}, {false}};
struct soundncolorsremote remoteData = {{128}, {false}};


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


void sendData() {
  // Used here to reply to remote with the remoteData which should not change

  Serial.println();
  Serial.println("Sending data to remote:"); 
  Serial.println("Sliders --> ");
  Serial.print(remoteData.hue);
  Serial.println();
  Serial.println("Buttons --> ");
  for (int i = 0; i < REMOTE_BUTTON_COUNT; i++) {
    Serial.print(remoteData.buttonState[i]);
  }
  esp_err_t sendStatus = esp_now_send(broadcastAddress, &remoteData, sizeof(soundncolorsremote));

  if (sendStatus == ESP_OK) {
    Serial.println("Sent data to remote successfully.");
  } else {
    Serial.println("Could not send data to remote.");
  }
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *soundncolors_data, int data_len) {
  
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

  sendData();  // perform the echo, in the final code this is done in loop() as it polls
}


void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, ledPin, RGB>(leds, NUM_LEDS);
  WiFi.mode(WIFI_STA);
  initESPNow();
  esp_now_register_recv_cb(OnDataRecv);
  initBroadcastPeer();
}

void loop() {
  // update led
  
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setHue(localData.hue[j]);
  }
}
