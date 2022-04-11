#include <WiFi.h>
#include <FastLED.h>
#include <SPI.h>
#include <SD.h>
#include "read_write.h"
#include "XT_DAC_AUDIO/XT_DAC_Audio.h"

#define SLIDER_DIST 30
#define SLIDER_COUNT 1
#define BUTTON_COUNT 1
#define PINGDELAY 100
#define SOUND_MUL 0.0343
#define COLOUR_STEP 0.5
#define NUM_LEDS 10

// Pins
/* 
ESP32 in use is the 30 pin version, the flash pins are not exposed.
ESP32 is wired currently with:
Common Cathode LED (To be replaced with WS2812B which takes only one data line)
-------------------------------------------------
Red - D27
Green - D26
Blue - D25

WS2812B
--------
Data - D27
// Might have a second one
If no one interacts with cube, i.e change to distance is below threshold and 
the button is not pressed, the microcontroller turns the light to white.
See if need to split the pins

Ultrasound sensor for single slider(viewed from back)
----------------------
Trigger - D13
Shared Echo - D14

Buttons
-------
BT1/BT2/BT3 - D15


Speaker 
-------------
Input - D21

SD Card Reader(TBA)
-------------------
SPI so 4 pins expected

Candidate SPI channels are:
SPI   |MOSI    |MISO    |CLK     |CS
VSPI  |GPIO 23 |GPIO 19 |GPIO 18 |GPIO 5
HSPI  |GPIO 13 |GPIO 12 |GPIO 14 |GPIO 15
Flash SPI is taken up already in 30 pin version, and you can't use it anyways

Use VSPI
MOSI - D23
MISO - D19
CLK - D18
CS - D5

*/

const int trigPins[SLIDER_COUNT] = {13};  
const int echoPin = 14;  
const int buttonPins[BUTTON_COUNT] = {15};    
const int speakerPin = 21;
const int SD_MOSI = 23;
const int SD_MISO = 19;
const int SD_CLK = 18;
const int SD_CS = 5;
const int ledPin = 27;
const uint8_t broadcastAddress[6] = {};  // to be added 
bool buttonState = 0;

// XT_Wav_Class 

// Stored states, buttons and hue value
struct {
  float hue;
  float distances[SLIDER_COUNT];
} soundncolors;  // for serializing data

float hue;
float distances[SLIDER_COUNT];  // distances in cm in order of sensor, slider count is the remote one actually
unsigned long lastPing;
volatile unsigned long startEcho;
volatile unsigned long stopEcho;
float old_hue = 128.0;


void ISR_ECHO();
void ISR_BUTTON();
void OnDataSent();
void OnDataRecv();

void setup() {
  Serial.begin(115200);
  Serial.println();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;  // set up buffer for peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  for (int i = 0; i < SLIDER_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
  }
  
  for (int i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT);
  }

  SD.begin(SD_CS);  // SD object is a singleton for one SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failure");
    return;
  }

  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  
  File file = SD.open("/joke.txt")  // obtain file handle
  if (!file) {
    Serial.println("joke.txt does not exist. Creating file...");
    writeFile(SD, "/joke.txt", "Why did the chicken cross the road? \r\nHow else is she going to get to the other side?"); 
  }
  else {
    Serial.println("No one likes to listen to the same awful joke twice");
  }
  
  file.close();  // ensure you close your file handle when you finish writing
  
  pinMode(echoPin, INPUT);  // shared echo pin

  pinMode(speakerPin, OUTPUT);
 
  attachInterrupt(digitalPinToInterrupt(echoPin), ISR_ECHO, CHANGE);  // interrupt CPU 0 when slider detects change, might cause button to halt
  attachInterrupt(digitalPinToInterrupt(buttonPin), ISR_BUTTON, FALLING);  // interrupt CPU 0 when button is released
  lastPing = millis();  // get current milisecond for delay later
  FastLED.addLeds<WS2812B, ledPin, RGB>(leds, NUM_LEDS);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *soundncolors_data, int data_len) {
  // If received button and sensor data, update struct of button and sensor data
  memcpy(&soundncolors, &soundncolors_data, sizeof(soundncolors));
}

float distance2colourval(float dist) {
  // Expects a distance in cm
  // Return value from 0.0 to 255.0 if distance is less than max slider distance, else returns 0
  return (dist < SLIDER_DIST) ? ((dist - 2) / (SLIDER_DIST - 2)) * 255 : 0;
}

void doMeasurement() {
  // Does measurement of distances, and returns an array of n distances
  unsigned long startWait;
  unsigned long timeout = 50;  // 23ms is 4m which is already out of range of sensor, we just gave it 27ms more than it needed.

  for (int s = 0; s < SLIDER_COUNT; s++) {
    startEcho = 0;  // Reset start
    stopEcho = 0;  // Reset stop time

    digitalWrite(trigPins[s], HIGH); 
    delayMicroseconds(10);
    digitalWrite(trigPins[s], LOW);

    startWait = millis();  // save start time for echo signal, for timeout detection

    while (startEcho == 0 or stopEcho == 0) {
      if (startWait + timeout < millis()) {
        // Timed out, distance of 0
        startEcho = 0;
        stopEcho = 0;
        break;
      }
    }
    noInterrupts();  // disable interrupts temporarily just in case other sensor does things
    distances[s] = ((stopEcho - startEcho) / 2) * SOUND_MUL;
    interrupts();
  }
}


void checkButton() {
  // Check if button has been pressed, and if it is, play sound if the timing is fufilled
  if (buttonState = true) {
    Serial.printf("Buttons has been pressed");  // add play sound here
    buttonState ^= 1;
  }
}

void setColours(float hue) {
  // expects a float representing hue from 0.0 to 255.0
  // Will set the WS2812B to the appropriate colour
  
  // cross fade code
  old_hue = (hue > old_hue) ? old_hue + COLOUR_STEP : old_hue - COLOUR_STEP;
  
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setHue(old_hue);
  }
  FastLED.show();
}

void loop() {
  // loop mainly does a check for time from last ping, and if it has been long enough it does a measurement, followed by updating the LED with the output
  // loop also checks for button presses, if any buttonPins is pressed, or pulled high, for now it just prints 

  if (millis() - lastPing >= PINGDELAY) {
    doMeasurement();
    float hue = distance2colourval(distances[0]);
    Serial.println(String("Slider Sensor: ") + distances[0] + " cm");  // print distance
    setColours(hue);
  }
  
  checkButton();
  // if (millis() - lastNote >= NOTEDELAY[CURR_NOTE]) && (randomButton ==  {
    // playWav(CURR_NOTE, speakerPin);
}


void IRAM_ATTR ISR_ECHO() {
  // Set up ISR to be stored in RAM
  // Further speedups with direct port register control https://www.instructables.com/Faster-ESP32/ --> ESP8266 and ESP32 as it is is fast enough already, plus digitalRead is an alias
  int pinRead = digitalRead(echoPin);
  if (pinRead) {
    // High state --> Start timer for echo
    startEcho = micros();
  }
  else {
    // Low state --> Stop timer for echo
    stopEcho = micros();
  }
}

void IRAM_ATTR ISR_BUTTON() {
  buttonState ^= 1;
}
