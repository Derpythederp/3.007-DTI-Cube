#include <WiFi.h>
#include <FastLED.h>
#include <SPI.h>
#include <SD.h>
#include "XT_DAC_AUDIO/XT_DAC_Audio.h"

#define SLIDER_DIST 15

#define PINGDELAY 1000
#define SOUND_MUL 0.0343
#define COLOUR_STEP 0.5

#define REMOTE_BUTTON_COUNT 1
#define MAX_ESPNOW_FAILURES 10
#define NUM_STRIPS 2
#define CHANNEL 1  // Both sender and receiever has to be on the same channel
#define DEBUG_PAIR 0  // if true, then esp_now_is_peer_exist will be called as an additional check
#define COLOUR_STEP 0.2
#define NUM_SONGS 4
#define DEBOUNCEINTERVAL 175
#define DEBUG true

// Pins
/* 
ESP32 in use is the 30 pin version, the flash pins are not exposed.
ESP32 is wired currently with:

WS2812B
--------
Data (Line 1) - D26
Data (Line 2) - D27
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
Input - D25 (DAC)

SD Card Reader (To be removed)
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


// Pin Definitions
const int trigPin = 13;
const int echoPin = 14;  
const int speakerPin = 25;
const int ledPins = {26, 27};
const int buttonPin = 15;
//const int SD_MOSI = 23;
//const int SD_MISO = 19;
//const int SD_CLK = 18;
//const int SD_CS = 5;

// Networking Definitions
const uint8_t broadcastAddress[6] = {0x58, 0xBF, 0x25, 0x14, 0x50, 0xA0};  // ESP32 Soldered to module
//const uint8_t broadcastAddress[6] = {0x9C, 0x9C, 0x1F, 0xDD, 0x52, 0xF0};  // ESP32 with full electronics
void OnDataRecv();
esp_now_peer_info_t remote; 
int ESPNow_Fail_Count = 0;


// LED definitions 
float oldHue = 128;  // for cross fade
const int ledCounts[NUM_STRIPS] = {1, 1};  // should error as a compile check if NUM_STRIPS doesn't match
CRGB ledOne[ledCounts[0]];
CRGB ledTwo[ledCounts[1]];

// Ultrasonic sensor Definitions
unsigned long lastPing;
volatile unsigned long startEcho;  // volatile is needed for shared variables between ISR and loop to prevent compiler from deleting it via optimization
volatile unsigned long stopEcho;
float old_hue = 128.0;
float distance;

// Button definitions
int lastClicked = 0;

// Interrupt Service Routine
void ISR_ECHO();
void ISR_BUTTON();


// Note Status and network structs
struct soundncolors {
  float hue;
  int noteCount;
}

struct soundncolors localData;  // buffer to store incoming data for deserialization
struct soundncolors remoteData;  // buffer to store outgoing data

// Music definitions 
XT_MusicScore_Class *currentMusic;

int8_t PROGMEM TwinkleTwinkle[] = {
  NOTE_C5,NOTE_C5,NOTE_G5,NOTE_G5,NOTE_A5,NOTE_A5,NOTE_G5,BEAT_2,
  NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,NOTE_D5,NOTE_C5,BEAT_2,
  NOTE_G5,NOTE_G5,NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,BEAT_2,
  NOTE_G5,NOTE_G5,NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,BEAT_2,
  NOTE_C5,NOTE_C5,NOTE_G5,NOTE_G5,NOTE_A5,NOTE_A5,NOTE_G5,BEAT_2,
  NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,NOTE_D5,NOTE_C5,BEAT_4,  
  NOTE_SILENCE,BEAT_5,SCORE_END
};

int8_t PROGMEM AmongUs[] = {
  NOTE_C5, NOTE_DS5, NOTE_F5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_C5, NOTE_AS4, NOTE_D5, NOTE_C5, NOTE_SILENCE, NOTE_SILENCE, NOTE_SILENCE,
  NOTE_C5, NOTE_DS5, NOTE_F, NOTE_FS5, NOTE_F, NOTE_DS5, NOTE_FS5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_SILENCE, SCORE_END
};

int8_t PROGMEM SailingSailing[] = {
  NOTE_G4,NOTE_C5,NOTE_C5,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_C5,NOTE_A4,NOTE_G4,NOTE_G4,
  NOTE_F4,NOTE_F4,NOTE_F4,NOTE_G4,NOTE_F4,NOTE_E4,NOTE_G4,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_A4,
  NOTE_B4,NOTE_C5,NOTE_D5,NOTE_G4,NOTE_C5,NOTE_C5,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_C5,
  NOTE_A4,NOTE_G4,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_B4,NOTE_B4,NOTE_C5,NOTE_C5,NOTE_D5,
  NOTE_D5,NOTE_E5,NOTE_C5,NOTE_D5,NOTE_B4,NOTE_C5,SCORE_END
};
int lastPlayed = 0;
int songPlaying = 0;
int noteCount = 0;

XT_DAC_Audio_Class DacAudio(speakerPin,0);  // Initialize the Audio with GPIO 25 (DAC) and timer group 0

XT_MusicScore_Class Music(TwinkleTwinkle,TEMPO_ALLEGRO,INSTRUMENT_PIANO);
XT_MusicScore_Class Music2(TwinkleTwinkle,TEMPO_ALLEGRO,INSTRUMENT_SAXOPHONE); 
XT_MusicScore_Class Music3(AmongUs,TEMPO_PRESTO,INSTRUMENT_ORGAN);
XT_MusicScore_Class Music4(SailingSailing,TEMPO_PRESTISSIMO,INSTRUMENT_ORGAN);

/**** Functions ****/

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
    if (esp_now_is_peer_exist(remote.peer_addr)) {
      Serial.println("LOL you did not sucessfully pair with remote.");
    } else {
      Serial.println("Remote has been paired with successfully");
    }
  }
}


void sendData() {
  if (DEBUG) {
    Serial.println();
    Serial.println("Sending data to remote:"); 
    Serial.println("Sliders --> ");
    Serial.print(remoteData.hue);
    Serial.println();
    Serial.println("Current note --> ");
    Serial.print(remoteData.noteCount);
    Serial.println();
  }
  esp_err_t sendStatus = esp_now_send(broadcastAddress,(uint8_t *) &remoteData, sizeof(soundncolorsremote));

  if (sendStatus == ESP_OK) {
    Serial.println("Sent data to remote successfully.");
  } else {
    Serial.println("Could not send data to remote.");
  }
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  
  // If received button and sensor data, update struct of button and sensor data
  // I know it sounds counter intutive but localData is data that is destined for the current device
  memcpy(&localData, incomingData, sizeof(soundncolors));
  if (DEBUG) {
    Serial.println();
    Serial.println("Data received!");
    Serial.println("Data from remote:"); 
    Serial.println("Sliders --> ");
    Serial.print(localData.hue);
    Serial.println();
    Serial.println("Current note --> ");
    Serial.print(localData.noteCount);
    Serial.println();
  }
  
  if (remoteData.noteCount != localData.noteCount) {
    // update noteCount to the new value, the checks for noteCount is done by sender
    remoteData.noteCount = localData.noteCount;
//    playSound(MusicSheet[localData.noteCount]);
    lastPlayed = millis();  // update timer
  }
}

float distance2colourval(float dist) {
  // Expects a distance in cm
  // Return value from 0.0 to 255.0 if distance is less than max slider distance, else returns 0
  return (dist < SLIDER_DIST) ? ((dist - 2) / (SLIDER_DIST - 2)) * 255 : 0;
}

void doMeasurement() {
  // Does measurement of distance and updates value
  unsigned long startWait;
  unsigned long timeout = 50;  // 23ms is 4m which is already out of range of sensor, we just gave it 27ms more than it needed.

  startEcho = 0;  // Reset start
  stopEcho = 0;  // Reset stop time

  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  startWait = millis();  // save start time for echo signal, for timeout detection

  while (startEcho == 0 or stopEcho == 0) {
    if (startWait + timeout < millis()) {
      // Timed out, distance of 0
      Serial.println("Timed out, sensor might be blocked, disconnected or broken.");
      startEcho = 0;
      stopEcho = 0;
      break;
    }
    noInterrupts();  // disable interrupts temporarily just in case
    distance = ((stopEcho - startEcho) / 2) * SOUND_MUL;
    interrupts();
  }
}


void setColours() { 
  oldHue = (localData.hue > oldHue) ? oldHue + COLOUR_STEP : oldHue - COLOUR_STEP;
  for (int strip = 0; strip < NUM_STRIPS; strip++) {
    for (int i = 0; i < ledCounts[strip]; i++) {
      leds[i].setHue(oldHue);
    }
  }
  FastLED.show();
}

void checkButton() {
  // Debounce protection
  if (digitalRead(buttonPin) && ((millis() - lastClicked) > DEBOUNCEINTERVAL)) {
    lastClicked = millis();  // last successful click
    // If next note is ready, then register the button press
//    if (millis() - lastPlayed > noteDelay[localData.noteCount]) {}  
  }
}

void checkButton() {
  // Debounce protection
  // Add check for release
  if (digitalRead(buttonPin) && ((millis() - lastClicked) > DEBOUNCEINTERVAL)) {
    lastClicked = millis();  // last successful click
    if (DEBUG) {
      Serial.println("Button pressed!");
      Serial.println();
    }
    currentMusic->sendNextNote();
  }
}

// Main logic and loops
void setup() {
  Serial.begin(115200);
  Serial.println();
  
  pinMode(trigPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(echoPin, INPUT); 
  pinMode(speakerPin, OUTPUT); // Need code in the timer and beep noises

  
  attachInterrupt(digitalPinToInterrupt(echoPin), ISR_ECHO, CHANGE);  // interrupt CPU 0 when slider detects change, might cause button to halt
  lastPing = millis();  // get current milisecond for delay later
  
  FastLED.addLeds<WS2812B, ledPin[0], RGB>(ledOne, ledCounts[0]);
  FastLED.addLeds<WS2812B, ledPin[1], RGB>(ledTwo, ledCounts[1]);

  DacAudio.Play(&Music);       
  currentMusic = &Music;
}


void loop() {
  // loop mainly does a check for time from last ping, and if it has been long enough it does a measurement, followed by updating the LED with the output
  // loop also checks for button presses, if any buttonPins is pressed, or pulled high, for now it just prints 

  if (millis() - lastPing >= PINGDELAY) {
    doMeasurement();
    float hue = distance2colourval(distance);
    Serial.println(String("Slider Sensor: ") + distance + " cm");  // print distance
    setColours(hue);
  }
  
  // if (millis() - lastNote >= NOTEDELAY[CURR_NOTE]) && (randomButton ==  {
    // playWav(CURR_NOTE, speakerPin);
}


// Interrupt Service Routines
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
