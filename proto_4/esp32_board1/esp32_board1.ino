#include <WiFi.h>
#include <FastLED.h>
#include <SPI.h>
#include <SD.h>
#include <esp_now.h>
#include "DAC_Audio.h"
#include "settings.h"


// Pins

/****
THINGS TO CHANGE BETWEEN MODULES:
- LED NUMBER (ledCounts)
- MAC ADDRESS
- MusicOffset

If this is the first board to play, (Board2)
noSoundUpdate = true
MusicOffset = 0
localData=-1

****/

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
const int ledPin1 = 26;
const int ledPin2 = 27;
const int buttonPin = 15;


// Networking Definitions
const uint8_t broadcastAddress[6] = {0x58, 0xBF, 0x25, 0x14, 0x50, 0xA0};  // ESP32 Soldered to module
//const uint8_t broadcastAddress[6] = {0x9C, 0x9C, 0x1F, 0xDD, 0x52, 0xF0};  // ESP32 with full electronics
void OnDataRecv();
esp_now_peer_info_t remote; 
int ESPNow_Fail_Count = 0;


// LED definitions 
float oldHue = 128.0;  // for cross fade
float lastColorUpdate;
const int ledCounts[] = {64, 64};  // In order, GPIO 26 and 27
CRGB ledStrip1[64];  
CRGB ledStrip2[64];

// Ultrasonic sensor Definitions
unsigned long lastPing;
volatile unsigned long startEcho;  // volatile is needed for shared variables between ISR and loop to prevent compiler from deleting it via optimization
volatile unsigned long stopEcho;
float distance;

// Button definitions
unsigned int lastClicked = 0;
int buttonState;
int lastButtonState = LOW;  // used for edge detection of button

// Interrupt Service Routine
void ISR_ECHO();
void ISR_BUTTON();


// Note Status and network structs
unsigned int lastPacketSent;
bool noSoundUpdate = false;  // One of the module will have this set to true, else there will be glitches
struct soundncolors {
  float hue;
  int noteCount;
  bool buttonPressed;  // if button has been pressed, to check for idle
};

struct soundncolors incomingBuffer; // buffer to store incoming data for deserialization
struct soundncolors localData = {128.0, 0, false};  // To store old state for comparison
struct soundncolors remoteData = {128.0, 0, false};  // buffer to store outgoing data
const int MusicOffset = 1;

// Idle Status counter
float lastAction;  // in seconds

// Music definitions 
XT_MusicScore_Class *currentMusic;

int8_t PROGMEM WayBackThen[] = {
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_DS5,NOTE_B4,NOTE_B4,NOTE_A4,NOTE_G4,NOTE_A4,NOTE_B4,
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_B4,NOTE_A4,NOTE_G4,NOTE_A4,NOTE_G4,NOTE_E4,NOTE_E4
};  // 26 Notes

int8_t PROGMEM PinkSoldiers[] = {
  NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_G4, 
  NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_GS4, NOTE_CS4, NOTE_GS4, 
  NOTE_CS4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_F4, 
  NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_G4, NOTE_G4
};  // 34 Notes

int8_t PROGMEM TwinkleTwinkle[] = {
  NOTE_C5,NOTE_C5,NOTE_G5,NOTE_G5,NOTE_A5,NOTE_A5,NOTE_G5,
  NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,NOTE_D5,NOTE_C5,
  NOTE_G5,NOTE_G5,NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,
  NOTE_G5,NOTE_G5,NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,
  NOTE_C5,NOTE_C5,NOTE_G5,NOTE_G5,NOTE_A5,NOTE_A5,NOTE_G5,
  NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,NOTE_D5,NOTE_C5
  };  // 48 Notes

int8_t PROGMEM AmongUs[] = {
  NOTE_C5, NOTE_DS5, NOTE_F5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_C5, NOTE_AS4, NOTE_D5, NOTE_C5,
  NOTE_C5, NOTE_DS5, NOTE_F, NOTE_FS5, NOTE_F, NOTE_DS5, NOTE_FS5, NOTE_FS5, NOTE_F5, NOTE_DS5, 
  NOTE_FS5, NOTE_F5, NOTE_DS5
};  // 23 Notes

int8_t PROGMEM SailingSailing[] = {
  NOTE_G4,NOTE_C5,NOTE_C5,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_C5,NOTE_A4,NOTE_G4,NOTE_G4,
  NOTE_F4,NOTE_F4,NOTE_F4,NOTE_G4,NOTE_F4,NOTE_E4,NOTE_G4,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_A4,
  NOTE_B4,NOTE_C5,NOTE_D5,NOTE_G4,NOTE_C5,NOTE_C5,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_C5,
  NOTE_A4,NOTE_G4,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_B4,NOTE_B4,NOTE_C5,NOTE_C5,NOTE_D5,
  NOTE_D5,NOTE_E5,NOTE_C5,NOTE_D5,NOTE_B4,NOTE_C5
}; // 50 Notes

int8_t PROGMEM MegaSong[] = {
  NOTE_C5,NOTE_C5,NOTE_G5,NOTE_G5,NOTE_A5,NOTE_A5,NOTE_G5,
  NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,NOTE_D5,NOTE_C5,
  NOTE_G5,NOTE_G5,NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,
  NOTE_G5,NOTE_G5,NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,
  NOTE_C5,NOTE_C5,NOTE_G5,NOTE_G5,NOTE_A5,NOTE_A5,NOTE_G5,
  NOTE_F5,NOTE_F5,NOTE_E5,NOTE_E5,NOTE_D5,NOTE_D5,NOTE_C5,
  NOTE_G4,NOTE_C5,NOTE_C5,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_C5,NOTE_A4,NOTE_G4,NOTE_G4,
  NOTE_F4,NOTE_F4,NOTE_F4,NOTE_G4,NOTE_F4,NOTE_E4,NOTE_G4,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_A4,
  NOTE_B4,NOTE_C5,NOTE_D5,NOTE_G4,NOTE_C5,NOTE_C5,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_C5,
  NOTE_A4,NOTE_G4,NOTE_G4,NOTE_A4,NOTE_A4,NOTE_A4,NOTE_B4,NOTE_B4,NOTE_C5,NOTE_C5,NOTE_D5,
  NOTE_D5,NOTE_E5,NOTE_C5,NOTE_D5,NOTE_B4,NOTE_C5,
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_DS5,NOTE_B4,NOTE_B4,NOTE_A4,NOTE_G4,NOTE_A4,NOTE_B4,
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_B4,NOTE_B4,NOTE_B4,
  NOTE_B4,NOTE_A4,NOTE_G4,NOTE_A4,NOTE_G4,NOTE_E4,NOTE_E4,
  NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_G4, 
  NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_GS4, NOTE_CS4, NOTE_GS4, 
  NOTE_CS4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_F4, NOTE_GS4, NOTE_F4, 
  NOTE_GS4, NOTE_G4, NOTE_GS4, NOTE_G4, NOTE_G4
  NOTE_C5, NOTE_DS5, NOTE_F5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_C5, NOTE_AS4, NOTE_D5, NOTE_C5,
  NOTE_C5, NOTE_DS5, NOTE_F, NOTE_FS5, NOTE_F, NOTE_DS5, NOTE_FS5, NOTE_FS5, NOTE_F5, NOTE_DS5, 
  NOTE_FS5, NOTE_F5, NOTE_DS5,
}; // 181 Notes

int lastPlayed = 0;
int songPlaying = 0;
int noteCount = 0;

XT_DAC_Audio_Class DacAudio(speakerPin,0);  // Initialize the Audio with GPIO 25 (DAC) and timer group 0

XT_MusicScore_Class Music(MegaSong,TEMPO_ALLEGRO,INSTRUMENT_PIANO, 181, MusicOffset, NUMMODULES);
XT_MusicScore_Class Music2(TwinkleTwinkle,TEMPO_ALLEGRO,INSTRUMENT_ORGAN, 48, MusicOffset, NUMMODULES); 
XT_MusicScore_Class Music3(AmongUs,TEMPO_PRESTO,INSTRUMENT_ORGAN, 23, MusicOffset, NUMMODULES);
XT_MusicScore_Class Music4(SailingSailing,TEMPO_PRESTISSIMO,INSTRUMENT_ORGAN, 50, MusicOffset, NUMMODULES);
XT_MusicScore_Class Music5(WayBackThen,TEMPO_ALLEGRO,INSTRUMENT_ORGAN, 26, MusicOffset, NUMMODULES);
XT_MusicScore_Class Music6(PinkSoldiers,TEMPO_ALLEGRO,INSTRUMENT_ORGAN, 34, MusicOffset, NUMMODULES);

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
  esp_err_t sendStatus = esp_now_send(broadcastAddress,(uint8_t *) &remoteData, sizeof(soundncolors));

  if (DEBUG) {
    if (sendStatus == ESP_OK) {
      Serial.println("Sent data to remote successfully.");
    } else {
      Serial.println("Could not send data to remote.");
    }
  }
    
  remoteData.buttonPressed=false;  // Just as a fail safe, this should always be set to false unless pressed
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  
  // If received button and sensor data, update struct of button and sensor data
  // I know it sounds counter intutive but localData is data that is destined for the current device

  // The check for idle state should be, if old localData /= incomingData defined as:
  // 1) incomingData.Hue exceeds a threshold of IDLETHRESHOLDDISTANCE 
  // if (abs(incomingData.Hue - localData.hue) > distance2colourval(IDLETHRESHOLDDISTANCE))
  // 2) ((localDAta.noteCount + 2) == incomingBuffer.noteCount then noteCount is correct and that means the other module has been updated.
  // 
  // 
  // Pseudo code:
  // 1) Store in incomingBuffer, the incoming network data  (done)
  // 2) Check if incomingBuffer has changes to the state of the current staircade module, i.e if it fails idle checks, then update lastAction (see above for criteria)
  // 3) If ((localData.noteCount + 2) == incomingBuffer.noteCount) then the correct noteCount has been sent in --> currentMusic->sendNextNote(), noSoundUpdate flag set to false so that more notes can be checked by button
  // 4) Just update the localData to be equal to network data after the idle check and the noteCount check
  // Updates localData only.
  
  memcpy(&incomingBuffer, incomingData, sizeof(soundncolors));
  
  if (DEBUG) {
    Serial.println();
    Serial.println("Data received!");
    Serial.println("Data from remote:"); 
    Serial.println("Sliders --> ");
    Serial.print(incomingBuffer.hue);
    Serial.println();
    Serial.println("Current note --> ");
    Serial.print(incomingBuffer.noteCount);
    Serial.println();
  }

  if ((abs(incomingBuffer.hue - localData.hue) > distance2colourval(IDLETHRESHOLDDISTANCE))) {
    lastAction = seconds();
  }

  if ((localData.noteCount + 2) == incomingBuffer.noteCount) {
//    noInterrupts();
    currentMusic->sendNextNote();
    noSoundUpdate = false;
    lastAction = seconds();
//    interrupts();
  }

  // Hacky workaround to sync idle
  if (incomingBuffer.buttonPressed) {
    // if not idle, then lastAction will be now
    lastAction = seconds();
  }
  
  memcpy(&localData, incomingData, sizeof(soundncolors));
}


float distance2colourval(float dist) {
  // Expects a distance in cm
  // Return value from 0.0 to 255.0 if distance is less than max slider distance, else returns 0
  return (dist < SLIDER_DIST) ? ((dist - 2) / (SLIDER_DIST - 2)) * 255 : 0;
}

void doMeasurement() {
  // Does measurement of distance and updates value of remoteData.hue after converting the distance to colour value.
  unsigned long startWait;
  unsigned long timeout = 50;  // 23ms is 4m which is already out of range of sensor, we just gave it 27ms more than it needed.

  startEcho = 0;  // Reset start
  stopEcho = 0;  // Reset stop time

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
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
    float distanceNew = ((stopEcho - startEcho) / 2) * SOUND_MUL;
    if (distanceNew < 400.0 && distanceNew > 2) {
      // Eliminate trash results and even out distances 
      if (abs(distance - distanceNew) > IDLETHRESHOLDDISTANCE) {
        lastAction = seconds();
      }
      distance = distanceNew;
    }
    remoteData.hue = distance2colourval(distance);  
    interrupts();
  }
}

void breathingColours() {
   // If called in loop will cause LED brightness to breathe
   float breath = (exp(sin(millis()/1000.0*PI)) - 0.36787944)*108.0;
   breath = map(breath, 0, 255, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
   FastLED.setBrightness(breath);
}

void setColours() { 
  // setColours() will update the colours based on localData.hue
  FastLED.setBrightness(DEFAULT_BRIGHTNESS);
  
  oldHue = (localData.hue > oldHue) ? oldHue + COLOUR_STEP : oldHue - COLOUR_STEP;
  for (int i = 0; i < ledCounts[0]; i++) {
    ledStrip1[i].setHue(oldHue);
  }
  for (int j = 0; j < ledCounts[1]; j++) {
    ledStrip2[j].setHue(oldHue);
  }
  FastLED.show();
}

void setWhite() {
  for (int i = 0; i < ledCounts[0]; i++) {
    ledStrip1[i].setHSV(0, 0, 255);
  }
  for (int j = 0; j < ledCounts[1]; j++) {
    ledStrip2[j].setHSV(0, 0, 255);
  }
  FastLED.show();
}

void checkButton() {
  // Only detects if button is depressed once, so that you can't hold down the button
  // checkButton() will increment the remoteData.noteCount = localData.noteCount + 1, as well as set flag for noSoundUpdate to true so that remoteData.noteCount cannot be updated.
  
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastClicked = millis();
  }
  
  if ((millis() - lastClicked) > DEBOUNCEINTERVAL) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        if (DEBUG) {
          Serial.println("Button pressed!");
        }
        remoteData.buttonPressed=true;
        if (!noSoundUpdate) {
          if (DEBUG) {
            Serial.println("Updating remoteData note count and locking any further updates");
          }
          remoteData.noteCount = localData.noteCount + 1;
          noSoundUpdate = true;
        }
      lastAction = seconds();
      } else {
        remoteData.buttonPressed=false;
      }
    }
  }
  lastButtonState = reading;
}

void checkNewSong() {
    if (Music.Playing==false && Music2.Playing==false && Music3.Playing==false && Music4.Playing==false && Music5.Playing==false && Music6.Playing==false) {
    Serial.println("Switching Songs!");
    songPlaying++;
    switch (songPlaying % NUM_SONGS) {
      case 0:
        Serial.println("Playing Mega Song");
        Music.ResetFromOffset(MusicOffset);
        DacAudio.Play(&Music, false);
        currentMusic = &Music;
        break;
      case 1:
        Music2.ResetFromOffset(MusicOffset);
        Serial.println("Playing Twinkle Organ");
        DacAudio.Play(&Music2, false);
        currentMusic = &Music2; 
        break;        
      case 2:
        Music3.ResetFromOffset(MusicOffset);
        Serial.println("Playing Amongus");
        DacAudio.Play(&Music3, false);
        currentMusic = &Music3;
        break;
      case 3:
        Music4.ResetFromOffset(MusicOffset);
        Serial.println("Playing Sailing Sailing");
        DacAudio.Play(&Music4, false);
        currentMusic = &Music4;
        break;       
      case 4:
        Music5.ResetFromOffset(MusicOffset);
        Serial.println("Playing Way Back Then");
        DacAudio.Play(&Music5, false);
        currentMusic = &Music5;
        break;       
      case 5:
        Music5.ResetFromOffset(MusicOffset);
        Serial.println("Playing Pink Soldiers");
        DacAudio.Play(&Music6, false);
        currentMusic = &Music6;
        break;   
    }
    noteCount = 0;  // reset note count for the next song
  }
}

// Main logic and loops
void setup() {
  Serial.begin(115200);
  Serial.println();
  
  // Init WiFi and ESPNow
  WiFi.mode(WIFI_STA);
  delay(1000);
  initESPNow();
  esp_now_register_recv_cb(OnDataRecv);
  initBroadcastPeer();
  
  pinMode(trigPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(echoPin, INPUT); 
  pinMode(speakerPin, OUTPUT); // Need code in the timer and beep noises

  // Init ulrasonic sensors
  attachInterrupt(digitalPinToInterrupt(echoPin), ISR_ECHO, CHANGE);  // interrupt CPU 0 when slider detects change, might cause button to halt
  lastPing = millis();  // get current milisecond for delay later


  // Init LED strips. Has to be a const so I can't really use for loops
  // the value of 'ledPins' is not usable in a constant expression, why?
  FastLED.addLeds<WS2812B, ledPin1, RGB>(ledStrip1, ledCounts[0]);
  FastLED.addLeds<WS2812B, ledPin2, RGB>(ledStrip2, ledCounts[1]);

  FastLED.setBrightness(DEFAULT_BRIGHTNESS);

  // Init audio files locally
  noInterrupts();
  DacAudio.Play(&Music);       
  currentMusic = &Music;
  lastAction = seconds();
  lastPacketSent = millis();
  interrupts();
}


void loop() {
  // loop mainly does a check for time from last ping, and if it has been long enough it does a measurement, followed by updating the LED with the output
  // loop also checks for button presses, if any buttonPins is pressed, or pulled high, for now it just prints 
  noInterrupts();
  checkButton();
  DacAudio.FillBuffer();
  checkNewSong();
  interrupts();
  if (millis() - lastPing >= PINGDELAY) {
    doMeasurement();
    
    if (DEBUG)
      Serial.println(String("Slider Sensor: ") + distance + " cm");  // print distance
      
    lastPing = millis();
  }

  if (millis() - lastColorUpdate >= COLORDELAY) {
    noInterrupts();
    if (seconds() - lastAction >= IDLETIME) {
      breathingColours();
      setWhite();
    } else {
      setColours();
    }
    lastColorUpdate = millis();
    interrupts();
  }

 // If last data sent was > PACKETDELAY, senddat
 if (millis() - lastPacketSent > PACKETDELAY) {
  sendData();
  lastPacketSent = millis();
 }
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
