#include "hsvtorgb.h"
#include <WiFi.h>
#include <FastLED.h>
#include <SPI.h>
#include <SD.h>

// #define COMMON_ANODE  // just to invert the calculations for RGB
#define SLIDER_DIST 30
#define SLIDER_COUNT 2
#define BUTTON_COUNT 3
#define PINGDELAY 100
#define SOUND_MUL 0.0343
#define COLOUR_STEP 0.5

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

Left ultrasound sensor (viewed from back)
----------------------
Trigger - D13
Shared Echo - D14

//Middle ultrasound sensor (TBA)
//------------------------------
//Trigger - 
//Shared Echo - D14
//
//Right ultrasound sensor
//-----------------------
//Trigger - D12
//Shared Echo - D14

Buttons
-------
BT1 - D15
BT2 - D2
BT3 - D4
BT4 (TBA)- D35
BT5 (TBA) - D34

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

const int trigPins[SLIDER_COUNT] = {12, 13};  // From front, left and right, middle tba
const int echoPin = 14;  
const int buttonPins[BUTTON_COUNT] = {15, 2, 4};    // 35 and 34 doesn't have buttons yet
const int ledPins[3] = {27, 26, 25};  
const int speakerPin = 21;


// Stored states, buttons and hue value
float hue;
float distances[SLIDER_COUNT];  // distances in cm in order of sensor
unsigned long lastPing;
volatile unsigned long startEcho;
volatile unsigned long stopEcho;
float old_rgb[3] = {128.0, 128.0, 128.0};


void ISR_ECHO();

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  for (int i = 0; i < SLIDER_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
  }
  
  for (int i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT);
  }

  for (int i = 0; i < 3; i++) {
    // Each LED has its own channel lol, they ain't sharing
    ledcSetup(i, 1000, 10);  // channel, frequency, resolution
    ledcAttachPin(ledPins[i], i);
  }

  pinMode(echoPin, INPUT);  // shared echo pin

  pinMode(speakerPin, OUTPUT);
 
  attachInterrupt(digitalPinToInterrupt(14), ISR_ECHO, CHANGE);  // interrupt CPU 0 when slider detects change, might cause button to halt
  lastPing = millis();  // get current milisecond for delay later
}


float distance2colourval(float dist) {
  // Expects a distance in cm
  // Return normalized value from 0.0 to 1.0 if distance is less than max slider distance, else returns 0
  return (dist < SLIDER_DIST) ? ((dist - 2) / (SLIDER_DIST - 2)) : 0;
}

void doMeasurement() {
  // Expects a pointer to array of distances
  // Does measurement of distances, and returns an array of n distances
  unsigned long startWait;
  unsigned long timeout = 50;  // 23ms is 4m which is already out of range of sensor, we just gave it 27ms more than it needed.

  for (int s = 0; s < SLIDER_COUNT; s++) {
    startEcho = 0;  // Reset start
    stopEcho = 0;  // Reset stop time

    //    digitalWrite(trigPins[s], LOW);
    //    delayMicroseconds(5);  // not required as loop is slow enough
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

void setColours(float hue, float brightness) {
  // expects a float representing hue from 0.0 to 1.0, and a float representing brightness from 0.0 to 1.0
  // Will set the common cathode LED to the appropriate colour

  float rgb[3];
  hsv2rgb(hue, 1.0, brightness, rgb);  // Test if will have bug, I guess no cos modified in place

//  int red = (int)(rgb[0] * 255);
//  int green = (int)(rgb[1] * 255);
//  int blue = (int)(rgb[2] * 255);

  #ifdef COMMON_ANODE
    rgb[0] = 255 - rgb[0];
    rgb[1] = 255 - rgb[1];
    rgb[2] = 255 - rgb[2];
  #endif

  Serial.println(String(hue) + '-' + brightness + '-' + rgb[0] + '-' + rgb[1] + '-' + rgb[2]);
  
  // cross fade code
  old_rgb[0] = (rgb[0] > old_rgb[0]) ? old_rgb[0] + COLOUR_STEP : old_rgb[0] - COLOUR_STEP;
  old_rgb[1] = (rgb[1] > old_rgb[1]) ? old_rgb[1] + COLOUR_STEP : old_rgb[1] - COLOUR_STEP;
  old_rgb[2] = (rgb[2] > old_rgb[2]) ? old_rgb[2] + COLOUR_STEP : old_rgb[2] - COLOUR_STEP;


  ledcWrite(0, old_rgb[0]);
  ledcWrite(1, old_rgb[1]);
  ledcWrite(2, old_rgb[2]);
}

void loop() {
  // loop mainly does a check for time from last ping, and if it has been long enough it does a measurement, followed by updating the LED with the output
  // loop also checks for button presses, if any buttonPins is pressed, or pulled high, for now it just prints 

  if (millis() - lastPing >= PINGDELAY) {
    doMeasurement();
    float hue = distance2colourval(distances[0]);
    float brightness = distance2colourval(distances[1]);

    Serial.println(String("L Sensor: ") + distances[0] + " cm" + ' ' + String("R Sensor: ") + distances[1] + " cm");  // print distance
    setColours(hue, brightness);
  }
  
  // Button press code
//
//  int button_1 = digitalRead(buttonPins[0]);
//  int button_2 = digitalRead(buttonPins[1]);
//  int button_3 = digitalRead(buttonPins[2]);
//
//  if (button_1 || button_2 || button_3) {
//    Serial.println("Button was pressed!");
//  }
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
