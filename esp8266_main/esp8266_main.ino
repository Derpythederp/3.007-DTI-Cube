#include "hsvtorgb.h"
#include <ESP8266WiFi.h>
// #define COMMON_ANODE  // just to invert the calculations for RGB
#define SLIDER_DIST 25
#define SLIDER_COUNT 2
#define BUTTON_COUNT 3
#define PINGDELAY 50
#define SOUND_MUL 0.0343
#define COLOUR_STEP 1

// Pins
const int trigPins[SLIDER_COUNT] = {12, 13};  // array of trigger pins, current is D6 (GPIO 12) for left, D7 (GPIO 13) for right 
const int echoPin = 14;  // shared echo pin for all 3 sensors, current GPIO 14 or D5
const int buttonPins[BUTTON_COUNT] = {16, 5, 4};  // currently 3, D0, D1, D2
const int ledPins[3] = {15, 0, 2};  // currently Red is D8, Blue is D3, Green is D4

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
  Serial.println("BOOTED ESP8266... Serial connected!");
  
  for (int i = 0; i < SLIDER_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
  }
  
  for (int i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  pinMode(echoPin, INPUT);  // shared echo pin
 
  attachInterrupt(digitalPinToInterrupt(14), ISR_ECHO, CHANGE);  // interrupt CPU 0 when slider detects change, might cause button to halt
  lastPing = millis();  // get current milisecond for delay later
}


float distance2colourval(float dist) {
  // Expects a distance in cm
  // Return normalized hue from 0.0 to 1.0 if distance is less than max slider distance, else returns 0
  // Will not be used in final proto cos it has 3 sensors for direct RGB
  
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
  
  analogWrite(ledPins[0], old_rgb[0]);
  analogWrite(ledPins[1], old_rgb[1]);
  analogWrite(ledPins[2], old_rgb[2]);
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


ICACHE_RAM_ATTR void ISR_ECHO() {
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


// Tasklist:
// Smooth out the ultrasonic sensor readings
