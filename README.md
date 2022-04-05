# 3.007-DTI-Cube
Tis but a cube. Two cubes to be exact.

![fidget cube](assets/cube.jpg)


## Prototype 1
Made from ESP8266 to test ultrasonic sensors and light. Uses 2 sensors corresponding to Hue and Brightness for HSB/HSV which is then converted to RGB to control light levels using PWM.

<p float="middle">
  <img src="assets/proto_1.png" width="48%" />
  <img src="assets/proto_1_irl.png" width="48%" />
</p>

## Prototype 2
Ported Prototype 1 code to Prototype 2 which is instead done on an ESP32 board. The biggest advantage of this is the fact that I get to play with 36 (30 usable) GPIO pins instead of the 17 on ESP8266 (in reality the usable pins are like 10 pins only). Multiprocessing and Bluetooth is an extra plus.


## Prototype 3
Prototype 2 continued. The plan changed, there is only 1 ultrasonic sensor (for 1 slider), 3 buttons. This should be the full prototype with the WS2812B LED strip (FastLED library), ESPNow networking, speakers and SD card reader.


![Prototype 3](/assets/DTI_Cube_Proto.png)

## Tasklists and features
- [x] Prototype 1 code ported to ESP32
- [ ] Button handler for 3 buttons
- [ ] Neopixel control 
- [ ] Connectivity with other ESP boards using ESPNow and packet structure
- [ ] Multiprocessing 
  - [ ] Core 0 pinned to poll for incoming network packets and process them by changing cube colour
  - [ ] Core 1 pinned to interrupt on slider and button changes, and process them and send them as outgoing packets 
- [ ] Speaker code
  - [ ] FAT32 read from VSPI (CS pin D5)
  - [ ] Send mono audio to speaker
- [ ] WS2812B code
  - [ ] Turn the LED idle state (when not much interaction) to white light
- [ ] Schematics for final prototype
