// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

#ifdef ESP32
  #include <WiFi.h>
  #define BOARD_NAME ESP32
#else
  #include <ESP8266WiFi.h>
  #define BOARD_NAME ESP8266
#endif

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.print(String(BOARD_NAME) + " MAC Address:  ");
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}
