#define <SD.h>
#include <SPI.h>

bool checkSD(int SD_CS) {
  // Returns true if SD card is present
  SD.begin(SD_CS);  // SD object is a singleton for one SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failure");
    return false;
  }
  return true;
}
void readFile(char *file, int SD_CS) {
  // Expects a file object from SD.open()
  File file = SD.open(file);
  if (checkSD(file)){
    readFile(SD, file);
  }
  file.close();
}
void writeFile();
void existsFile();
void mkdirFile();
void rmdirFile();
void openDirFile();
void removeFile();
