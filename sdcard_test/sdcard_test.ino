// Test for SD Card reading and writing
// as well as audio using XT_DAC

#include "read_write.h"
#include <SD.h>
#include <SPI.h>
#define BUFFER_SIZE 1248 


const int SD_MOSI = 23;
const int SD_MISO = 19;
const int SD_CLK = 18;
const int SD_CS = 5;
const int dacPin = 25; 
uint8_t rawBuffer[BUFFER_SIZE];

// First 44 bytes of wav file are this header, subsequent data is in raw "chunks" defined by this header
struct WavHeader_Struct {
    //   RIFF Section    
    char RIFFSectionID[4];      // Letters "RIFF"
    uint32_t Size;              // Size of entire file less 8
    char RiffFormat[4];         // Letters "WAVE"
    
    //   Format Section    
    char FormatSectionID[4];    // letters "fmt"
    uint32_t FormatSize;        // Size of format section less 8
    uint16_t FormatID;          // 1=uncompressed PCM
    uint16_t NumChannels;       // 1=mono,2=stereo
    uint32_t SampleRate;        // 44100, 16000, 8000 etc.
    uint32_t ByteRate;          // =SampleRate * Channels * (BitsPerSample/8)
    uint16_t BlockAlign;        // =Channels * (BitsPerSample/8)
    uint16_t BitsPerSample;     // 8,16,24 or 32
  
    // Data Section
    char DataSectionID[4];      // The letters "data"
    uint32_t DataSize;          // Size of the data that follows
} WavHeader;


                                      

void SDCardInit() {
  {        
    pinMode(SD_CS, OUTPUT); 
    digitalWrite(SD_CS, HIGH); // SD card chips select, must use GPIO 5 (ESP32 SS)
    if(!SD.begin(SD_CS))
    {
        Serial.println("SD Card initialization failure!");
        return;                // end program
    }
}

bool ValidWavData(WavHeader_Struct* Wav) {
  // Check for invalid wav files 
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0) {    
    Serial.print("Invalid data - Not RIFF format");
    return false;        
  }
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0) {
    Serial.print("Invalid data - Not Wave file");
    return false;           
  }
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0) {
    Serial.print("Invalid data - No format section found");
    return false;       
  }
  if(memcmp(Wav->DataSectionID,"data",4)!=0) {
    Serial.print("Invalid data - data section not found");
    return false;      
  }
  if(Wav->FormatID!=1) {
    Serial.print("Invalid data - format Id must be 1");
    return false;                          
  }
  if(Wav->FormatSize!=16) {
    Serial.print("Invalid data - format section size must be 16.");
    return false;                          
  }
  if((Wav->NumChannels!=1)&(Wav->NumChannels!=2)) {
    Serial.print("Invalid data - only mono or stereo permitted.");
    return false;   
  }
  if(Wav->SampleRate>48000) {
    Serial.print("Invalid data - Sample rate cannot be greater than 48000");
    return false;                       
  }
  if((Wav->BitsPerSample!=8)& (Wav->BitsPerSample!=16)) {
    Serial.print("Invalid data - Only 8 or 16 bits per sample permitted.");
    return false;                        
  }
  return true;
}


void DumpWAVHeader(WavHeader_Struct* Wav) {
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0) {
    Serial.print("Not a RIFF format file - ");    
    PrintData(Wav->RIFFSectionID,4);
    return;
  } 
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0) {
    Serial.print("Not a WAVE file - ");  
    PrintData(Wav->RiffFormat,4);  
    return;
  }  
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0) {
    Serial.print("fmt ID not present - ");
    PrintData(Wav->FormatSectionID,3);      
    return;
  } 
  if(memcmp(Wav->DataSectionID,"data",4)!=0) {
    Serial.print("data ID not present - "); 
    PrintData(Wav->DataSectionID,4);
    return;
  }  
  // All looks good, dump the data
  Serial.print("Total size :");Serial.println(Wav->Size);
  Serial.print("Format section size :");Serial.println(Wav->FormatSize);
  Serial.print("Wave format :");Serial.println(Wav->FormatID);
  Serial.print("Channels :");Serial.println(Wav->NumChannels);
  Serial.print("Sample Rate :");Serial.println(Wav->SampleRate);
  Serial.print("Byte Rate :");Serial.println(Wav->ByteRate);
  Serial.print("Block Align :");Serial.println(Wav->BlockAlign);
  Serial.print("Bits Per Sample :");Serial.println(Wav->BitsPerSample);
  Serial.print("Data Size :");Serial.println(Wav->DataSize);
}


void PrintData(const char* Data,uint8_t NumBytes) {  // Helper function for printing chars
    for(uint8_t i=0;i<NumBytes;i++)
      Serial.print(Data[i]); 
      Serial.println();  
}

void setup() {
  Serial.begin(115200);
  SDCardInit();
  diskUsage();
  // set up GPIO 25 as DAC Pin

  File WavFile = SD.open("01a_C.wav");
  if (!WavFile) {
    Serial.println("Could not open wav file");
  } else {
    WavFile.read((byte *) &WavHeader, 44);
    DumpWAVHeader
  }
  
  
}

void readRaw(fs::FS &fs, const char * path, uint8_t* rawBuffer){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }
    
    while(file.available()){
        memcpy(rawBuffer, (uint8_t *)file.read(), sizeof(BUFFER_SIZE));
    }
    file.close();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input == "du") {
      diskUsage();
    } else if (input == "ls") {
      listDir(SD, "/", 1);
    } else if (input == "cat") {
      readRaw(SD, "/05a_G.wav", rawBuffer);
    }
    // Doesn't seem like FAT32 has inodes so it is implemeted as a copy followed by a delete
  }
}
