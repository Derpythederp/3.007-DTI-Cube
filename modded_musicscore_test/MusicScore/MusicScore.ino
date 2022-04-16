#include "MusicDefinitions.h"
#include "DAC_Audio.h"
#define DEBOUNCEINTERVAL 175
#define NUM_SONGS 4


// Data for the melody. Note followed by optional change in playing length in 1/4 beats. See documentation for more details
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


int noteCount = 0;
const int buttonPin = 15;
int lastClicked = 0;
int lastPlayed = 0;
int songPlaying = 0;
bool lastButtonState = false;  // for LOW

XT_DAC_Audio_Class DacAudio(25,0);

XT_MusicScore_Class Music(TwinkleTwinkle,TEMPO_ALLEGRO,INSTRUMENT_PIANO);
XT_MusicScore_Class Music2(TwinkleTwinkle,TEMPO_ALLEGRO,INSTRUMENT_SAXOPHONE); 
XT_MusicScore_Class Music3(AmongUs,TEMPO_PRESTO,INSTRUMENT_ORGAN);
XT_MusicScore_Class Music4(SailingSailing,TEMPO_PRESTISSIMO,INSTRUMENT_ORGAN);

void checkButton() {
  // Debounce protection
  // Add check for release
  if (digitalRead(buttonPin) && ((millis() - lastClicked) > DEBOUNCEINTERVAL)) {
    lastClicked = millis();  // last successful click
    Serial.println("Button pressed");
    // If next note is ready, then register the button press
    Serial.println();
    currentMusic->sendNextNote();
  }
}

void setup() {
  Serial.begin(115200);
  DacAudio.Play(&Music4);       
  currentMusic = &Music4;
  pinMode(buttonPin, INPUT);
}

void checkNewSong() {
    if (Music.Playing==false && Music2.Playing==false && Music3.Playing==false && Music4.Playing==false) {
    Serial.println("Switching Songs!");
    songPlaying++;
    switch (songPlaying % NUM_SONGS) {
      case 0:
        Serial.println("Playing Twinkle Piano");
        DacAudio.Play(&Music, false);
        currentMusic = &Music;
        break;
      case 1:
        Serial.println("Playing Twinkle Organ");
        DacAudio.Play(&Music2, false);
        currentMusic = &Music2; 
        break;        
      case 2:
        Serial.println("Playing Amongus");
        DacAudio.Play(&Music3, false);
        currentMusic = &Music3;
        break;
      case 3:
        Serial.println("Playing Sailing Sailing");
        DacAudio.Play(&Music4, false);
        currentMusic = &Music4;
        break;    
    }
    noteCount = 0;  // reset note count for the next song
  }
}

void loop() {
  checkButton();
  DacAudio.FillBuffer();
  checkNewSong();
}
