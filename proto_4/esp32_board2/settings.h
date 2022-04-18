#define SLIDER_DIST 15

#define PINGDELAY 200
#define SOUND_MUL 0.0343
#define COLOUR_STEP 0.8
#define COLORDELAY 8
#define MIN_BRIGHTNESS 8  // Lower bound for idle state
#define MAX_BRIGHTNESS 255  // Upper bound for idle state
#define DEFAULT_BRIGHTNESS 128         

#define REMOTE_BUTTON_COUNT 1
#define MAX_ESPNOW_FAILURES 10
#define CHANNEL 1  // Both sender and reciever has to be on the same channel
#define COLOUR_STEP 0.1
#define NUM_SONGS 4
#define DEBOUNCEINTERVAL 100
#define DEBUG false
#define DEBUG_PAIR true  // if true, then esp_now_is_peer_exist will be called as an additional check

#define seconds() (millis()/1000)
#define IDLETIME 30  // 10 seconds
#define IDLETHRESHOLDDISTANCE 3  // in cm, to give a bit of allowance for sensor inaccuracy and jumping
#define PACKETDELAY 100
#define NUMMODULES 2
