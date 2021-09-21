// This sketch plays PSG files via a AY-3-8910 which
// is a 3-voice programmable sound generator (PSG)

// optimised code to save space, will now fit on a 'ATmega168' ... just 15 bytes free for stack/local variables

#include <SPI.h>
#include "SdFat.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include "fudgefont.h"  // Based on the Adafruit5x7 font, with '!' to '(' changed to work as a VU BAR (8 chars)

// **********************************************
//#define BUFFER_SIZE (64)  //  *** ATmega168 (1K) ***  Smaller buffer/non optimied - limiting size as we need all the memory we can get.
#define BUFFER_SIZE (256)   //  *** ATmega328 (2K) ***  Allows optimised byte counters which can safely wrap back to zero.
byte playBuf[BUFFER_SIZE];  // 256 MAX 
// **********************************************
#if (BUFFER_SIZE==256)
#define ADVANCE_PLAY_BUFFER  playPos++; 
#define ADVANCE_LOAD_BUFFER  loadPos++; 
#else 
#define ADVANCE_PLAY_BUFFER  playPos++; if (playPos>=BUFFER_SIZE) playPos=0; 
#define ADVANCE_LOAD_BUFFER  loadPos++; if (loadPos>=BUFFER_SIZE) loadPos=0; 
#endif

// Byte commands - Incoming data from file
#define  END_OF_INTERRUPT_0xFF            (0xff) 
#define  END_OF_INTERRUPT_MULTIPLE_0xFE   (0xfe) 
#define  END_OF_MUSIC_0xFD                (0xfd) 

// Screen Ypos
#define DISPLAY_ROW_FILENAME        (0)
#define DISPLAY_ROW_FILE_COUNTER    (1)
#define DISPLAY_ROW_BYTES_LEFT      (1)
#define DISPLAY_ROW_VU_METER_TOP    (2)
#define DISPLAY_ROW_VU_METER_BOTTOM (3)

enum AYMode { INACTIVE, WRITE, LATCH };

enum {
  pinReset = 2, // AY38910 reset
  pinClock,     // AY38910 clock
  pinSHCP,      // 74HC595 clock
  pinSTCP,      // 74HC595 latch
  pinDS,        // 74HC595 data
  pinSkip,      // skip to next random song
  pinBC1,       // AY38910 BC1
  pinBDIR,      // AY38910 BDIR
  pinCS         // SD card select (CS)
};

// 128x32 i2c OLED - 0.96".  
// SDA=A4 ,  SCL=A5
#define I2C_ADDRESS (0x3C)   // used by oled
SSD1306AsciiAvrI2c oled;

SdFat  m_sd;  // takes 595 bytes
SdFile m_fp;  // takes 36 bytes
SdFile m_dir; // takes 36 bytes

enum  { FLAG_PLAY, FLAG_NEXT, FLAG_BUTTON_DOWN, FLAG_BUTTON_REPEAT, FLAG_REFRESH_DISPLAY };

byte playFlag;
 
#define VU_METER_INTERNAL_SCALE (2)  // speed scale 2^n values

int filesCount = 0;
int fileNum = 0;  // file indexs start from zero
int skipCnt = 0;   // Dont play new sequences via interrupt when this is positive
uint32_t fileSize;

byte loadPos;  // WARNING : this counter will wrap back to zero
byte playPos;  // WARNING : this counter will wrap back to zero
byte volumeA=0;
byte volumeB=0;
byte volumeC=0;

void setup() {
 // Serial.begin(9600);   // this library eats 177 bytes, remove from release
  //Serial.println(sizeof(SdFat));

  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont( fudged_Adafruit5x7 ); // original Adafruit5x7 font with tweeks at start for VU meter
  oled.clear();
  
  if (m_sd.begin(pinCS, SD_SCK_MHZ(50))) {
    countPlayableFiles();
  }

  setupPins();
  setupTimer();
  init2MhzClock();
  resetAY();
  bitSet(playFlag,FLAG_NEXT);
  bitSet(playFlag,FLAG_REFRESH_DISPLAY);
}

void setupPins() {
  pinMode(pinBC1, OUTPUT);
  pinMode(pinBDIR, OUTPUT);
  pinMode(pinReset, OUTPUT);
  pinMode(pinClock, OUTPUT);
  pinMode(pinSHCP, OUTPUT);
  pinMode(pinSTCP, OUTPUT);
  pinMode(pinDS, OUTPUT);
  pinMode(pinSkip, INPUT_PULLUP);
}

void setupTimer() {
  cli();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS12);
  TIMSK1 = _BV(OCIE1A);
  TCNT1 = 0;
  OCR1A = 1250;
  sei();
}


//*** Compiler results for a ATmega168 ("cut down" pro mini with JUST 1k, Atmega238P has 2k) ***
//Sketch uses 12602 bytes (87%) of program storage space. Maximum is 14336 bytes.
//Global variables use 1009 bytes (98%) of dynamic memory, leaving 15 bytes for local variables. Maximum is 1024 bytes.
//
// ATmega168 ONLY HAS 15 bytes FREE !    TOO LITTLE FOR CODE TO RUN... DROPPED CACHE SIZE DOWN TO 64bytes ON THE 1k version

void countPlayableFiles() {
 // SdFile* file = (SdFile*)playBuf;  // buffers not currently in use, so putting it to good use :)
  if (m_dir.open("/", O_READ)) {  // is never closed, as we are using rewind later.
    while (m_fp.openNext(&m_dir, O_READ)) {
      if (m_fp.isFile()) {
         // really need to save as mutch dynamic memory as possible.  Here we are checking for the "PSG" file header byte by byte.
         if (m_fp.available() && m_fp.read() == 'P' && m_fp.available() &&  m_fp.read()=='S' && m_fp.available() && m_fp.read()=='G') {
          filesCount++;
         }
      }
      m_fp.close();
    }
     //   m_dir.close();  // this is ok ... Leave it open, dont close. 
  }
}

void loop() {
  
  if (bitRead(playFlag,FLAG_NEXT)) {
    oled.clear();  // called early as selectFile uses oled to display filename
    selectFile(fileNum);

    oled.setCursor(0, DISPLAY_ROW_FILE_COUNTER);
   //oled.print(F("Playing:"));
    oled.print(fileNum+1); 
    oled.print(F("/"));
    oled.print(filesCount);

    fileNum++;
    if (fileNum >= filesCount)
       fileNum = 0;

    bitClear(playFlag,FLAG_NEXT);
    bitSet(playFlag,FLAG_PLAY);
  }

  loadNextByte();

  if (bitRead(playFlag,FLAG_REFRESH_DISPLAY)) {  
    
    oled.setCursor((128/2)-6-6-6,DISPLAY_ROW_VU_METER_TOP);
    displayVolumeTop(volumeA/VU_METER_INTERNAL_SCALE);  // dividing by 2, scaled maths used (*2 scale used setting VU meter)
    displayVolumeTop(volumeB/VU_METER_INTERNAL_SCALE);
    displayVolumeTop(volumeC/VU_METER_INTERNAL_SCALE);

    oled.setCursor((128/2)-6-6-6,DISPLAY_ROW_VU_METER_BOTTOM);
    displayVolumeBottom(volumeA/VU_METER_INTERNAL_SCALE);
    displayVolumeBottom(volumeB/VU_METER_INTERNAL_SCALE);
    displayVolumeBottom(volumeC/VU_METER_INTERNAL_SCALE);
    

    oled.setCursor(128-32, DISPLAY_ROW_BYTES_LEFT);
    oled.print(fileSize/1024); 
    oled.print("K "); 
    
    volumeA--;  // drift all the VU meters down over time (values are inernaly scaled)
    volumeB--;
    volumeC--;  

    bitClear(playFlag,FLAG_REFRESH_DISPLAY);
  }

}

void selectFile(int fileIndex) {
  m_fp.close();  // FUDGE
  m_dir.rewind();  // is always left open, so we just rewind 
  int k = 0;
  while (m_fp.openNext(&m_dir, O_READ)) {
    if (m_fp.isFile()) {
      if (m_fp.available() && m_fp.read() == 'P' && m_fp.available() &&  m_fp.read()=='S' && m_fp.available() && m_fp.read()=='G') {
        if (k == fileIndex) { 
          fileSize = m_fp.fileSize();
          oled.setCursor(0,DISPLAY_ROW_FILENAME);
          // We can safely use the play buffer here to save dynamic memory.
          m_fp.getName((char*)playBuf,22);  // 21 characters (+1 for null).  128pixel display using 'Adafruit5x7' font i.e 128/(5+gap of 1) 
          oled.print((char*)playBuf);
          advancePastHeader();
          break;
        }
      }
      k++;
    }
    m_fp.close(); // break will skip this... thats ok and as designed
  }
 // memset(playBuf,0xfd,BUFFER_SIZE);
}

void loadNextByte() {

  if (loadPos == playPos - 1)  // cache is behind, wait for it to refill
    return ;
  if (loadPos == (BUFFER_SIZE - 1) && playPos == 0)  // cache is behind
     return ;

  if (m_fp.available()) {
    playBuf[loadPos] =  m_fp.read();
    fileSize--; 
  }
  else {
     playBuf[loadPos] = 0xFD;
  }
  ADVANCE_LOAD_BUFFER
}

void advancePastHeader() {
  loadPos = playPos = 0;
  while (m_fp.available()) {
    byte b = m_fp.read();
    if (b == 0xFF) break; // body data stats with a FF, we use this to skip over the header
  }
}

void resetAY() {
  setAYMode(INACTIVE);
  digitalWrite(pinReset, LOW);
  delay(50);
  digitalWrite(pinReset, HIGH);
  delay(50);
}

// The AY38910 clock pin needs to be driven at a requency of 1.75 MHz.
// We can get an interrupt to trigger at 1.778 MHz ( 1.5873% difference, close enough)
#define PERIOD  (9)             // 9 CPU cycles ~ 1.778 MHz
void init2MhzClock() {
  TCCR2B = 0;                       // stop timer
  TCNT2 = 0;                        // reset timer
  TCCR2A =   _BV(COM2B1)            // non-inverting PWM on OC2B
           | _BV(WGM20)             // fast PWM mode, TOP = OCR2A
           | _BV(WGM21);            // 
  TCCR2B = _BV(WGM22) | _BV(CS20);  // 
  OCR2A = PERIOD - 1;
  OCR2B = (PERIOD / 2) - 1;
}

void setAYMode(AYMode mode) {
  switch (mode) {
    case INACTIVE: PORTB &= B11111100; break;
    case WRITE: PORTB |= B00000010; break;
    case LATCH: PORTB |= B00000011; break;
  }
}

void writeAY( byte port , byte data ) {

  if (port < 16) {
   
    switch (port) {
      case 8: volumeA = data*VU_METER_INTERNAL_SCALE; break;   // *2 for scaled maths (VU mether speed)
      case 9: volumeB = data*VU_METER_INTERNAL_SCALE; break;
      case 10: volumeC = data*VU_METER_INTERNAL_SCALE; break;         
    }      

    setAYMode(INACTIVE);
    digitalWrite(pinSTCP, LOW);
    shiftOut(pinDS, pinSHCP, MSBFIRST, port);
    digitalWrite(pinSTCP, HIGH);
    setAYMode(LATCH);
    
    setAYMode(INACTIVE);
    digitalWrite(pinSTCP, LOW);
    shiftOut(pinDS, pinSHCP, MSBFIRST, data);
    digitalWrite(pinSTCP, HIGH);
    setAYMode(WRITE);
    setAYMode(INACTIVE);
  }
}

// PSG music format (body)
// [0xff]              : end of interrupt (eoi) - waits for 20 ms
// [0xfe],[byte]       : multiple eoi - byte for how many times to wait for 80 ms.
// [0xfd]              : end of music
// [0x00..0x0f],[byte] : psg register, byte data for this register

void playNotes() {
  while (isCacheReady()) {
    byte b = playBuf[playPos];
    ADVANCE_PLAY_BUFFER
    switch(b) {
       case END_OF_INTERRUPT_0xFF: return;
       case END_OF_INTERRUPT_MULTIPLE_0xFE:
          if (isCacheReady()) {
            b = playBuf[playPos];
            ADVANCE_PLAY_BUFFER
            if ((b==0xff) && (fileSize/1024==0) )  {
                // Some tunes have tones of repeated ending sequences of "0xfe 0xff", "0xff" is a long pause.
                // example : "NewZealandStoryThe" has a very long pause at the end I'm guessing by design to handerover to the ingame tune.
                skipCnt=0;  // ignore stupidly long pauses, but only when nearing the end of the tune
            }
            else {
               if (!bitRead(playFlag,FLAG_BUTTON_REPEAT)) {
                skipCnt = b<<2;  //   x4, to make each a 80 ms wait
               }
             }
            return;
          }else {
            loadPos--; // cancling that last advance
            playPos--; // cache not ready, need to wait a bit. Rewinding back to the starting command.
          }
       break;
       case END_OF_MUSIC_0xFD:   bitSet(playFlag,FLAG_NEXT);  return;
       default:  // 0x00 to 0xFD
          if (isCacheReady()) {
            writeAY(b, playBuf[playPos]);
            ADVANCE_PLAY_BUFFER
          } else {
            loadPos--;  // cancling that last advance
            playPos--;  // cache not ready, need to wait a bit. Rewinding back to the starting command.
          }
       break;
    }
  }
}

bool isCacheReady() {
  return playPos != loadPos;
}

ISR(TIMER1_COMPA_vect) {

  if (digitalRead(pinSkip) == LOW) {   
     
      // Reusing 'skipCnt' here, it's being used as a button press repeat delay.
      if (!bitRead(playFlag,FLAG_BUTTON_DOWN)) {
        resetAY();
        skipCnt=1024+60;
        volumeA = volumeB = volumeC = 0;
      }
      if (skipCnt < 1024) {
        skipCnt=1024 + 22;
        bitSet(playFlag,FLAG_NEXT);
        bitSet(playFlag,FLAG_BUTTON_REPEAT);
      } 
      bitSet(playFlag,FLAG_BUTTON_DOWN);
      skipCnt--;
  }else {
     if (bitRead(playFlag,FLAG_BUTTON_DOWN)) {
        bitClear(playFlag,FLAG_BUTTON_DOWN);
        if (!bitRead(playFlag,FLAG_BUTTON_REPEAT)) {
          bitSet(playFlag,FLAG_NEXT);
        }
        bitClear(playFlag,FLAG_BUTTON_REPEAT);
        skipCnt=0;  // finnished reusing this variable, giving it back with correct state
     };
  }

  if ( bitRead(playFlag,FLAG_NEXT) || (--skipCnt > 0)){
    return;
  }

  bitSet(playFlag,FLAG_REFRESH_DISPLAY);  // notify main loop to refresh the display (keeping slow things away/outside from the interrupt)
  playNotes();

}

inline void displayVolumeTop(byte volume) {
  if (volume>=8){
    oled.print( (char) ('!' + (((volume)&0x07)) ) );
    oled.print( (char) ('!' + (((volume)&0x07)) ) );

  }
  else { 
    oled.print( F("  ") );  
  }
}
inline void displayVolumeBottom(byte volume) {
  if (volume<8) {
    oled.print( (char) ('!' + (((volume)&0x07)) ) );
    oled.print( (char) ('!' + (((volume)&0x07)) ) );
  }
  else {
   oled.print( F("(("));
  }
}
