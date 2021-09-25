// This sketch plays PSG files via a AY-3-8910 which
// is a 3-voice programmable sound generator (PSG)
// Code has been optimised (mainly to save space) and will now fit on a 'ATmega168'

// This code is based on "AY-3-8910 Player Arduino OLED" hosted on PCBWAY (you can find a link to the original code that I've reworked)
// I ordered 5 PCB's from them, take a look here https://www.pcbway.com/project/shareproject/AY_3_8910_Player_Arduino_OLED.html
//
// Just noticed another sketch on github, https://gist.github.com/anteo/0e5d8867df7568a6523d54e19983d8e0
// At a guess, looks like the PCBWAY's version by Roman Kubat is a fork of this.

#include <SPI.h>
#include "SdFat.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include "fudgefont.h"  // Based on the Adafruit5x7 font, with '!' to '(' changed to work as a VU BAR (8 chars)

// **********************************************
//#define BUFFER_SIZE (64)  //  *** ATmega168 (1K) ***  Smaller buffer/non optimized - limiting size as we need all the memory we can get.
#define BUFFER_SIZE (256)   //  *** ATmega328 (2K) ***  Allows optimised byte counters which can safely wrap back to zero.
byte playBuf[BUFFER_SIZE];  // 256 MAX 
// **********************************************

//*** Compiler results for a ATmega168 ("cut down" pro mini with JUST 1k, Atmega238P has 2k) ***
//Sketch uses 12602 bytes (87%) of program storage space. Maximum is 14336 bytes.
//Global variables use 1009 bytes (98%) of dynamic memory, leaving 15 bytes for local variables. Maximum is 1024 bytes.
//
// Without doing something drastic the ATmega168 ONLY HAS 15 bytes free!    
// SO DROPPED CACHE SIZE DOWN TO 64 bytes ON THE 1k VERSION

#if (BUFFER_SIZE==256)
#define ADVANCE_PLAY_BUFFER  circularBufferReadIndex++; 
#define ADVANCE_LOAD_BUFFER  circularBufferLoadIndex++; 
#else 
#define ADVANCE_PLAY_BUFFER  circularBufferReadIndex++; if (playPos>=BUFFER_SIZE) playPos=0; 
#define ADVANCE_LOAD_BUFFER  circularBufferLoadIndex++; if (loadPos>=BUFFER_SIZE) loadPos=0; 
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

// Arduino (Atmega) pins default to INPUT (high-impedance state)
enum {
  pinReset = 2,   // AY38910 reset
  pinClock,       // AY38910 clock
  pinSHCP,        // 74HC595 clock
  pinSTCP,        // 74HC595 latch
  pinDS,          // 74HC595 data
  pinNextButton,  // advance to next tune
  pinBC1,         // AY38910 BC1
  pinBDIR,        // AY38910 BDIR
  pinCS           // SD card select (CS)
};

// 128x32 i2c OLED - 0.96".  
// SDA=A4 ,  SCL=A5
#define I2C_ADDRESS (0x3C)   // used by oled
SSD1306AsciiAvrI2c oled;

SdFat  m_sd;  // takes 595 bytes
SdFile m_fp;  // takes 36 bytes
SdFile m_dir; // takes 36 bytes

enum  { FLAG_PLAY, FLAG_PLAY_NEXT_TUNE, FLAG_BUTTON_DOWN, FLAG_BUTTON_REPEAT, FLAG_REFRESH_DISPLAY };

byte playFlag;
 
#define VU_METER_INTERNAL_SCALE (2)  // speed scale 2^n values

int filesCount = 0;
int fileIndex = 0;  // file indexes start from zero
int interruptCountSkip = 0;   // Don't play new sequences via interrupt when this is positive (do nothing for 20ms)
uint32_t fileSize;

byte circularBufferLoadIndex;  // WARNING : this counter wraps back to zero by design
byte circularBufferReadIndex;  // WARNING : this counter wraps back to zero by design
byte volumeChannelA=0;
byte volumeChannelB=0;
byte volumeChannelC=0;

// startup the display, audit files, fire-up timers at 1.75 MHz'ish, reset AY chip
void setup() {

  //Serial.begin(9600);   // this library eats 177 bytes, remove from release
  //Serial.println(sizeof(SdFat));

  setupPins();
  setupClockForAYChip();
  resetAY();
  
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont( fudged_Adafruit5x7 ); // original Adafruit5x7 font with tweeks at start for VU meter

  SD_CARD_MISSING_RETRY:
  oled.clear();
  // The SD card reader must keep up with the logic process (both run at 50MHz)
  if (m_sd.begin(pinCS, SD_SCK_MHZ(50))) {
    countPlayableFiles();
  } else {
    oled.print(F("Waiting for SD card"));
    delay(1000*2);
    goto SD_CARD_MISSING_RETRY; 
  }

  bitSet(playFlag,FLAG_PLAY_NEXT_TUNE);
  bitSet(playFlag,FLAG_REFRESH_DISPLAY);
  setupProcessLogicTimer(); // start the logic interrupt up last
}

// This main loop looks after advancing to the next tune, caching and refreshing the display.
// Music data is cached from file (sd card) using a circular buffer.
void loop() {

  if (bitRead(playFlag,FLAG_PLAY_NEXT_TUNE)) {
    resetAY();
    oled.clear();  // called early as selectFile uses oled to display filename
    selectFile(fileIndex);

    oled.setCursor(0, DISPLAY_ROW_FILE_COUNTER);
    oled.print(fileIndex+1); 
    oled.print(F("/"));
    oled.print(filesCount);

    fileIndex++;
    if (fileIndex >= filesCount)
       fileIndex = 0;

    bitClear(playFlag,FLAG_PLAY_NEXT_TUNE);
    bitSet(playFlag,FLAG_PLAY);
  }

  cacheSingleByteRead();  //cache more music data if needed

  if (bitRead(playFlag,FLAG_REFRESH_DISPLAY)) {  
    
    oled.setCursor((128/2)-6-6-6,DISPLAY_ROW_VU_METER_TOP);
    displayVuMeterTopPar(volumeChannelA/VU_METER_INTERNAL_SCALE);  // dividing by 2, scaled maths used (*2 scale used setting VU meter)
    displayVuMeterTopPar(volumeChannelB/VU_METER_INTERNAL_SCALE);
    displayVuMeterTopPar(volumeChannelC/VU_METER_INTERNAL_SCALE);

    oled.setCursor((128/2)-6-6-6,DISPLAY_ROW_VU_METER_BOTTOM);
    displayVuMeterBottomPar(volumeChannelA/VU_METER_INTERNAL_SCALE);
    displayVuMeterBottomPar(volumeChannelB/VU_METER_INTERNAL_SCALE);
    displayVuMeterBottomPar(volumeChannelC/VU_METER_INTERNAL_SCALE);

    oled.setCursor(128-32, DISPLAY_ROW_BYTES_LEFT);
    oled.print(fileSize/1024); 
    oled.print("K "); 
    
    volumeChannelA--;  // drift all the VU meters down over time (values are internally scaled)
    volumeChannelB--;
    volumeChannelC--;  

    bitClear(playFlag,FLAG_REFRESH_DISPLAY);
  }
}

// Configure the direction of the digital pins. (pins default to INPUT at power up)
void setupPins() {  
  pinMode(pinReset, OUTPUT);
  pinMode(pinClock, OUTPUT);
  pinMode(pinSHCP, OUTPUT);
  pinMode(pinSTCP, OUTPUT);
  pinMode(pinDS, OUTPUT);
  // A push button switch is connected between 'pinNextButton' & ground, 
  // without the need for any resistor as reference to 5V thanks to the internal pull-up. Nice.
  pinMode(pinNextButton, INPUT_PULLUP);
  pinMode(pinBC1, OUTPUT);
  pinMode(pinBDIR, OUTPUT);
  // pinMode(pinCS, INPUT);
}

// The AY38910 clock pin needs to be driven at a frequency of 1.75 MHz.
// We can get an interrupt to trigger at 1.778 MHz ( 1.5873% difference, close enough)
#define PERIOD  (9)                 // 9 CPU cycles (or 1.778 MHz)
void setupClockForAYChip() {
  TCCR2B = 0;                       // stop timer
  TCNT2 = 0;                        // reset timer
  TCCR2A = _BV(COM2B1)              // non-inverting PWM on OC2B (pin3)
          |_BV(WGM20)|_BV(WGM21);   // fast PWM mode, TOP=OCR2A
  TCCR2B = _BV(WGM22)|_BV(CS20);    // 
  OCR2A = PERIOD - 1;               // timer2 counts from 0 to 8 (9 cycles at 16 MHz)
  OCR2B = (PERIOD / 2) - 1;         // duty cycle (remains high count part)
}

// Clear Timer on Compare 
// Interrupt on "Timer/Counter1 Compare Match A"
// 16000000 (ATmega16MHz) / 256 prescaler = 62500ms , 62500ms / 1250 counts = 50ms
// 1000ms / 50ms = 20ms
void setupProcessLogicTimer() {
  cli();                            // Disable interrupts while setting up
  TCCR1A = 0;                       // Timer/Counter Control register
  TCNT1 = 0;                        // Timer/Counter Register 
  TCCR1B = _BV(WGM12) | _BV(CS12);  // CTC mode,256 prescaler
  TIMSK1 = _BV(OCIE1A);             // Enable timer compare interrupt
  OCR1A = 1250;  	                  // compared against TCNT1 if same ISR is called
  sei();                            // enable interrupt
} 

void countPlayableFiles() {
 // SdFile* file = (SdFile*)playBuf;  // buffers not currently in use, so putting it to good use :)
  if (m_dir.open("/", O_READ)) {  // is never closed, as we are using rewind later.
    while (m_fp.openNext(&m_dir, O_READ)) {
      if (m_fp.isFile()) {
         if (isFilePSG()) {
          filesCount++;
         }
      }
      m_fp.close();
    }
    // *** m_dir.close();  Don't close, this is fine - We just leave it open forever  ***
  }
}

void selectFile(int fileIndex) {
  m_fp.close();  // FUDGE
  m_dir.rewind();  // is always left open, so we just rewind 
  int k = 0;
  while (m_fp.openNext(&m_dir, O_READ)) {
    if (m_fp.isFile()) { 
      if (isFilePSG()) {
        if (k == fileIndex) { 
          fileSize = m_fp.fileSize();
          oled.setCursor(0,DISPLAY_ROW_FILENAME);
          // We can safely use the play buffer here to save dynamic memory.
          m_fp.getName((char*)playBuf,22);  // 21 characters (+1 for null).  128pixel display using 'Adafruit5x7' font i.e 128/(5+gap of 1) 
          oled.print((char*)playBuf);
          advancePastHeader();
          break;  // Found it - leave this file open, cache takes over from here on and process it.
        }
      }
      k++;
    }
    m_fp.close(); // This isn't the file you're looking for, continue looking.
  }
}

// Reads a single byte from the SD card into the cache (circular buffer)
// Anything after EOF sets a END_OF_MUSIC_0xFD command into the cache
// which will trigger a FLAG_PLAY_NEXT_TUNE.
void cacheSingleByteRead() {
  if (circularBufferLoadIndex == circularBufferReadIndex - 1)  // cache is behind, wait for it to refill
    return ;
  if (circularBufferLoadIndex == (BUFFER_SIZE - 1) && circularBufferReadIndex == 0)  // cache is behind
     return ;

  if (m_fp.available()) {
    playBuf[circularBufferLoadIndex] =  m_fp.read();
    fileSize--; 
  }
  else {
     playBuf[circularBufferLoadIndex] = END_OF_MUSIC_0xFD;
  }
  ADVANCE_LOAD_BUFFER
}

// Bypass header information, we can't make use of the extra playback features.
//
// Example of Header(16 bytes) followed by thee start of the raw byte data
// HEADER: 50 53 47 1A 00 00 00 00 00 00 00 00 00 00 00 00 
// DATA  : FF FF 00 F9 06 16 07 38 FF 00 69 06 17 FF 00 F9 ...
//         ^^ ^^
void advancePastHeader() {
  circularBufferLoadIndex = circularBufferReadIndex = 0;
  while (m_fp.available()) {
    byte b = m_fp.read();
    if (b == 0xFF) {
      // The raw data always starts with two 0xFF's, we use this FF marker to skip past the header.
      // Note: We only need to find the first FF.. the player code will see the next FF
      //       and treat it as a do nothing 20ms pause.  I have no idea if that's the plan for this PSG format
      //       I'm doubting anyone will care (i.e. instead do two FF's skips and so no 20ms pause).
      break; 
    }
  }
}

// Reset AY chip to stop sound output
// Reset line needs to go High->Low->High for AY38910/12
// Reset pulse width must be 500ns (min), this comes from the AY38910/12 datasheet.
void resetAY() {
  setAYMode(INACTIVE);
  digitalWrite(pinReset, LOW); // just assume it starts high - we only care about the low edges
  delay(1);  // to be safe
  digitalWrite(pinReset, HIGH);
  delay(1);
}

// PORTB maps to Arduino digital pins 8 to 13 (PB0 to PB5)
// pins (PB6 and PB7) are not available ( 16MHz crytsal is connected with Pin-9 (XTAL2/PB6) and Pin-10 (XTAL1/PB7)
void setAYMode(AYMode mode) {
  switch (mode) {
    case INACTIVE:  PORTB &= _BV(PB7)|_BV(PB6)|_BV(PB5)|_BV(PB4)|_BV(PB3)|_BV(PB2); break;  //B11111100
    case WRITE:     PORTB |= _BV(PB1); break;             // output: pin 9
    case LATCH:     PORTB |= _BV(PB1)|_BV(PB0);  break;   // output: pins 8,9
  }
}

// Update VU meter & send latching data to AY chip
// NOTE: *** Used by interrupt, keep code lightweight ***
void writeAY( byte port , byte data ) {
  if (port < 16) {
    switch (port) {
      case 8: volumeChannelA = data*VU_METER_INTERNAL_SCALE; break;   // *2 for scaled maths (VU meter speed)
      case 9: volumeChannelB = data*VU_METER_INTERNAL_SCALE; break;
      case 10:volumeChannelC = data*VU_METER_INTERNAL_SCALE; break;         
    }  
        
    // send data to the 74HC595 chip (limited amount of pins available on a Arduino pro mini)
    // *** Port data ***
    setAYMode(INACTIVE);
    digitalWrite(pinSTCP, LOW);                 // Hold 74HC595 latchPin low for transmit
    shiftOut(pinDS, pinSHCP, MSBFIRST, port);   // byte data out (inernally one bit at a time) 
    digitalWrite(pinSTCP, HIGH);                // 74HC595 latch pin high we are done dont listen any more 
    setAYMode(LATCH);  
    // *** Command data ***
    setAYMode(INACTIVE);
    digitalWrite(pinSTCP, LOW);
    shiftOut(pinDS, pinSHCP, MSBFIRST, data);
    digitalWrite(pinSTCP, HIGH);
    setAYMode(WRITE);
    setAYMode(INACTIVE);
  }
}

// PSG music format (body)
// [0xff]              : End of interrupt (EOI) - waits for 20 ms
// [0xfe],[byte]       : Multiple EOI, following byte provides how many times to wait 80ms.
// [0xfd]              : End Of Music
// [0x00..0x0f],[byte] : PSG register, following byte is accompanying data for this register
// (Again... This method need to be lightweight as it's part of the interrupt)
void playNotes() {
  while (isCacheReady()) {
    byte b = playBuf[circularBufferReadIndex];
    ADVANCE_PLAY_BUFFER
    switch(b) {
       case END_OF_INTERRUPT_0xFF: return;
       case END_OF_INTERRUPT_MULTIPLE_0xFE:
          if (isCacheReady()) {
            b = playBuf[circularBufferReadIndex];
            ADVANCE_PLAY_BUFFER
            
            if ((b==0xff) && (fileSize/32==0) )  {
                // Some tunes have ver long pauses at the end (caused by repeated sequences of "0xfe 0xff").
                // For example "NewZealandStoryThe.psg" has a very long pause at the end, I'm guessing by design to handover to the ingame tune.
               interruptCountSkip=4;  // 4 works well for me! Forcing shorter pauses, but only when nearing the end of the tune and its FF
               resetAY();  
               // keeps doing this special timing adjustment for the last 32 bytes read
            }
            else {
               if (!bitRead(playFlag,FLAG_BUTTON_REPEAT)) {
                interruptCountSkip = b<<2;  //   x4, to make each a 80 ms wait
               }
            }
            return;
          }else {
            circularBufferLoadIndex--; // canceling  that last advance
            circularBufferReadIndex--; // cache not ready, need to wait a bit. Rewinding back to the starting command.
          }
       break;
       case END_OF_MUSIC_0xFD:   bitSet(playFlag,FLAG_PLAY_NEXT_TUNE);  return;
       default:  // 0x00 to 0xFC
          if (isCacheReady()) {
            writeAY(b, playBuf[circularBufferReadIndex]);
            ADVANCE_PLAY_BUFFER
          } else {
            circularBufferLoadIndex--;  // canceling  that last advance
            circularBufferReadIndex--;  // cache not ready, need to wait a bit. Rewinding back to the starting command.
          }
       break;
    }
  }
}

// Timer/Counter1 Compare Match A
ISR(TIMER1_COMPA_vect) {
  if (digitalRead(pinNextButton) == LOW) {   
      // Reusing 'interruptCountSkip' here, it's being used as a button press repeat delay.
      if (!bitRead(playFlag,FLAG_BUTTON_DOWN)) {
        resetAY();
        interruptCountSkip=1024+60;
        volumeChannelA = volumeChannelB = volumeChannelC = 0;
      }
      if (interruptCountSkip < 1024) {
        interruptCountSkip=1024 + 22;
        bitSet(playFlag,FLAG_PLAY_NEXT_TUNE);
        bitSet(playFlag,FLAG_BUTTON_REPEAT);
      } 
      bitSet(playFlag,FLAG_BUTTON_DOWN);
      interruptCountSkip--;
  }else {
     if (bitRead(playFlag,FLAG_BUTTON_DOWN)) {
        bitClear(playFlag,FLAG_BUTTON_DOWN);
        if (!bitRead(playFlag,FLAG_BUTTON_REPEAT)) {
          bitSet(playFlag,FLAG_PLAY_NEXT_TUNE);
        }
        bitClear(playFlag,FLAG_BUTTON_REPEAT);
        interruptCountSkip=0;  // finished reusing this variable, giving it back with correct state
     };
  }

  if ( bitRead(playFlag,FLAG_PLAY_NEXT_TUNE) || (--interruptCountSkip > 0)){
    return;
  }

  bitSet(playFlag,FLAG_REFRESH_DISPLAY);  // notify main loop to refresh the display (keeping slow things away/outside from the interrupt)
  playNotes();
}

// Q: Why top and bottom functions?
// A: Two character are joined to make a tall VU meter.
inline void displayVuMeterTopPar(byte volume) {
  if (volume>=8){
    // Note: x8 characters have been redefined for the VU memter starting from '!' 
    oled.print( (char) ('!' + (((volume)&0x07)) ) ); 
    oled.print( (char) ('!' + (((volume)&0x07)) ) );
  }
  else { 
    oled.print( F("  ") );  // nothing to show, clear.  (F() puts text into program mem) 
  }
}
inline void displayVuMeterBottomPar(byte volume) {
  if (volume<8) {
    // Note: x8 characters have been redefined for the VU memter starting from '!' 
    oled.print( (char) ('!' + (((volume)&0x07)) ) );
    oled.print( (char) ('!' + (((volume)&0x07)) ) );
  }
  else {
   oled.print( F("(("));  // '(' is redefined as a solid bar for VU meter
  }
}

// Returns true when data is waiting and ready on the cache.
bool isCacheReady() {
  return circularBufferReadIndex != circularBufferLoadIndex;
}

// Here we are checking for the "PSG" file header, byte by byte.
bool isFilePSG() {
  // Doing every little bit to save some dynamic memory.
  // Could put "PSG" in program mem, but that would still require a counter eating away at the stack (yes one byte but I'm very low on mem)
  return (m_fp.available() && m_fp.read() == 'P' && m_fp.available() &&  m_fp.read()=='S' && m_fp.available() && m_fp.read()=='G');
}
