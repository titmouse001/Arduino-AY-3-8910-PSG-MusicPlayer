// This sketch plays PSG files via a AY-3-8910 which
// is a 3-voice programmable sound generator (PSG)

// optimised code to save space, will now fit on a 'ATmega168' ... just 15 bytes free for stack/local variables

// This code is based on "AY-3-8910 Player Arduino OLED" hosted on PCBWAY

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
// ATmega168 ONLY HAS 15 bytes FREE !    
// TOO LITTLE FOR CODE TO RUN... SO DROPPED CACHE SIZE DOWN TO 64 bytes ON THE 1k VERSION

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

enum {
  pinReset = 2, // AY38910 reset
  pinClock,     // AY38910 clock
  pinSHCP,      // 74HC595 clock
  pinSTCP,      // 74HC595 latch
  pinDS,        // 74HC595 data
  pinNextButton,      // advance to next tune
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

 // Serial.begin(9600);   // this library eats 177 bytes, remove from release
  //Serial.println(sizeof(SdFat));

  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont( fudged_Adafruit5x7 ); // original Adafruit5x7 font with tweeks at start for VU meter
  oled.clear();
  
  if (m_sd.begin(pinCS, SD_SCK_MHZ(50))) {
    countPlayableFiles();
  }

  setupPins();
  setupProcessLogicTimer();
  setupClockForAYChip();
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
  pinMode(pinNextButton, INPUT_PULLUP);
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
// interrupt on "Timer/Counter1 Compare Match A"
// 16000000 (16MHz) / 256 prescaler = 62500ms , 62500ms / 1250 counts = 50ms
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
         // really need to save as much dynamic memory as possible.  Here we are checking for the "PSG" file header byte by byte.
         if (m_fp.available() && m_fp.read() == 'P' && m_fp.available() &&  m_fp.read()=='S' && m_fp.available() && m_fp.read()=='G') {
          filesCount++;
         }
      }
      m_fp.close();
    }
    // *** m_dir.close();  Don't close, this is fine - We just leave it open forever  ***
  }
}

// This main loop looks after advancing to the next tune, caching and refreshing the display.
// Music data is cached from file (sd card) using a circular buffer.
void loop() {

  if (bitRead(playFlag,FLAG_NEXT)) {
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

    bitClear(playFlag,FLAG_NEXT);
    bitSet(playFlag,FLAG_PLAY);
  }

  loadNextByte();  //cache more music data if needed

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

  if (circularBufferLoadIndex == circularBufferReadIndex - 1)  // cache is behind, wait for it to refill
    return ;
  if (circularBufferLoadIndex == (BUFFER_SIZE - 1) && circularBufferReadIndex == 0)  // cache is behind
     return ;

  if (m_fp.available()) {
    playBuf[circularBufferLoadIndex] =  m_fp.read();
    fileSize--; 
  }
  else {
     playBuf[circularBufferLoadIndex] = 0xFD;
  }
  ADVANCE_LOAD_BUFFER
}

// Bypass header information, we can't make use of the extra playback features.
void advancePastHeader() {
  circularBufferLoadIndex = circularBufferReadIndex = 0;
  while (m_fp.available()) {
    byte b = m_fp.read();
    if (b == 0xFF) break; // body data stats with a FF, we use this to skip over the header
  }
}

// Reset AY chip to stop sound output
// Reset line needs to go High->Low->High for AY38910/12
// Reset pulse width must be 500ns (min)
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
// This method need to be lightweight as it's part of the interrupt)
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
                // Some tunes have tones of repeated ending sequences of "0xfe 0xff", "0xff" is a long pause.
                // example : "NewZealandStoryThe" has a very long pause at the end I'm guessing by design to handover to the ingame tune.
               interruptCountSkip=4;  // 4 works well for me! Forcing shorter pauses, but only when nearing the end of the tune and its FF
               resetAY();  
               // keeps doing this timing adjustment for the last 32 bytes
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
       case END_OF_MUSIC_0xFD:   bitSet(playFlag,FLAG_NEXT);  return;
       default:  // 0x00 to 0xFD
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
        bitSet(playFlag,FLAG_NEXT);
        bitSet(playFlag,FLAG_BUTTON_REPEAT);
      } 
      bitSet(playFlag,FLAG_BUTTON_DOWN);
      interruptCountSkip--;
  }else {
     if (bitRead(playFlag,FLAG_BUTTON_DOWN)) {
        bitClear(playFlag,FLAG_BUTTON_DOWN);
        if (!bitRead(playFlag,FLAG_BUTTON_REPEAT)) {
          bitSet(playFlag,FLAG_NEXT);
        }
        bitClear(playFlag,FLAG_BUTTON_REPEAT);
        interruptCountSkip=0;  // finished reusing this variable, giving it back with correct state
     };
  }

  if ( bitRead(playFlag,FLAG_NEXT) || (--interruptCountSkip > 0)){
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
