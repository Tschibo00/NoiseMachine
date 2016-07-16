#include <avr/pgmspace.h>
#include "MyLedControl.h"
#include <EEPROM.h>
#include <Wire.h>

#if defined(__AVR_ATmega328P__)
// Timer2 is the same on the mega328 and mega168
#define __AVR_ATmega168__
#endif
//#define profiling    // define this if profiling information in serial monitor shall be outputted

/*
Experimental noise machine
3 oscillators combined with two functions.
First row: Press button 1-3 and turn pots to configure oscillator 1-3
	left: cutoff, middle: resonance, right: frequency
Second row: Activate/mute oscillator 1-3
Third row:  Select operator for oscillator 1+2: Add, Difference, XOR (threshold 200/255), XOR(threshold 40/255)
Fourth row: Select operator for oscillator 3+result of 1+2: Add, Difference, XOR (threshold 200/255), XOR(threshold 40/255)
Switch oscillator type of oscillator 1-3 by clicking left black button row 1-3 (selected waveform is displayed on green LEDs)
Press Black button top right +1-16 to load setting
Press black button top right and the left to it simultaneously +1-16 to write setting
*/
//#define NUMBER_VOICES 3
static byte i;
static long j;
static int16_t dist, sumOsc, sumOld;
static int8_t selectedVoice=0;
static int8_t displayVoice = 0;
/*
// per voice parameters
static uint16_t freq[NUMBER_VOICES];
static uint16_t cnt[NUMBER_VOICES];
static int16_t o[NUMBER_VOICES];
static int16_t memO[NUMBER_VOICES];
static int16_t lastO[NUMBER_VOICES];
static int16_t cutoff[NUMBER_VOICES];
static int16_t resonance[NUMBER_VOICES];
static int16_t volSub[NUMBER_VOICES];        // 0=full volume, 255=silence
static int16_t o_, lastO_, memO_;
*/

static uint8_t selectedSetting=0;

static uint16_t freq1=200,freq2=100,freq3=20;
static uint16_t cnt1,cnt2,cnt3;
static int16_t o1,o2,o3;
static int16_t memO1,memO2,memO3;
static int16_t lastO1,lastO2,lastO3;
static int16_t cutoff1=100,cutoff2=100,cutoff3=100;
static int16_t resonance1=0,resonance2=0,resonance3=0;
static int16_t volSub1=0,volSub2=0,volSub3=0;
static bool active1=true, active2=false, active3=false;
static uint8_t selectedOperator1, selectedOperator2;
static uint8_t selectedWave1, selectedWave2, selectedWave3;

static int16_t cutRead=0;
static int16_t resRead=0;
static uint8_t interruptWaiter;

static uint8_t blinker=0;

#define WAVE1 1
#define WAVE2 2
#define WAVE3 3
#define FILTER 4
#define MODE_LOADSETTING 5
#define MODE_WRITESETTING 6
const byte infoDisp[7*4] PROGMEM =
  {B01000100,B10101010,B10101010,B01000100,      // octave 0	**NOT USED**
   B01000100,B10101100,B10100100,B01000100,      // octave 1
   B01001100,B10100010,B10100100,B01001110,      // octave 2
   B01001100,B10100110,B10100110,B01001100,      // octave 3
   B11101010,B10001010,B11001010,B10001011,      // filter
   B10001100,B10001010,B10001010,B11101100,      // Ld=Load setting
   B10010110,B10010101,B11110110,B11110101,      // Wr=Write setting
 };

const uint8_t sinetable[256] = { 127,130,133,136,139,142,145,148,151,154,157,160,163,166,169,172,
175,178,181,184,186,189,192,194,197,200,202,205,207,209,212,214,
216,218,221,223,225,227,229,230,232,234,235,237,239,240,241,243,
244,245,246,247,248,249,250,250,251,252,252,253,253,253,253,253,
254,253,253,253,253,253,252,252,251,250,250,249,248,247,246,245,
244,243,241,240,239,237,235,234,232,230,229,227,225,223,221,218,
216,214,212,209,207,205,202,200,197,194,192,189,186,184,181,178,
175,172,169,166,163,160,157,154,151,148,145,142,139,136,133,130,
127,123,120,117,114,111,108,105,102,99,96,93,90,87,84,81,
78,75,72,69,67,64,61,59,56,53,51,48,46,44,41,39,
37,35,32,30,28,26,24,23,21,19,18,16,14,13,12,10,
9,8,7,6,5,4,3,3,2,1,1,0,0,0,0,0,
0,0,0,0,0,0,1,1,2,3,3,4,5,6,7,8,
9,10,12,13,14,16,18,19,21,23,24,26,28,30,32,35,
37,39,41,44,46,48,51,53,56,59,61,64,67,69,72,75,
78,81,84,87,90,93,96,99,102,105,108,111,114,117,120,123 };

static LedControl lc=LedControl();

static const byte ROWS = 6; // Four rows
static const byte COLS = 4; // Three columns
static const byte rowPins[ROWS] = { 1,2,3,4,13,0 };
static const byte colPins[COLS] = { 9,8,7,6 }; 
static byte keypadState[ROWS][COLS];
static bool keyLocked[ROWS][COLS];
static byte dispBuff[8];
static uint8_t osciBuffer[128];

#define OLED_I2C_ADDRESS   0x3C
#define OLED_CONTROL_BYTE_CMD_SINGLE	0x80
#define OLED_CONTROL_BYTE_CMD_STREAM	0x00
#define OLED_CONTROL_BYTE_DATA_STREAM	0x40
#define OLED_CONTROL_BYTE_DATA_SINGLE   0xc0
#define OLED_CMD_SET_CONTRAST			0x81	// follow with 0x7F
#define OLED_CMD_DISPLAY_RAM			0xA4
#define OLED_CMD_DISPLAY_ALLON			0xA5
#define OLED_CMD_DISPLAY_NORMAL			0xA6
#define OLED_CMD_DISPLAY_INVERTED 		0xA7
#define OLED_CMD_DISPLAY_OFF			0xAE
#define OLED_CMD_DISPLAY_ON				0xAF
#define OLED_CMD_SET_MEMORY_ADDR_MODE	0x20	// follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE		0x21	// can be used only in HORZ/VERT mode - follow with 0x00 + 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE			0x22	// can be used only in HORZ/VERT mode - follow with 0x00 + 0x07 = PAGE7
#define OLED_CMD_SET_COL_NIBBLE_LO            0x00
#define OLED_CMD_SET_COL_NIBBLE_HI            0x10
#define OLED_CMD_SET_PAGE_START                0xb0
#define OLED_CMD_SET_DISPLAY_START_LINE	0x40
#define OLED_CMD_SET_SEGMENT_REMAP		0xA1	
#define OLED_CMD_SET_MUX_RATIO			0xA8	// follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE		0xC8	
#define OLED_CMD_SET_DISPLAY_OFFSET		0xD3	// follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP		0xDA	// follow with 0x12
#define OLED_CMD_NOP 					0xE3	// NOP
#define OLED_CMD_SET_DISPLAY_CLK_DIV	0xD5	// follow with 0x80
#define OLED_CMD_SET_PRECHARGE			0xD9	// follow with 0x22
#define OLED_CMD_SET_VCOMH_DESELCT		0xDB	// follow with 0x30
#define OLED_CMD_SET_CHARGE_PUMP		0x8D	// follow with 0x14

void setup(){
  cli();//disable interrupts
 
//set timer1 interrupt at 15,625khz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A=15;    // (16.000.000/64/15625)-1 = 16-1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS11 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A) | (1<<OCIE2A);

  /****Set timer0 for 8-bit fast PWM output ****/
  pinMode(5, OUTPUT); // Make timer’s PWM pin an output
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00); 

  /****Set timer2 for sequencer loop ****/
  TCCR2A = (1<<WGM21) | (1<<CS20) | (1<<CS21) | (1<<CS22);
  TCNT2=0;
  OCR2A=250;            // about 1000hz
  TCCR2B|=(1<<WGM12);
  TIMSK2|=(1<<OCIE2A);

  sei();//enable interrupts
  
  // initialize keypad handling
  // set rows to HIGH Z
  for (int i = 0; i < ROWS; i++) {
    pinMode (rowPins[i], INPUT);
    digitalWrite (rowPins[i], LOW);
  }
  // set cols to input with pullups
  for (int j = 0; j < COLS; j++) {
    pinMode (colPins[j], INPUT);
    digitalWrite (colPins[j], HIGH);
  }
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      keypadState[i][j] = 0;
      keyLocked[i][j]=false;
    }
  }  

  oled_init();
  delay(250);				// **** what for? ****
}

int restrictValue(int val, int min, int max) {
  long temp=max-min;
  temp*=(val<0?0:(val>1023?1023:val));        // restrict to valid range
  temp/=1023;
  return temp+min;
}

ISR(TIMER1_COMPA_vect){//timer 1 interrupt
/*  for (int8_t voice=0; voice<NUMBER_VOICES; voice++) {
    cnt[voice]+=freq[voice];              // timer will automatically wraparound at 65535
    o_=(cnt[voice]<32768?100:-100);    // square wave
    lastO_=lastO[voice];
    dist=o_-lastO_;                // LPF with resonance
    memO_=memO[voice];
    memO_+=dist*cutoff[voice]/256;
    lastO_+=memO_+dist*resonance[voice]/256;
    lastO_=(lastO_<-128?-128:(lastO_>127?127:lastO_));      // constrain the value to -128..127
    lastO[voice]=lastO_;
    memO[voice]=(memO_<-128?-128:(memO_>127?127:memO_));          // constrain the value to -128..127
    o_=lastO_-volSub[voice]+128;
    o[voice]=(o_<0?0:o_);
  }
  dist=o[0]+o[1]+o[2]-384;
  OCR0B=(dist<0?0:(dist>255?255:dist));
//  osciBuffer[cnt[selectedVoice]>>9]=OCR0B;
*/
  cnt1+=freq1;              // timer will automatically wraparound at 65535
  switch (selectedWave1) {
	case 0:	o1 = 127 - (cnt1 >> 8); break;     // sawtooth wave
	case 1:	o1 = (cnt1<32768 ? 100 : -100); break;     // sawtooth wave
	case 2:	o1 = sinetable[cnt1 >> 8] - 128; break;    // sine wave
  }
  dist=o1-lastO1;                // LPF with resonance
  memO1+=dist*cutoff1/256;
  lastO1+=memO1+dist*resonance1/256;
  lastO1=(lastO1<-128?-128:(lastO1>127?127:lastO1));      // constrain the value to -128..127
  memO1=(memO1<-128?-128:(memO1>127?127:memO1));          // constrain the value to -128..127
  o1=lastO1-volSub1+128;
  o1=(o1<0?0:o1);

  cnt2+=freq2;              // timer will automatically wraparound at 65535
  switch (selectedWave2) {
  case 0:	o2 = 127 - (cnt2 >> 8); break;     // sawtooth wave
  case 1:	o2 = (cnt2<32768 ? 100 : -100); break;     // sawtooth wave
  case 2:	o2 = sinetable[cnt2 >> 8] - 128; break;    // sine wave
  }
  dist=o2-lastO2;                // LPF with resonance
  memO2+=dist*cutoff2/256;
  lastO2+=memO2+dist*resonance2/256;
  lastO2=(lastO2<-128?-128:(lastO2>127?127:lastO2));      // constrain the value to -128..127
  memO2=(memO2<-128?-128:(memO2>127?127:memO2));          // constrain the value to -128..127
  o2=lastO2-volSub2+128;
  o2=(o2<0?0:o2);

  cnt3+=freq3;              // timer will automatically wraparound at 65535
  switch (selectedWave3) {
  case 0:	o3 = 127 - (cnt3 >> 8); break;     // sawtooth wave
  case 1:	o3 = (cnt3<32768 ? 100 : -100); break;     // sawtooth wave
  case 2:	o3 = sinetable[cnt3 >> 8] - 128; break;    // sine wave
  }
  dist=o3-lastO3;                // LPF with resonance
  memO3+=dist*cutoff3/256;
  lastO3+=memO3+dist*resonance3/256;
  lastO3=(lastO3<-128?-128:(lastO3>127?127:lastO3));      // constrain the value to -128..127
  memO3=(memO3<-128?-128:(memO3>127?127:memO3));          // constrain the value to -128..127
  o3=lastO3-volSub3+128;
  o3=(o3<0?0:o3);
  
  if (!active1)	o1 = 0;
  if (!active2)	o2 = 0;
  if (!active3)	o3 = 0;

  switch (selectedOperator1) {
	case 0:
		dist = o1 + o2;
		break;
	case 1:
		dist = abs(o1 - o2);
		break;
	case 2:
		dist = ((((o1 > 200) && (o2 < 200)) || ((o1 < 200) && (o2 > 200))) ? 255 : 0);
		break;
	case 3:
		dist = ((((o1 > 40) && (o2 < 40))|| ((o1 < 40) && (o2 > 40))) ? 255 : 0);
		break;
  }
  switch (selectedOperator2) {
  case 0:
	  dist = dist + o3;
	  break;
  case 1:
	  dist = abs(dist - o3);
	  break;
  case 2:
	  dist = ((((dist > 200) && (o3 < 200)) || ((dist < 200) && (o3 > 200))) ? 255 : 0);
	  break;
  case 3:
	  dist = ((((dist > 40) && (o3 < 40)) || ((dist < 40) && (o3 > 40))) ? 255 : 0);
	  break;
  }

  OCR0B =(dist<0?0:(dist>255?255:dist));

/*  if (displayVoice == -1) {
	  switch (selectedVoice) {
	  case 0: osciBuffer[cnt1 >> 9] = OCR0B; break;
	  case 1: osciBuffer[cnt2 >> 9] = OCR0B; break;
	  case 2: osciBuffer[cnt3 >> 9] = OCR0B; break;
	  }
  }
  else {*/
	  switch (displayVoice) {
      case -1: osciBuffer[cnt1 >> 9]=OCR0B;break;
  	  case 0: osciBuffer[cnt1 >> 9] = o1; break;
  	  case 1: osciBuffer[cnt2 >> 9] = o2; break;
  	  case 2: osciBuffer[cnt3 >> 9] = o3; break;
	  }
//  }
 }

/* Keyboard layout as follows:
row 0-3: 16 keypad, rows counting from top to bottom, col 0-3 counting from left to right
row 4: separate keys, col 3-0 counting from top to bottom on bread board
*/
void keypad_scan(){
  for (int i = 0; i < ROWS; i++) {
    // set row to LOW
    pinMode (rowPins[i], OUTPUT);

    for (int j = 0; j < COLS; j++) {
      int val = digitalRead (colPins[j]);
      keypadState[i][j] = (val == LOW);
    }

    // set row to High Z
    pinMode (rowPins[i], INPUT);
  }
}

// checks if a key is pressed (i.e. return true as long as key is pressed)
bool getKeyPress(byte i, byte j) {
  return keypadState[i][j];
}

// checks if a key is clicked (i.e. only returns true once until key is depressed)
bool getKeyClick(byte i, byte j) {
  if (keyLocked[i][j]) {
    if (!keypadState[i][j]) {
      keyLocked[i][j]=false;
    }
    return false;
  } else {
    if (keypadState[i][j]) {
      keyLocked[i][j]=true;
      return true;
    } else {
      return false;
    }
  }
}

// returns the bit pattern for pressed function keys. this only returns the current status, no click detection
uint8_t getFunctionKeys() {
  uint8_t t=0;
  for (int8_t i=3; i>=0; i--) {
    t=t<<1;
    t|=keypadState[4][i]|((keypadState[5][i])<<4);
  }
  return t;
}

// get the key click for the note keys 1-16. first click (1-16 in order) is returned
// if no key is pressed, -1 is returned
int8_t getNoteClick() {
  for (byte i=0; i<4; i++)
    for (byte j=0; j<4; j++)
      if (getKeyClick(j, 3-i))
        return i*4+j;
  return -1;
}

// get the key press for the note keys 1-16. first click (1-16 in order) is returned
// if no key is pressed, -1 is returned
int8_t getNotePress() {
  for (byte i=0; i<4; i++)
    for (byte j=0; j<4; j++)
      if (getKeyPress(j, 3-i))
        return i*4+j;
  return -1;
}

// show the current info in the lower 4 lines of the display (i.e. mode, octave, or entered note)
// if index 0 is given, lower 4 lines are cleared
void showInfo(byte textIndex) {
  if (textIndex==0) {
    for (byte i=0;i<4;i++)
      lc.setRow(i+4,0);
  } else {
    for (byte i=0;i<4;i++)
      lc.setRow(i+4,pgm_read_byte(&infoDisp[textIndex*4+i]));
  }
}

void writeSetting() {
  uint16_t targetAddr=selectedSetting*32;    // each setting uses 32 bytes
  EEPROM.update(targetAddr++, freq1>>8);
  EEPROM.update(targetAddr++, freq1&255);
  EEPROM.update(targetAddr++, freq2>>8);
  EEPROM.update(targetAddr++, freq2&255);
  EEPROM.update(targetAddr++, freq3>>8);
  EEPROM.update(targetAddr++, freq3&255);
  EEPROM.update(targetAddr++, (uint16_t)cutoff1>>8);
  EEPROM.update(targetAddr++, (uint16_t)cutoff1&255);
  EEPROM.update(targetAddr++, (uint16_t)cutoff2>>8);
  EEPROM.update(targetAddr++, (uint16_t)cutoff2&255);
  EEPROM.update(targetAddr++, (uint16_t)cutoff3>>8);
  EEPROM.update(targetAddr++, (uint16_t)cutoff3&255);
  EEPROM.update(targetAddr++, (uint16_t)resonance1>>8);
  EEPROM.update(targetAddr++, (uint16_t)resonance1&255);
  EEPROM.update(targetAddr++, (uint16_t)resonance2>>8);
  EEPROM.update(targetAddr++, (uint16_t)resonance2&255);
  EEPROM.update(targetAddr++, (uint16_t)resonance3>>8);
  EEPROM.update(targetAddr++, (uint16_t)resonance3&255);
  EEPROM.update(targetAddr++, (uint8_t)active1);
  EEPROM.update(targetAddr++, (uint8_t)active2);
  EEPROM.update(targetAddr++, (uint8_t)active3);
  EEPROM.update(targetAddr++, selectedOperator1);
  EEPROM.update(targetAddr++, selectedOperator2);
  EEPROM.update(targetAddr++, selectedWave1);
  EEPROM.update(targetAddr++, selectedWave2);
  EEPROM.update(targetAddr++, selectedWave3);
}

void readSetting() {
  uint16_t targetAddr=selectedSetting*32;    // each setting uses 32 bytes
  freq1=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  freq2=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  freq3=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  cutoff1=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  cutoff2=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  cutoff3=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  resonance1=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  resonance2=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  resonance3=(EEPROM.read(targetAddr++)<<8)|EEPROM.read(targetAddr++);
  active1=EEPROM.read(targetAddr++);
  active2=EEPROM.read(targetAddr++);
  active3=EEPROM.read(targetAddr++);
  selectedOperator1=EEPROM.read(targetAddr++);
  selectedOperator2=EEPROM.read(targetAddr++);
  selectedWave1=EEPROM.read(targetAddr++);
  selectedWave2=EEPROM.read(targetAddr++);
  selectedWave3=EEPROM.read(targetAddr++);
}

void updateDispBuff(bool stuffIn[]) {
  for (int i=0; i<4; i++) {
    if (stuffIn[i<<2]) dispBuff[i]|=8;
    if (stuffIn[i<<2|1]) dispBuff[i]|=4;
    if (stuffIn[i<<2|2]) dispBuff[i]|=2;
    if (stuffIn[i<<2|3]) dispBuff[i]|=1;
  }
}

void oled_init() {
  Wire.begin();						// Init the I2C interface (pins A4 and A5 on the Arduino Uno board) in Master Mode.
  TWBR=0;						// Set the I2C to HS mode - 400KHz! TWBR = (CPU_CLK / I2C_CLK) -16 /2. Some report that even 0 is working. **** test it out ****
  Wire.beginTransmission(OLED_I2C_ADDRESS);		// Begin the I2C comm with SSD1306's address (SLA+Write)
  Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);		// Tell the SSD1306 that a command stream is incoming
  Wire.write(OLED_CMD_DISPLAY_OFF);			// Turn the Display OFF
  Wire.write(OLED_CMD_SET_CONTRAST);			// set contrast
  Wire.write(0xff);
  Wire.write(OLED_CMD_SET_VCOMH_DESELCT);		// Set the V_COMH deselect volatage to max (0,83 x Vcc)
  Wire.write(0x30);
  Wire.write(OLED_CMD_SET_MEMORY_ADDR_MODE);		// vertical addressing mode
  Wire.write(0x01);
  Wire.write(OLED_CMD_SET_CHARGE_PUMP);			// Enable the charge pump
  Wire.write(0x14);
  Wire.write(OLED_CMD_DISPLAY_ON);			// Turn the Display ON
  Wire.write(OLED_CMD_SET_PAGE_RANGE);                  // use the current page
  Wire.write(0);
  Wire.write(7);
  Wire.write(OLED_CMD_SET_COLUMN_RANGE);                // use all columns
  Wire.write(0);
  Wire.write(127);
  Wire.endTransmission();
}

/*
********************************
Main loop which runs all the controls, display, sequencer and value stuff
This is called every 5ms to ensure to catch the sync signal and have some proper sequencer timing
The remaining time is used for displaying the oscilloscope display (see loop())
********************************
*/
ISR(TIMER2_COMPA_vect) {
  sei();                    // re-enable interrups directly, so the sound interrupt is ensured to run
  
  interruptWaiter++;
  if (interruptWaiter<5)
    return;
  interruptWaiter=0;

  cutRead=analogRead(A0);
  resRead=1023-analogRead(A1);
  
  keypad_scan();

  volSub1 = 0;
  volSub2 = 0;
  volSub3 = 0;

  int8_t clickedKey;
  switch (getFunctionKeys()) {
    case B10000000:                      // load pattern
      showInfo(MODE_LOADSETTING);
      clickedKey=getNoteClick();
      if (clickedKey!=-1) {
        selectedSetting=clickedKey;
        readSetting();
      }
    break;
    case B10001000:                      // write pattern
      showInfo(MODE_WRITESETTING);
      clickedKey=getNoteClick();
      if (clickedKey!=-1) {
        selectedSetting=clickedKey;
        writeSetting();
      }
    break;
    case B00000000:                      // enter notes
    case B00000100:
    case B00000010:
    case B00000001:
      switch (getNotePress()) {
        case 0:
          selectedVoice=0;
    	    displayVoice = 0;
          showInfo(WAVE1);
        break;
        case 1:
          selectedVoice=1;
    	    displayVoice = 1;
    	    showInfo(WAVE2);
        break;
        case 2:
          selectedVoice=2;
    	    displayVoice = 2;
    	    showInfo(WAVE3);
        break;
      	default:
      		showInfo(0);
          selectedVoice=-1;
      		displayVoice = -1;
      	break;
      }
      if (getKeyClick(4, 2)) {
        selectedWave1 = (selectedWave1+1)%3;
      }
      if (getKeyClick(4, 1)) {
        selectedWave2 = (selectedWave2 + 1) % 3;
      }
      if (getKeyClick(4, 0)) {
        selectedWave3 = (selectedWave3 + 1) % 3;
      }
    
      if (getKeyClick(0, 2)) active1 = !active1;
      if (getKeyClick(1, 2)) active2 = !active2;
      if (getKeyClick(2, 2)) active3 = !active3;
    
      if (getKeyPress(0, 1)) selectedOperator1 = 0;
      if (getKeyPress(1, 1)) selectedOperator1 = 1;
      if (getKeyPress(2, 1)) selectedOperator1 = 2;
      if (getKeyPress(3, 1)) selectedOperator1 = 3;
    
      if (getKeyPress(0, 0)) selectedOperator2 = 0;
      if (getKeyPress(1, 0)) selectedOperator2 = 1;
      if (getKeyPress(2, 0)) selectedOperator2 = 2;
      if (getKeyPress(3, 0)) selectedOperator2 = 3;
    break;
 }

  switch (selectedVoice) {
	  case 0:
		  cutoff1 = cutRead / 4;
		  resonance1 = resRead - 512;
		  freq1 = analogRead(A2);
	  break;
	  case 1:
		  cutoff2 = cutRead / 4;
		  resonance2 = resRead - 512;
		  freq2 = analogRead(A2);
	  break;
	  case 2:
		  cutoff3 = cutRead / 4;
		  resonance3 = resRead - 512;
		  freq3 = analogRead(A2);
	  break;
  }

  dispBuff[0] = (2 << (6 - selectedVoice))|(2<<(2-selectedWave1));
  dispBuff[1] = ((active1 ? 128 : 0) | (active2 ? 64 : 0) | (active3 ? 32 : 0)) | (2 << (2 - selectedWave2));
  dispBuff[2] = (2 << (6 - selectedOperator1)) | (2 << (2 - selectedWave3));
  dispBuff[3] = 2 << (6- selectedOperator2);

  for (int i=0; i<4; i++)                    // update display upper half
    lc.setRow(i, dispBuff[i]);
}


void loop(){
  uint8_t sample, sampleLo, sampleHi;
  uint8_t dispII, page;
  while (true) {
    for (dispII=0; dispII<128; dispII++) {
      Wire.beginTransmission(OLED_I2C_ADDRESS);
      Wire.write(OLED_CONTROL_BYTE_DATA_STREAM);
      sample=(osciBuffer[127-dispII])>>2;
      sampleLo=1<<(sample&7);
      sampleHi=sample>>3;
      for (page=0; page<8; page++)
        Wire.write(page==sampleHi?sampleLo:0);
      Wire.endTransmission();   
    }
  }
}
