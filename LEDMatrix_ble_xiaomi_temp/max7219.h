// MAX7219 functions by Pawel A. Hernik
// 2016.12.10 updated for rotated LED Martices, define ROTATE below (0,90 or 270)
// 2019.01.27 experimental 5-10x faster implementation of shiftOut() for AVR 

// MAX7219 commands:
#define CMD_NOOP   0
#define CMD_DIGIT0 1
#define CMD_DIGIT1 2
#define CMD_DIGIT2 3
#define CMD_DIGIT3 4
#define CMD_DIGIT4 5
#define CMD_DIGIT5 6
#define CMD_DIGIT6 7
#define CMD_DIGIT7 8
#define CMD_DECODEMODE  9
#define CMD_INTENSITY   10
#define CMD_SCANLIMIT   11
#define CMD_SHUTDOWN    12
#define CMD_DISPLAYTEST 15

byte scr[NUM_MAX*8 + 8]; // +8 for scrolled char

void sendCmd(int addr, byte cmd, byte data)
{
  digitalWrite(CS_PIN, LOW);
  for (int i = NUM_MAX-1; i>=0; i--) {
    shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, i==addr ? cmd : 0);
    shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, i==addr ? data : 0);
  }
  digitalWrite(CS_PIN, HIGH);
}

void sendCmdAll(byte cmd, byte data)
{
  digitalWrite(CS_PIN, LOW);
  for (int i = NUM_MAX-1; i>=0; i--) {
    shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, cmd);
    shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, data);
  }
  digitalWrite(CS_PIN, HIGH);
}

void refresh(int addr) {
  for (int i = 0; i < 8; i++)
    sendCmd(addr, i + CMD_DIGIT0, scr[addr * 8 + i]);
}

// fastest version (10x faster than shiftOut), but limited to hardcoded pins
void shiftOutFast(uint8_t val)
{
  for(int i = 0; i < 8; i++)  {
    if(val & (1 << (7-i))) PORTC|=1; else PORTC&=~1; // A2
    PORTC|=1;  PORTC&=~1;  // A0
  }
} 

// slower but the most universal
void shiftOutFast(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
  uint8_t maskDAT = digitalPinToBitMask(dataPin);
  volatile uint8_t *outDAT = portOutputRegister(digitalPinToPort(dataPin));
  uint8_t maskCLK = digitalPinToBitMask(clockPin);
  volatile uint8_t *outCLK = portOutputRegister(digitalPinToPort(clockPin));
  for(int i = 0; i < 8; i++)  {
    if(val & (1 << (7-i))) *outDAT|=maskDAT; else *outDAT&=~maskDAT;
    *outCLK|=maskCLK;  *outCLK&=~maskCLK;
  }
} 

// 4-5x faster than shiftOut
void shiftOutFast(uint8_t maskDAT, volatile uint8_t *outDAT, uint8_t maskCLK, volatile uint8_t *outCLK, uint8_t val)
{
  for(int i = 0; i < 8; i++)  {
    if(val & (1 << (7-i))) *outDAT|=maskDAT; else *outDAT&=~maskDAT;
    *outCLK|=maskCLK;  *outCLK&=~maskCLK;
  }
} 

void refreshAllRot270() {
  byte mask = 0x01;
  for (int c = 0; c < 8; c++) {
    digitalWrite(CS_PIN, LOW);
    for(int i=NUM_MAX-1; i>=0; i--) {
      byte bt = 0;
      for(int b=0; b<8; b++) {
        bt<<=1;
        if(scr[i * 8 + b] & mask) bt|=0x01;
      }
      shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, CMD_DIGIT0 + c);
      shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, bt);
    }
    digitalWrite(CS_PIN, HIGH);
    mask<<=1;
  }
}

void refreshAllRot90() {
  uint8_t maskDAT = digitalPinToBitMask(DIN_PIN);
  volatile uint8_t *outDAT = portOutputRegister(digitalPinToPort(DIN_PIN));
  uint8_t maskCLK = digitalPinToBitMask(CLK_PIN);
  volatile uint8_t *outCLK = portOutputRegister(digitalPinToPort(CLK_PIN));
  byte mask = 0x80;
  for (int c = 0; c < 8; c++) {
    digitalWrite(CS_PIN, LOW);
    for(int i=NUM_MAX-1; i>=0; i--) {
      byte bt = 0;
      for(int b=0; b<8; b++) {
        bt>>=1;
        if(scr[i * 8 + b] & mask) bt|=0x80;
      }
      #if FASTSHIFT==1
      shiftOutFast(maskDAT, outDAT, maskCLK, outCLK, CMD_DIGIT0 + c);
      shiftOutFast(maskDAT, outDAT, maskCLK, outCLK, bt);
      #else
      shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, CMD_DIGIT0 + c);
      shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, bt);
      #endif
    }
    digitalWrite(CS_PIN, HIGH);
    mask>>=1;
  }
}

void refreshAll() {
#if ROTATE==270
  refreshAllRot270();
#elif ROTATE==90
  refreshAllRot90();
#else
  for (int c = 0; c < 8; c++) {
    digitalWrite(CS_PIN, LOW);
    for(int i=NUM_MAX-1; i>=0; i--) {
      shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, CMD_DIGIT0 + c);
      shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, scr[i * 8 + c]);
    }
    digitalWrite(CS_PIN, HIGH);
  }
#endif
}


void clr()
{
  for (int i = 0; i < NUM_MAX*8; i++) scr[i] = 0;
}

void scrollLeft()
{
  for(int i=0; i < NUM_MAX*8+7; i++) scr[i] = scr[i+1];
}

void invert()
{
  for (int i = 0; i < NUM_MAX*8; i++) scr[i] = ~scr[i];
}

void initMAX7219()
{
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  sendCmdAll(CMD_DISPLAYTEST, 0);
  sendCmdAll(CMD_SCANLIMIT, 7);
  sendCmdAll(CMD_DECODEMODE, 0);
  sendCmdAll(CMD_INTENSITY, 0); // minimum brightness
  sendCmdAll(CMD_SHUTDOWN, 0);
  clr();
  refreshAll();
}

// =======================================================================

void showChar(char ch, int col, const uint8_t *data)
{
  int w = pgm_read_byte(data);
  for(int i=0; i<w; i++) scr[col+i] = pgm_read_byte(data+1+ch*w+i);
}

// =======================================================================

void setCol(int col, byte v)
{
  scr[col] = v;
}

// =======================================================================

