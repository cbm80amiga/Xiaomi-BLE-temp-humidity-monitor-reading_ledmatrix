// Reading Xiaomi Mi BLE Temperature & Humidity Monitor
// Code for the video: https://youtu.be/MlbOM6DuulQ
// (C)2019 Pawel A. Hernik
// BLE code based on Dmitry Grinberg and Florian Echtler work

/* PINOUT:
  MAX7219 LED Matrix:
  DIN A2/D16
  CS  A1/D15
  CLK A0/D14

  nRF24L01 from pin side/top:
  -------------
  |1 3 5 7    |
  |2 4 6 8    |
  |           |
  |           |
  |           |
  |           |
  |           |
  -------------
  1 - GND  blk   GND
  2 - VCC  wht   3V3
  3 - CE   orng  9
  4 - CSN  yell  10
  5 - SCK  grn   13
  6 - MOSI blue  11
  7 - MISO viol  12
  8 - IRQ  gray  2

 More info about nRF24L01:
 http://arduinoinfo.mywikis.net/wiki/Nrf24L01-2.4GHz-HowTo#po1
*/


#include <SPI.h>
#include <RF24.h>
RF24 radio(9,10);

#define NUM_MAX 4
#define ROTATE  90
#define FASTSHIFT 1

#define DIN_PIN 16
#define CS_PIN  15
#define CLK_PIN 14

#include "max7219.h"

// serial debug - all packets dump
int sdebug = 1;

// =======================================================================
const uint8_t channel[3]   = {37,38,39};  // BLE advertisement channel number
const uint8_t frequency[3] = { 2,26,80};  // real frequency (2400+x MHz)

struct bleAdvPacket { // for nRF24L01 max 32 bytes = 2+6+24
  uint8_t pduType;
  uint8_t payloadSize;  // payload size
  uint8_t mac[6];
  uint8_t payload[24];
};

uint8_t currentChan=0;
bleAdvPacket buffer;

void initBLE() 
{
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.disableCRC();
  radio.setChannel( frequency[currentChan] );
  radio.setRetries(0,0);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAddressWidth(4);
  radio.openReadingPipe(0,0x6B7D9171); // advertisement address: 0x8E89BED6 (bit-reversed -> 0x6B7D9171)
  radio.openWritingPipe(  0x6B7D9171);
  radio.powerUp();
}

void hopChannel()
{
  currentChan++;
  if(currentChan >= sizeof(channel)) currentChan = 0;
  radio.setChannel( frequency[currentChan] );
}

bool receiveBLE(int timeout)
{
  radio.startListening();
  delay(timeout);
  if(!radio.available()) return false;
  while(radio.available()) {
    radio.read( &buffer, sizeof(buffer) );
    swapbuf( sizeof(buffer) );
    whiten( sizeof(buffer) );
  }
  return true;
}

// change buffer content to "wire bit order"
void swapbuf(uint8_t len) 
{
  uint8_t* buf = (uint8_t*)&buffer;
  while(len--) {
    uint8_t a = *buf;
    uint8_t v = 0;
    if (a & 0x80) v |= 0x01;
    if (a & 0x40) v |= 0x02;
    if (a & 0x20) v |= 0x04;
    if (a & 0x10) v |= 0x08;
    if (a & 0x08) v |= 0x10;
    if (a & 0x04) v |= 0x20;
    if (a & 0x02) v |= 0x40;
    if (a & 0x01) v |= 0x80;
    *(buf++) = v;
  }
}

void whiten(uint8_t len)
{
  uint8_t* buf = (uint8_t*)&buffer;
  // initialize LFSR with current channel, set bit 6
  uint8_t lfsr = channel[currentChan] | 0x40;
  while(len--) {
    uint8_t res = 0;
    // LFSR in "wire bit order"
    for (uint8_t i = 1; i; i <<= 1) {
      if (lfsr & 0x01) {
        lfsr ^= 0x88;
        res |= i;
      }
      lfsr >>= 1;
    }
    *(buf++) ^= res;
  }
}

// =======================================================================

char buf[100];
int temp=-1000;
int hum=-1;
int bat=-1;
int x,cnt=0,mode=0,v1,v10;
int tempOld=-1230;
int humOld=-123;
int batOld=-123;
int cntOld = -1;
char *modeTxt="";

const uint8_t digits3x7[] PROGMEM = {3,
  0x7F, 0x41, 0x7F, // 0
  0x04, 0x02, 0x7F,
  0x79, 0x49, 0x4F,
  0x41, 0x49, 0x7F,
  0x1F, 0x10, 0x7C,
  0x4F, 0x49, 0x79,
  0x7F, 0x49, 0x79,
  0x01, 0x71, 0x0F,
  0x7F, 0x49, 0x7F,
  0x4F, 0x49, 0x7F, // 9
  0x08, 0x08, 0x08, // 10 -
  0x07, 0x05, 0x07, // 11 deg
  0x31, 0x0c, 0x23, // 12 %
};

void setup() 
{
  initBLE();
  Serial.begin(115200);
  Serial.println(F("BLE Mi Bluetooth Temperature & Humidity Monitor"));
  initMAX7219();
  sendCmdAll(CMD_SHUTDOWN, 1);
  sendCmdAll(CMD_INTENSITY, 0);
  clr();
  refreshAll();
}
// =======================================================================
bool isTempOK(int v) { return (v>=-400 && v<=800); }
bool isHumOK(int v) { return (v>=0 && v<=1000); }

// =======================================================================
void loop()
{
  clr();
  receiveBLE(100);
  uint8_t *recv = buffer.payload;
  if(buffer.mac[5]==0x4c && buffer.mac[1]==0x3b && buffer.mac[0]==0xe)  // limit to my Xiaomi MAC address
  if(recv[5]==0x95 && recv[6]==0xfe && recv[7]==0x50 && recv[8]==0x20)
  {
    cnt=recv[11];
    mode=recv[18];
    int mesSize=recv[3];
    int plSize=buffer.payloadSize-6;
    if(mode==0x0d && plSize==25) { // temperature + humidity (missing msb, lsb is reconstructed from previous value)
      //setCol(17,0x80);
      temp=recv[21]+recv[22]*256;
      modeTxt="TH";
      if(sdebug) snprintf(buf,100,"#%02x %02x %s %02x %3d'C (%3d%%)",cnt,mode,modeTxt,recv[3],recv[21]+recv[22]*256,recv[23]);
      if(humOld>0) {  // reconstructing humidity from previous value and new lsb
        hum = (humOld & ~0xff) | recv[23];
        if(hum-humOld>128) hum -= 256; else
        if(humOld-hum>128) hum += 256;
      }
    } else if(mode==0x04 && plSize==23) {  // temperature
      //setCol(18,0x80);
      temp=recv[21]+recv[22]*256;
      modeTxt="T ";
      if(sdebug) snprintf(buf,100,"#%02x %02x %s %02x %3d'C       ",cnt,mode,modeTxt,recv[3],recv[21]+recv[22]*256);
    } else if(mode==0x06 && plSize==23) {  // humidity
      //setCol(19,0x80);
      hum=recv[21]+recv[22]*256;
      modeTxt="H ";
      if(sdebug) snprintf(buf,100,"#%02x %02x %s %02x %3d%%        ",cnt,mode,modeTxt,recv[3],recv[21]+recv[22]*256);
    } else if(mode==0x0a && plSize==22) {  // battery level
      //setCol(20,0x80);
      bat=recv[21];
      modeTxt="B ";
      if(sdebug) snprintf(buf,100,"#%02x %02x %s %02x %03d%% batt   ",cnt,mode,modeTxt,recv[3],recv[21]);
    } else {
      modeTxt="??";
      if(sdebug) snprintf(buf,100,"!!!!!!%02x %02x %s %02x %03d %03d",cnt,mode,modeTxt,recv[3],recv[21],recv[22]);
    }
    if(sdebug) {
      Serial.print(buf);
      snprintf(buf,100,"  [%02x:%02x:%02x:%02x:%02x:%02x] ch%d s=%02d: ",buffer.mac[5],buffer.mac[4],buffer.mac[3],buffer.mac[2],buffer.mac[1],buffer.mac[0],currentChan,plSize);
      Serial.print(buf);
      int n = plSize<=24?plSize:24;
      for(uint8_t i=0; i<n; i++) { snprintf(buf,100,"%02x ",buffer.payload[i]); Serial.print(buf); }
      Serial.println();
    }
  }
  hopChannel();

  //unsigned long tm=millis();
  if(!isTempOK(temp) && isTempOK(tempOld)) temp=tempOld; // bad value, use old one
  if(!isHumOK(hum) && isHumOK(humOld)) hum=humOld; // bad value, use old one

  if(tempOld==temp && humOld==hum) return;
  tempOld=temp; humOld=hum;

  setCol(8,0x40);
  showChar(11, 14, digits3x7);
  if(!isTempOK(temp)) {
    showChar(10, 0, digits3x7);
    showChar(10, 4, digits3x7);
    showChar(10, 10, digits3x7);
  } else {
    if(temp/100) showChar(temp/100, 0, digits3x7);
    showChar((temp%100)/10, 4, digits3x7);
    showChar((temp%10), 10, digits3x7);
  }

  if(!isHumOK(hum)) {
    showChar(10, 21, digits3x7);
    showChar(10, 25, digits3x7);
  } else {
    showChar(hum/100, 21, digits3x7);
    showChar((hum%100)/10, 25, digits3x7);
  }
  showChar(12, 29, digits3x7);
  refreshAll();
  //Serial.println(millis()-tm);
}

// =======================================================================



