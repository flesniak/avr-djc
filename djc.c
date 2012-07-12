#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>

#include "usbdrv/usbdrv.h"
#include "descriptors.h"

#define BUFSIZE 512
#define HYSTERESIS 6

#define TOGGLEADC             (PORTD ^= (1 << 6))
#define ADCSELECT             (PORTD >> 6 & 1)
#define SETPLEXADR(x)         (PORTC = x & 7)
#define BUTTONPLEXSTATE(x)    (PINC >> (x+3) & 1)
#define BUTTONSAVEDSTATE(p,n) (saveButton[p] >> n & 1)
#define TOGGLESAVEDSTATE(p,n) (saveButton[p] ^= (1 << n))
#define LEDSTATE(p,n)         (saveLED[p] >> n & 1) //returns saved LED status of LED n on multiplexer p
#define ENCODERPORTA          (PIND >> 3 & 1)
#define ENCODERPORTB          (PIND >> 4 & 1)
#define ENCODERBUTTON         (PIND >> 5 & 1)
#define SHIFT                 (PIND & 1)

uchar saveButton[5];
uchar saveLED[4];
int saveADC[16];
uchar buffer[BUFSIZE];
uchar encoderState;
unsigned short bufferIndex;
unsigned short bufferSendIndex;
short debugtime = 0;

/****************
 * INITIALIZERS *
 ****************/

void initHardware()
{
  //Set in(0)/outputs(1)
  DDRA = 0;
  DDRB = 0b11110000; //pb0+1 jog1 pb2+3 jog2 pb4-7 LED outputs
  DDRC = 0b00000111; //pc0-2 multiplexer address, pc3-7 button poll input
  DDRD = 0b11000000; //pd0 shift key pd1+2 usb pd3-5 encoder pd6 adc select pd7 xfade center led

  //Enable all pull-ups
  PORTA = 0x00; //PORTA is ADC - no pullups here!
  PORTB = 0x00;
  PORTC = 0x00; //no pullups on plex outputs (state is defined by plex)
  PORTD = 0x01;

  //Disable pull-up on USB ports
  USB_CFG_IOPORT = ~((1 << USB_CFG_DMINUS_BIT) | (1 << USB_CFG_DPLUS_BIT));

  //Init ADC
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void initUsb()
{
  usbDeviceDisconnect();
  _delay_ms(150);
  usbDeviceConnect();
  usbInit();
}

void initMemory()
{
  wdt_enable(WDTO_1S);

  //Clean data buffers
  for(bufferIndex = 0; bufferIndex < BUFSIZE ; bufferIndex++)
    buffer[bufferIndex] = 0;
  bufferIndex = 0;
  bufferSendIndex = 0;

  uchar i;
  for(i=0; i < 16; i++)
    saveADC[i] = 0;
  for(i=0; i < 5; i++)
    saveButton[i] = 0;
  for(i=0; i < 4; i++)
    saveLED[i] = 0;
}

/********************
 * PACKET PREPARING *
 ********************/

void keyChange(uchar num, bool noteoff)
{
  if( bufferIndex <= BUFSIZE-4 ) {
    buffer[bufferIndex+0] = noteoff ? 0x08 : 0x09;
    buffer[bufferIndex+1] = noteoff ? 0x80 : 0x90;
    buffer[bufferIndex+2] = 0 + num;
    buffer[bufferIndex+3] = noteoff ? 0x00 : 0x7f;
    bufferIndex += 4;
  }
}

void adcChange(uchar chan, int value)
{
  if( bufferIndex <= BUFSIZE-4 ) {
    buffer[bufferIndex+0] = 0x0b;
    buffer[bufferIndex+1] = 0xb0;
    buffer[bufferIndex+2] = 64 + chan;
    buffer[bufferIndex+3] = value >> 3;
    bufferIndex += 4;
  }
}

/****************************
 * USB MANAGEMENT FUNCTIONS *
 ****************************/

uchar usbFunctionDescriptor(usbRequest_t * rq)
{
  if (rq->wValue.bytes[1] == USBDESCR_DEVICE) {
    usbMsgPtr = (uchar *) deviceDescrMIDI;
    return sizeof(deviceDescrMIDI);
  }
  if (rq->wValue.bytes[1] == USBDESCR_CONFIG) {
    usbMsgPtr = (uchar *) configDescrMIDI;
    return sizeof(configDescrMIDI);
  }
}

usbMsgLen_t usbFunctionSetup(uchar *setupData)
{
    return 0; //ignore all unknown requests
}

void usbFunctionWriteOut(uchar *data, uchar len)
{
  if( len < 3 )
    return;
  uchar ledIndex = data[2]-96;
  if( ledIndex > 31 )
    return;
  switch( data[0] ) {
  case 0x08 : saveLED[ledIndex/8] &= ~(1 << (ledIndex%8)); //Note-off
              keyChange(data[2],true);
              break;
  case 0x09 : saveLED[ledIndex/8] |= (1 << (ledIndex%8)); //Note-on
              keyChange(data[2],false);
              break;
  }
}

/******************
 * POLL FUNCTIONS *
 ******************/

void pollPlex() //poll and update every (De-)Multiplexer based function
{
  uchar address, plexnum;
  for(address = 0 ; address < 8 ; address++) { //walk plex address
    SETPLEXADR(address); //our multiplexer is blazing fast, no need to wait...

    //Check buttons
    for(plexnum = 0; plexnum < 5; plexnum++)
      if( BUTTONPLEXSTATE(plexnum) != BUTTONSAVEDSTATE(plexnum,address) ) {
        keyChange((address+1)*(plexnum+1),BUTTONSAVEDSTATE(plexnum,address) == 0); //Send key change packet
        TOGGLESAVEDSTATE(plexnum,address);
      }

    //Assign LED state
    PORTB = 0xf0 & ((LEDSTATE(3,address) << 7)
                  | (LEDSTATE(2,address) << 6)
                  | (LEDSTATE(1,address) << 5)
                  | (LEDSTATE(0,address) << 4));
  }
}

int adc(uchar channel)
{
  ADMUX = (channel & 7) | (1 << REFS0);
  ADCSRA |= 1 << ADSC;
  while( ADCSRA & (1 << ADSC) );
  return ADC;
}

void pollADC() {
  uchar adcIndex;
  uchar offset = ADCSELECT ? 8 : 0;
  for(adcIndex = 0; adcIndex < 8; adcIndex++) {
    int newvalue;
    newvalue = adc(adcIndex);
    if( abs(newvalue - saveADC[adcIndex+offset]) > HYSTERESIS ) {
      saveADC[adcIndex+offset] = newvalue;
      adcChange(adcIndex+offset,newvalue);
    }
  }
}

void pollEncoder() {
  if( ENCODERPORTA ) {
    if( encoderState & 1 ) {
      encoderState |= 1;
      if( ENCODERPORTB )
        keyChange(33,true); //CW
      else
        keyChange(32,true); //CCW
    }
  }
  else
    encoderState &= ~1;

  if( ENCODERBUTTON != (encoderState >> 1 & 1) ) {
    keyChange(34,!(encoderState >> 1 & 1));
    encoderState ^= 1 << 1;
  }
}

/*************
 * MAIN LOOP *
 *************/

int main() {
  initHardware();
  initUsb();
  initMemory();
  sei();
  short flashtime = 0;

  while( 1 ) {
    _delay_ms(10);
    wdt_reset();
    usbPoll();
    if( usbInterruptIsReady() && bufferIndex > 0 ) {
      usbSetInterrupt(buffer+bufferSendIndex, bufferIndex-bufferSendIndex > 8 ? 8 : bufferIndex-bufferSendIndex);
      bufferSendIndex += 8;
      if( bufferSendIndex >= bufferIndex ) {
        bufferIndex = 0;
        bufferSendIndex = 0;
      }
    }

    //First ADC step
    //pollADC();
    //TOGGLEADC;

    //Do something else to give ADC select time to settle
    pollPlex();

    //Second ADC step
    //pollADC();
    //TOGGLEADC;

    flashtime++;
    if( flashtime >= 50 ) {
      uchar index;
      for(index = 0; index < 4; index++)
        if( saveLED[index] )
          saveLED[index] = 0;
        else
          saveLED[index] = 255;
      flashtime = 0;
    }
}

return(0);
}
