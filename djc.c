#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>

#include "usbdrv/usbdrv.h"
#include "descriptors.h"

#define BUFSIZE 1024
#define HYSTERESIS 6

#define LWENABLE              (PORTD &= ~(1 << 6)) //former TOGGLEADC, now LatchWriteEnable
#define LWDISABLE             (PORTD |= (1 << 6)) //former TOGGLEADC, now LatchWriteDisable
#define SETPLEXADR(x)         (PORTC = x & 7)
#define BUTTONPLEXSTATE(x)    (PINC >> (x+3) & 1) //returns the state of the button currently addressed on multiplexer x
#define BUTTONSAVEDSTATE(p,n) (saveButton[p] >> n & 1) //returns the saved state of button n on multiplexer p
#define TOGGLESAVEDSTATE(p,n) (saveButton[p] ^= (1 << n))
#define LEDSTATE(p,n)         (saveLed[p] >> n & 1) //returns saved LED status of LED n on multiplexer p
#define ENCODERA              (PIND >> 3 & 1)
#define ENCODERB              (PIND >> 4 & 1)
#define ENCODERBUTTON         (PIND >> 5 & 1)
#define SHIFT                !(PIND & 1) //negative logic

uchar saveButton[5];
uchar saveLed[4];
int saveADC[16];
uchar buffer[BUFSIZE];
uchar encoderState;
uchar adcChannel; //0 for ADC0, 1 for ADC1
unsigned short bufferIndex;
unsigned short bufferSendIndex;

//masks my vumeter-leds     0     1     2     3     4     5     6     7     8
const uchar vuLedMask[] = { 0, 0x80, 0xA0, 0xA8, 0xAA, 0xAB, 0xAF, 0xBF, 0xFF };

/****************
 * INITIALIZERS *
 ****************/

void initHardware()
{
  //Set in(0)/outputs(1)
  DDRA = 0b00000000; //only inputs on adc for now
  DDRB = 0b11110000; //pb0+1 jog1 pb2+3 jog2 pb4-7 LED outputs
  DDRC = 0b00000111; //pc0-2 multiplexer address, pc3-7 button poll input
  DDRD = 0b11000000; //pd0 shift key pd1+2 usb pd3-5 encoder pd6 led lash enable pd7 xfade center led

  //Enable all pull-ups
  PORTA = 0x00; //PORTA is ADC - no pullups here!
  PORTB = 0x00;
  PORTC = 0x00; //no pullups on plex outputs (state is defined by plex)
  PORTD = 0x71;

  //Disable pull-up on USB ports
  USB_CFG_IOPORT &= ~((1 << USB_CFG_DMINUS_BIT) | (1 << USB_CFG_DPLUS_BIT));

  //Init ADC
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  //Interrupt INT1 for encoder
  EICRA = (1 << ISC11) | (0 << ISC10); //Interrupt INT1 on falling edge
  EIMSK = (1 << INT1); //enable INT1

  //PWM for xfade led
  TCCR2A = 0b10000001; //phase-correct PWM with output compare match bit clearing
  TCCR2B = 0b00000110; //prescaler 256 + 3rd PWM mode bit WGM22
  OCR2A = 0;

  //Disable LED write
  LWDISABLE;
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
  adcChannel = 0;

  uchar i;
  for(i=0; i < 16; i++)
    saveADC[i] = 0;
  for(i=0; i < 5; i++)
    saveButton[i] = 0;
  for(i=0; i < 4; i++)
    saveLed[i] = 0;
}

/********************
 * PACKET PREPARING *
 ********************/

/* MIDI OUTGOING (MixxxCtl -> PC)
 *   Note-On (0x09) / Note-Off (0x08) BUTTONS
 *      0-39 = 0x00-0x27 buttons
 *     40-79 = 0x28-0x4F buttons with shift pressed
 *     80-82 = 0x50-0x52 encoder ccw/cw/button (cw(w) Note-On only)
 *   System Control (0x0B) FADERS+POTS
 *      0-15 = 0x00-0x0F Faders/Pots
 *     16-19 = 0x10-0x13 Jog-Wheels
 *
 * MIDI INCOMING (PC -> MixxxCtl)
 *   Note-On (0x09) / Note-Off (0x08) INPUT -> LEDS
 *     0-31 = 0x60-0x7F LEDs
 *   System Control (0x0B) VU-LEDs at once
 *     0 = 0x80 set VU-LEDs at once (9*VUBstate+VUAstate)
 */

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
    buffer[bufferIndex+2] = 0 + chan;
    buffer[bufferIndex+3] = value >> 3;
    bufferIndex += 4;
  }
}

void genericValueChange(uchar chan, uchar value)
{
  if( bufferIndex <= BUFSIZE-4 ) {
    buffer[bufferIndex+0] = 0x0b;
    buffer[bufferIndex+1] = 0xb0;
    buffer[bufferIndex+2] = 0 + chan;
    buffer[bufferIndex+3] = value;
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
  if( data[0] == 0x09 && data[3] == 0 ) //if velocity is 0, switch off the led even on Note-On
    data[0]=0x08;
  switch( data[0] ) {
  case 0x08 : if( data[2] < 32 ) //we only have 32 leds
                saveLed[data[2]/8] &= ~(1 << (data[2]%8)); //Note-off
              break;
  case 0x09 : if( data[2] < 32 ) //we only have 32 leds
                saveLed[data[2]/8] |= (1 << (data[2]%8)); //Note-on
              break;
  case 0x0B : switch( data[2] ) {
                case 0 : saveLed[2] = ~vuLedMask[8-data[3]%9];  //handle vu-leds-at-once
                         saveLed[3] = vuLedMask[data[3]/9];
                         break;
                case 1 : saveLed[0] = data[3]; //change all leds at once
                         saveLed[1] = data[3];
                         saveLed[2] = data[3];
                         saveLed[3] = data[3];
                         break;
                case 2 : saveLed[2] = data[3];
                         break;
                case 3 : saveLed[3] = data[3];
                         break;
              }
              break;
  }
}

/******************
 * POLL FUNCTIONS *
 ******************/

void pollPlex() //poll and update every (De-)Multiplexer based function
{
  static uchar round = 0;
  uchar address, plexnum;
  for(address = 0 ; address < 8 ; address++) {
    SETPLEXADR(address); //walk plex address

    //Prepare and start ADC
    ADMUX = adcChannel | (1 << REFS0);
    ADCSRA |= 1 << ADSC;

    //Assign LED state
    PORTB = LEDSTATE(0,address) << 4 | LEDSTATE(1,address) << 5; //set leds for multiplexer 0 & 1
    //VU states range from 0-8. saveLed may be from 0 up to 4*8, states with mod4!=0 will be on sometimes
    unsigned char currentVuIndex = saveLed[2]/4+(saveLed[2]%4 < round ? 1 : 0);
    PORTB += (vuLedMask[currentVuIndex] >> address & 1) << 6;
//     PORTB += 6*(saveLed[address>3?3:2] < round ? 1 : 0);
//     PORTB += 7*(saveLed[address>3?3:2] < round ? 1 : 0);
    /*PORTB = 0xf0 & ((LEDSTATE(3,address) << 7)
                  | (LEDSTATE(2,address) << 6)
                  | (LEDSTATE(1,address) << 5)
                  | (LEDSTATE(0,address) << 4));*/
    LWENABLE; //everything is set, now the latches shall commit data

    //Check buttons
    for(plexnum = 0; plexnum < 5; plexnum++)
      if( BUTTONPLEXSTATE(plexnum) != BUTTONSAVEDSTATE(plexnum,address) ) {
        keyChange(address+8*plexnum+(SHIFT?40:0),BUTTONSAVEDSTATE(plexnum,address) == 0); //Send key change packet
        TOGGLESAVEDSTATE(plexnum,address);
      }

    //Encoder button
    if( ENCODERBUTTON != (encoderState & 1) ) {
      keyChange(82,!(encoderState & 1));
      encoderState ^= 1;
    }

    LWDISABLE; //latches have certainly finished now

    //Finish ADC
    int32_t newvalue;
    uchar index = address+8*adcChannel;
    while( ADCSRA & (1 << ADSC) );
    newvalue = 1023-ADC; //invert result to get correct direction
    if( abs(newvalue - saveADC[index]) > HYSTERESIS ) {
      saveADC[index] = newvalue;
      adcChange(index,newvalue);
      if( index == 2 ) { //only for xfade
        newvalue = newvalue-512;
        if( abs(newvalue) < HYSTERESIS ) //completely disables led on centered xfade
          newvalue = 0;
        else
          newvalue = (newvalue*newvalue)/1024;
        if( newvalue > 255 ) newvalue = 255;
        if( newvalue < 0 ) newvalue = 0;
        OCR2A = newvalue;
      }
    }
  }
  adcChannel ^= 1;
  if( round == 3 )
    round = 0;
  else
    round++;
}

#define JOG_HYSTERESIS 4
#define JOG_MIDI_CW  65
#define JOG_MIDI_CCW 63

void pollHarddisk() {
  //static uchar state = 1; //0=lower half-wave 1=zero 2=upper half-wave
  static uchar state = 0; //0=value1>value2 1=? 2=value1<value2
  
  int32_t value1, value2;
  ADMUX = 2 | (1 << REFS0);
  ADCSRA |= 1 << ADSC;
  while( ADCSRA & (1 << ADSC) );
  value1 = ADC;
  value1 -= 512; //centered to zero
  ADMUX = 3 | (1 << REFS0);
  ADCSRA |= 1 << ADSC;
  while( ADCSRA & (1 << ADSC) );
  value2 = ADC;
  value2 -= 512; //centered to zero

  //upper half-wave
  /*if( value1 > JOG_HYSTERESIS ) {
    if( state == 0 ) { //lower half-wave has already been here
      state = 1;
      if( value1 > value2 )
        genericValueChange(0x10,JOG_MIDI_CW);
      else
        genericValueChange(0x10,JOG_MIDI_CCW);
    } else
      state = 2;
  }
  //lower half-wave
  if( value1 < -1*JOG_HYSTERESIS ) {
    if( state == 2 ) { //upper half-wave has already been here
      state = 1;
      if( value1 < value2 )
        genericValueChange(0x10,JOG_MIDI_CW);
      else
        genericValueChange(0x10,JOG_MIDI_CCW);
    } else
      state = 0;
  }*/

  if( value1 > value2+JOG_HYSTERESIS && state == 2 ) {
    if( (value1+value2) > 0 )
      genericValueChange(0x10,JOG_MIDI_CCW);
    else
      genericValueChange(0x10,JOG_MIDI_CW);
    state = 0;
  } else if ( state == 0 && value1 < value2-JOG_HYSTERESIS ) {
    if( (value1+value2) < 0 )
      genericValueChange(0x10,JOG_MIDI_CCW);
    else
      genericValueChange(0x10,JOG_MIDI_CW);
    state = 2;
  }
}

ISR( INT1_vect ) {
  if( ENCODERB )
    keyChange(80,false); //CW
  else
    keyChange(81,false); //CCW
}

/*************
 * MAIN LOOP *
 *************/

int main() {
  initHardware();
  initUsb();
  initMemory();
  sei();

  while( 1 ) {
    _delay_ms(5);
    wdt_reset();
    usbPoll();

    //If possible, send out data from our buffer
    if( usbInterruptIsReady() && bufferIndex > 0 ) {
      usbSetInterrupt(buffer+bufferSendIndex, bufferIndex-bufferSendIndex > 8 ? 8 : bufferIndex-bufferSendIndex);
      bufferSendIndex += 8;
      if( bufferSendIndex >= bufferIndex ) {
        bufferIndex = 0;
        bufferSendIndex = 0;
      }
    }

    pollPlex();
    pollHarddisk();
  }

return(0);
}
