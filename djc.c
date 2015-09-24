#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>

#include "usbdrv/usbdrv.h"
#include "descriptors.h"

#define BUFSIZE 256
#define SYSEXBUFSIZE 32
#define HYSTERESIS 6

#define JOG_HYSTERESIS 4
#define CENTER 508 //calibrated
//#define CALIBRATION //enables jogwheel calibration: controller will continously report jog adc results

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

#define MIDI_CW             0x01
#define MIDI_CCW            0x7F
#define MIDI_JOG1           0x10
#define MIDI_JOG2           0x11
#define MIDI_ENCODER        0x12
#define MIDI_ENCODER_BUTTON 0x50

uchar saveButton[5];
uchar saveLed[4];
uchar saveVU[2];
int16_t saveADC[16];
uchar buffer[BUFSIZE];
uchar encoderState;
uchar adcChannel; //0 for ADC0, 1 for ADC1
unsigned short bufferIndex;
unsigned short bufferSendIndex;
uchar useVU;
uint8_t sysExBuffer[SYSEXBUFSIZE];
uint8_t sysExBufferIndex;
uint8_t jogState[2];

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

  //Enable pull-ups
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
  /*for(bufferIndex = 0; bufferIndex < BUFSIZE ; bufferIndex++) //unnecessary, bufferIndex prevents reading uninitialized buffer sections
    buffer[bufferIndex] = 0;*/
  bufferIndex = 0;
  bufferSendIndex = 0;
  adcChannel = 0;
  useVU = 1;
  sysExBufferIndex = 0;

  uchar i;
  for(i=0; i < 16; i++)
    saveADC[i] = 0;
  for(i=0; i < 5; i++)
    saveButton[i] = 0;
  for(i=0; i < 4; i++)
    saveLed[i] = 0;
  for(i=0; i < 2; i++) {
    jogState[i] = 0;
    saveVU[i] = 0;
  }
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
  return 0;
}

usbMsgLen_t usbFunctionSetup(__attribute__((unused)) uchar *setupData)
{
  return 0; //ignore all unknown requests
}

inline void usbMidiInDebug(uint8_t *data, uint8_t len) {
  genericValueChange(0x50, len & 0x7F);
  int8_t n;
  for (n=0; n<len; n++) {
    genericValueChange(0x40+n, data[n] >> 4);
    genericValueChange(0x40+n, data[n] & 0xF);
  }
}

//uses sysExBuffer which was filled by parseSysEx
void sysExAction() {
  usbMidiInDebug(sysExBuffer, sysExBufferIndex);
  if (sysExBuffer[0] == 0xF0 && sysExBuffer[sysExBufferIndex-1] == 0xF7) //scan for valid sysEx
    switch (sysExBuffer[1]) {
      case 0 : //update all led at once
        if (sysExBufferIndex < 8) break;
        useVU = 0; //disable integrated vu mode
        saveLed[0] = sysExBuffer[2] | (sysExBuffer[6] << 7 & 0x80); //8th bit of each byte is in last sysExBuffer
        saveLed[1] = sysExBuffer[3] | (sysExBuffer[6] << 6 & 0x80);
        saveLed[2] = sysExBuffer[4] | (sysExBuffer[6] << 5 & 0x80);
        saveLed[3] = sysExBuffer[5] | (sysExBuffer[6] << 4 & 0x80);
        break;
      case 1 : //update all led at once
        if (sysExBufferIndex < 6) break;
        saveLed[0] = sysExBuffer[2] | (sysExBuffer[4] << 7 & 0x80); //8th bit of each byte is in last sysExBuffer
        saveLed[1] = sysExBuffer[3] | (sysExBuffer[4] << 6 & 0x80);
        break;
      case 2 : //update all led at once
        if (sysExBufferIndex < 5) break;
        useVU = 1; //enable integrated vu mode
        saveVU[0] = sysExBuffer[2]; //no need for 8th bit as VU data is 0-32
        saveVU[1] = sysExBuffer[3];
        break;
    }
  sysExBufferIndex = 0; //reset transmission
}

//data has to be 4 bytes (one usb midi packet)
void parseSysEx(uint8_t* data) {
  switch (data[0]) {
    case 0x04 : //start or continue (3 bytes payload)
      if (sysExBufferIndex > SYSEXBUFSIZE-3) return;
      sysExBuffer[sysExBufferIndex+0] = data[1];
      sysExBuffer[sysExBufferIndex+1] = data[2];
      sysExBuffer[sysExBufferIndex+2] = data[3];
      sysExBufferIndex += 3;
      break;
    case 0x05 : //single byte / end (1 byte payload)
      if (sysExBufferIndex > SYSEXBUFSIZE-1) return;
      sysExBuffer[sysExBufferIndex+0] = data[1];
      sysExBufferIndex += 1;
      sysExAction();
      break;
    case 0x06 : //end (2 bytes payload)
      if (sysExBufferIndex > SYSEXBUFSIZE-2) return;
      sysExBuffer[sysExBufferIndex+0] = data[1];
      sysExBuffer[sysExBufferIndex+1] = data[2];
      sysExBufferIndex += 2;
      sysExAction();
      break;
    case 0x07 : //end (3 bytes payload)
      if (sysExBufferIndex > SYSEXBUFSIZE-3) return;
      sysExBuffer[sysExBufferIndex+0] = data[1];
      sysExBuffer[sysExBufferIndex+1] = data[2];
      sysExBuffer[sysExBufferIndex+2] = data[3];
      sysExBufferIndex += 3;
      sysExAction();
      break;
    case 0x0F : //single byte (1 byte payload)
      if (sysExBufferIndex > SYSEXBUFSIZE-1) return;
      sysExBuffer[sysExBufferIndex+0] = data[1];
      sysExBufferIndex += 1;
      break;
    default : //unknown message
      sysExBufferIndex = 0; //reset transmission
  }
}

void usbFunctionWriteOut(uchar *data, uchar len)
{
  /* data layout (midi10.pdf page 16):
   * data[0] CIN big-endian (0x04, 0x05, 0x06, 0x07, 0x0B)
   * data[1] message type (0x80, 0x90, 0xB0, 0xF0)
   * data[2] channel
   * data[3] value / velocity (7 bit, 0..7F)
   */
  if( len < 4 )
    return;

  //usbMidiInDebug(data, len);

  if( data[0] == 0x09 && data[3] == 0 ) //if velocity is 0, switch off the led even on Note-On
    data[0]=0x08;
  switch( data[0] ) {
    case 0x08 : //note-off
      if( data[2] < 32 ) //we only have 32 leds
        saveLed[data[2]/8] &= ~(1 << (data[2]%8));
      break;
    case 0x09 : //note-on
      if( data[2] < 32 ) //we only have 32 leds
        saveLed[data[2]/8] |= (1 << (data[2]%8));
      break;
    case 0x0B : //control change
      switch( data[2] ) {
        case 0 :
          useVU = 0; //disable integrated vu mode
          if (data[3]>8) return;
          saveLed[2] = ~vuLedMask[8-data[3]];
          break;
        case 1 :
          useVU = 0; //disable integrated vu mode
          if (data[3]>8) return;
          saveLed[3] = vuLedMask[data[3]];
          break;
        case 2 :
          useVU = 1; //enable integrated vu mode
          if (data[3]>32) return;
          saveVU[0] = data[3];
          break;
        case 3 :
          useVU = 1; //enable integrated vu mode
          if (data[3]>32) return;
          saveVU[1] = data[3];
          break;
        default :
          if ((data[2] & 0x60) == 0x60) { //special update for both VU's at once
            useVU = 1; //enable integrated vu mode
            saveVU[1] = (data[2] << 1 & 0x3F) | (data[3] >> 6 & 1);
            saveVU[0] = data[3] & 0x3F;
          }
      }
      break;
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
    case 0x0F: //system exclusive
      if (len >= 4)
        parseSysEx(data);
      if (len >= 8)
        parseSysEx(data+4);
      break;
  }
}

/******************
 * POLL FUNCTIONS *
 ******************/

void pollPlex() //poll and update every (De-)Multiplexer based function
{
  static uint8_t round = 0;
  uint8_t vuState[2] = {saveVU[0],saveVU[1]};
  if (useVU) { //calculate current VU state
    if (vuState[0] > 0) {
      vuState[0]--; //subtract "off" state
      const uint8_t intensity = vuState[0] & 0b11; //now, lowest 2 bits are "pwm" intensity
      const uint8_t currentVuIndex = ((vuState[0] & 0xFC) >> 2) + (intensity >= round ? 1 : 0);
      vuState[0] = ~vuLedMask[8-currentVuIndex];
    }
    if (vuState[1] > 0) {
      vuState[1]--; //subtract "off" state
      const uint8_t intensity = vuState[1] & 0b11; //now, lowest 2 bits are "pwm" intensity
      const uint8_t currentVuIndex = ((vuState[1] & 0xFC) >> 2) + (intensity >= round ? 1 : 0);
      vuState[1] = vuLedMask[currentVuIndex];
    }
  }

  for (uchar address = 0 ; address < 8 ; address++) {
    SETPLEXADR(address); //walk plex address

    //Prepare and start ADC
    ADMUX = adcChannel | (1 << REFS0);
    ADCSRA |= 1 << ADSC;

    //Assign LED state
    if (useVU) {
      PORTB = LEDSTATE(0,address) << 4 | LEDSTATE(1,address) << 5 | (vuState[0] >> address & 1) << 6 | (vuState[1] >> address & 1) << 7;
    } else {
      PORTB = LEDSTATE(0,address) << 4 | LEDSTATE(1,address) << 5 | LEDSTATE(2,address) << 6 | LEDSTATE(3,address) << 7;
    }
    LWENABLE; //everything is set, now the latches shall commit data

    //Check buttons
    for (uint8_t plexnum = 0; plexnum < 5; plexnum++)
      if( BUTTONPLEXSTATE(plexnum) != BUTTONSAVEDSTATE(plexnum,address) ) {
        keyChange(address+8*plexnum+(SHIFT?40:0),BUTTONSAVEDSTATE(plexnum,address) == 0); //Send key change packet
        TOGGLESAVEDSTATE(plexnum,address);
      }

    //Encoder button
    if( ENCODERBUTTON != (encoderState & 1) ) {
      keyChange(MIDI_ENCODER_BUTTON,!(encoderState & 1));
      encoderState ^= 1;
    }

    LWDISABLE; //latches have certainly finished now

    //Finish ADC
    uint8_t index = address+8*adcChannel;
    while (ADCSRA & (1 << ADSC));
    int16_t newvalue = 1023-ADC; //invert result to get correct direction
    if( abs(newvalue - saveADC[index]) > HYSTERESIS ) {
      saveADC[index] = newvalue;
      adcChange(index, newvalue);
      if( index == 2 ) { //only for xfade
        uint16_t temp = abs(newvalue-512)>>1; //center
        if( temp > 255 ) temp = 255;
        temp = (temp*temp)/256;
        OCR2A = temp;
      }
    }
  }

  adcChannel ^= 1;
  if( round == 3 )
    round = 0;
  else
    round++;
}

void pollHarddisk(uint8_t adcChannel1, uint8_t adcChannel2, uint8_t midiChannel, uint8_t* state) {
  //state: 0=value1>value2 1=? 2=value1<value2
  int16_t value1, value2;
  ADMUX = adcChannel1 | (1 << REFS0);
  ADCSRA |= 1 << ADSC;
  while( ADCSRA & (1 << ADSC) );
  value1 = ADC;
#ifdef CALIBRATION
    genericValueChange(0x60, value1 >> 5); //upper 5 bits
    genericValueChange(0x61, value1 & 0x1F); //lower 5 bits
#endif
  value1 -= CENTER; //centered to zero
  ADMUX = adcChannel2 | (1 << REFS0);
  ADCSRA |= 1 << ADSC;
  while( ADCSRA & (1 << ADSC) );
  value2 = ADC;
#ifdef CALIBRATION
    genericValueChange(0x70, value1 >> 5); //upper 5 bits
    genericValueChange(0x71, value1 & 0x1F); //lower 5 bits
#endif
  value2 -= CENTER; //centered to zero

  if (value1 > value2+JOG_HYSTERESIS && *state == 2) {
    if (value1+value2 > 0)
      genericValueChange(midiChannel,MIDI_CCW);
    else
      genericValueChange(midiChannel,MIDI_CW);
    *state = 0;
  } else if (*state == 0 && value1 < value2-JOG_HYSTERESIS) {
    if ((value1+value2) < 0)
      genericValueChange(midiChannel,MIDI_CCW);
    else
      genericValueChange(midiChannel,MIDI_CW);
    *state = 2;
  }
}

ISR( INT1_vect ) {
  if( ENCODERB )
    genericValueChange(0x12, MIDI_CW); //CW
  else
    genericValueChange(0x12, MIDI_CCW); //CCW
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
    _delay_ms(2);
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
    pollHarddisk(2,3,MIDI_JOG1, jogState);
    pollHarddisk(4,5,MIDI_JOG2, jogState+1);
  }

return(0);
}
