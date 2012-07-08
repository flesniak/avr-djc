#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define BLINKDELAY 250

int main()
{
DDRC = 0xff;
DDRA = 0xff;

PORTC = 0xff;

while(1) {
  _delay_ms(BLINKDELAY);
  PORTA = ~PORTA;
}

return(0);
}
