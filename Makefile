SRC = oddebug.c usbdrv.c djc.c
ASMSRC = usbdrvasm.S
CFLAGS =  -DF_CPU=20000000 -Os -mmcu=atmega644p -Wall -Wextra -std=gnu99

OBJ = $(patsubst %.c,%.o,$(SRC))
ASMOBJ = $(patsubst %.S,%.o,$(ASMSRC))
VPATH = usbdrv

AVRDUDE_MCU = m664p
AVRDUDE_PROG = usbasp

HFUSE = 0xd9
LFUSE = 0xff

.PHONY = all clean

all: djc.hex

djc.hex: djc
	avr-objcopy -O ihex $< $@

djc: $(OBJ) $(ASMOBJ)
	avr-gcc $(CFLAGS) $^ -o $@ 

%.o: %.c
	avr-gcc $(CFLAGS) -c $^ -o $@

%.o: %.S
	avr-gcc $(CFLAGS) -c $^ -o $@

flash: djc.hex
	avrdude -p $(AVRDUDE_MCU) -c $(AVRDUDE_PROG) -eU flash:w:$<

fuses:
	avrdude -p $(AVRDUDE_MCU) -c $(AVRDUDE_PROG) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m

clean:
	rm -f *.o $(VPATH)/*.o djc djc.hex
