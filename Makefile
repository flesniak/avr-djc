SRC = oddebug.c usbdrv.c djc.c
ASMSRC = usbdrvasm.S
CFLAGS =  -DF_CPU=20000000 -Os -mmcu=atmega644p

OBJ = $(patsubst %.c,%.o,$(SRC))
ASMOBJ = $(patsubst %.S,%.o,$(ASMSRC))
VPATH = usbdrv

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
	avrdude -p m644p -c usbasp -eU flash:w:djc.hex

clean:
	rm -f *.o $(VPATH)/*.o djc djc.hex
