cd usbdrv/
avr-gcc -DF_CPU=20000000 -Os -mmcu=atmega644p -c usbdrvasm.S
avr-gcc -DF_CPU=20000000 -Os -mmcu=atmega644p -c oddebug.c
avr-gcc -DF_CPU=20000000 -Os -mmcu=atmega644p -c usbdrv.c
cd ..
avr-gcc -DF_CPU=20000000 -Os -mmcu=atmega644p -c djc.c
avr-gcc -DF_CPU=20000000 -Os -mmcu=atmega644p -Wl -o djc usbdrv/usbdrv.o usbdrv/oddebug.o usbdrv/usbdrvasm.o djc.o
/opt/cross/bin/avr-objcopy -O ihex djc djc.hex
#avrdude -p m8 -c avrusb500 -eU flash:w:amp.hex
