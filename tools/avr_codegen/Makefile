
all : test.hex test.lst
PART=attiny85
PROGPART=t85

CFLAGS=-g -Wall -Os -mmcu=$(PART) -DF_CPU=8000000UL
ASFLAGS:=$(CFLAGS)
CC=avr-gcc


test.hex : test.elf
	avr-objcopy -j .text -j .data -O ihex test.elf test.hex 
	avrdude -c usbtiny -p $(PROGPART) -V -U flash:w:test.hex

test.elf : test.c
	avr-gcc -I  -g $(CFLAGS)   -mmcu=$(PART) -Wl,-Map,test.map -o $@ test.c -L/usr/lib/binutils/avr/2.18

test.lst : test.c
	avr-gcc -c -g -Wa,-a,-ad $(CFLAGS) test.c > $@

burn : test.hex
	avrdude -c usbtiny -p $(PROGPART) -V -U flash:w:test.hex

#For PLL
burnfuses :
	avrdude -c usbtiny -p $(PROGPART) -U lfuse:w:0x41:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 

burnfuses_legacy_timer_1 :
	avrdude -c usbtiny -p $(PROGPART) -U lfuse:w:0x53:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 

burnfuses_default :
	avrdude -c usbtiny -p $(PROGPART) -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 

readfuses :
	avrdude -c usbtiny -p $(PROGPART) -U hfuse:r:high.txt:b -U lfuse:r:low.txt:b

clean : 
	rm -f *~ high.txt low.txt test.hex test.map test.elf *.o sendpack.S makesendpack
