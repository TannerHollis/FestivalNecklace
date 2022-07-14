MCU=attiny45
FUSE_L=0x62
FUSE_H=0xDF
FUSE_E=0xFF
F_CPU=8000000
CC=avr-gcc
LD=avr-ld
OBJCOPY=avr-objcopy
SIZE=avr-size
AVRDUDE=avrdude
CFLAGS= -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
TARGET=main

SRCS = main.c light_ws2812.c

all:
	${CC} ${CFLAGS} -o ${TARGET}.o ${SRCS}
	${LD} -o ${TARGET}.elf ${TARGET}.o
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.o ${TARGET}.hex
	${SIZE} -C --mcu=${MCU} ${TARGET}.elf

flash:
	${AVRDUDE} -p ${MCU} -c usbasp -B10 -U flash:w:${TARGET}.hex:i -F -P usb

fuse:
	$(AVRDUDE) -p ${MCU} -c usbasp -B10 -U hfuse:w:${FUSE_H}:m -U lfuse:w:${FUSE_L}:m -U efuse:w:${FUSE_E}:m

clean:
	rm -f *.c~ *.o *.elf *.hex