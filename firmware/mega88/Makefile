# Name: Makefile
# Project: USB-MIDI-DMX
# Author: Takashi Toyoshima
# Creation Date: 2014-01-13
# Copyright: (c) 2014 by Takashi Toyoshima
# License: GNU GPL v2 (see License.txt)

DEVICE  = atmega88
F_CPU   = 10000000 # in Hz
FUSE_L  = 0xF2
FUSE_H  = 0xDF
FUSE_X  = 0xF9
HIDSPX  = hidspx -d2

CFLAGS  = -I.
OBJECTS = main.o

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

# symbolic targets:
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make hex ........ build main.hex"
	@echo "make program .... flash fuses and firmware"
	@echo "make fuse ....... flash the fuses"
	@echo "make flash ...... flash the firmware (use this on metaboard)"
	@echo "make clean ...... delete objects and hex file"

hex: main.hex

program: flash fuse

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
		{ echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(HIDSPX) -fL$(FUSE_L) -fH$(FUSE_H) -fX$(FUSE_X)

# rule for uploading firmware:
flash: main.hex
	$(HIDSPX) main.hex

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.elf *.o *~

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

# Since we don't want to ship the driver multipe times, we copy it into this project:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size main.hex

# debugging targets:

disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
