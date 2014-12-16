#------------------------------------------------------------------------------
# Makefile to compile the GNUBIO bootloader for
# ATMEGA32U4 micro-controller
#
# Written by T. Gack @ Solarfire Technologies, LLC
#
# 
#------------------------------------------------------------------------------


#
# Set a path to the AVR gnu compiler
#
TOOLPATH=$(HOME)/bin/avr8-gnu-toolchain/bin

#
# Set the output module name
#
OUTPUT=twi_atmega32u4


#
# Set the target MCU
#
MCU = atmega32u4


#
# Set the start address (byte address) to
# the start of bootloader memory
#
BOOTLOAD_SECTION=0x7000


#
# Set aside 2 pages of Flash for functions
# that will not fit into the bootloader section
# of memeory
#
EXTENDED_SECTION=0x6E00


ODIR=obj
OPTIMIZE=0
DEBUG=3




#
# Build variables
#
CC=$(TOOLPATH)/avr-gcc
OBJCOPY=$(TOOLPATH)/avr-objcopy
CFLAGS= -funsigned-char -funsigned-bitfields -O$(OPTIMIZE) -ffunction-sections -fdata-sections -g$(DEBUG) -Wall -mmcu=$(MCU) -c -std=gnu99 
ASFLAGS=-c -mmcu=$(MCU) 
LDFLAGS=-Wl,-Map="$(ODIR)/$(OUTPUT).map" -Wl,--start-group -Wl,--end-group -Wl,--gc-sections -Wl,--section-start=.text=$(BOOTLOAD_SECTION) -Wl,--section-start=.boot=0x0 -Wl,--undefined=__boot_vectors -Wl,--section-start=.text_extended=$(EXTENDED_SECTION) -mmcu=$(MCU)  
LIBS=

#
# Object file list
#
_OBJ = main.o twi.o flash_utilities.o boot.o
OBJ = $(patsubst %, $(ODIR)/%,$(_OBJ))

#
# Compile C File
#
$(ODIR)/%.o: %.c
	$(CC) $(CFLAGS) -MD -MP -MF "$(@:%o=%d)" -MT"$(@:%o=%d)" -MT"$(@:%o=%o)" -o $@ $< 

#
# Assemble assembly modules
#
$(ODIR)/%.o: %.S
	$(CC) $(ASFLAGS) -MD -MP -MF "$(@:%o=%d)" -MT"$(@:%o=%d)" -MT"$(@:%o=%o)"  -o $@ $< 
	
#
# Generate the intel hex programming file
#
$(ODIR)/$(OUTPUT).hex: $(ODIR)/$(OUTPUT).elf
	rm -f $(ODIR)/$(OUTPUT).hex
	$(OBJCOPY) -O ihex -j .text -j .text_extended -j .data -j .boot --set-section-flags=.boot=alloc,load "$(ODIR)/$(OUTPUT).elf" "$(ODIR)/$(OUTPUT).hex"

#
# Link object files into the executable file
#
$(ODIR)/$(OUTPUT).elf: $(OBJ)
	$(CC) -o $(ODIR)/$(OUTPUT).elf $(OBJ) $(LDFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f \
		$(ODIR)/*.o \
		$(ODIR)/*.d \
		$(ODIR)/*.elf \
		$(ODIR)/*.hex \
		$(ODIR)/*.eep \
		$(ODIR)/*.map




