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
ISPPATH=$(HOME)/bin/AVR/bin

#
# Set the output module name
#
OUTPUT=twi_atmega2560

#
# Select the appropriate slave ID
#
SLAVE_ID=2
#SLAVE_ID=3
#SLAVE_ID=4

#
# Set the target MCU
#
MCU = atmega2560


#
# Set the start address (byte address) to
# the start of bootloader memory
#
BOOTLOAD_SECTION=0x3E000


#
#  Setup flash programming variables
#
# AVRDUDE = $(ISPPATH)/avrdude
# PROGRAMMER_TYPE = jtag3isp
# PROGRAMMER_ARGS = 



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
LDFLAGS=-Wl,-Map="$(ODIR)/$(OUTPUT).map" -Wl,--start-group -Wl,--end-group -Wl,--gc-sections -Wl,--section-start=.text=$(BOOTLOAD_SECTION) -Wl,--section-start=.boot=0x0 -Wl,--undefined=__boot_vectors -mmcu=$(MCU)  
LIBS=

#
# Object file list
#
_OBJ = twi_mega2560.o twi.o flash_utilities.o
OBJ = $(patsubst %, $(ODIR)/%,$(_OBJ))

#
# Compile C File
#
$(ODIR)/%.o: %.c
	$(CC) $(CFLAGS) -DTWIID=$(SLAVE_ID) -MD -MP -MF "$(@:%o=%d)" -MT"$(@:%o=%d)" -MT"$(@:%o=%o)" -o $@ $< 

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
	$(OBJCOPY) -O ihex -j .text -j .text_extended -j .data -j .boot --set-section-flags=.boot=alloc,load "$(ODIR)/$(OUTPUT).elf" "$(ODIR)/$(OUTPUT)_S$(SLAVE_ID).hex"

#
# Link object files into the executable file
#
$(ODIR)/$(OUTPUT).elf: $(OBJ)
	$(CC) -o $(ODIR)/$(OUTPUT).elf $(OBJ) $(LDFLAGS) $(LIBS)
	

#flash: $(ODIR)/$(OUTPUT).hex
#	$(AVRDUDE) -c $(PROGRAMMER_TYPE) -p $(MCU) $(PROGRAMMER_ARGS) -U flash:w:"$(ODIR)/$(OUTPUT)_S$(SLAVE_ID).hex"

#program: flash

.PHONY: clean

clean:
	rm -f \
		$(ODIR)/*.o \
		$(ODIR)/*.d \
		$(ODIR)/*.elf \
		$(ODIR)/*.eep \
		$(ODIR)/*.map

#		$(ODIR)/*.hex \



