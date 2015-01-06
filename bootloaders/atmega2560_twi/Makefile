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
# SLAVE_ID=2
SLAVE_ID=3
##SLAVE_ID=4

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
DEBUG=2




#
# Build variables
#
CC=$(TOOLPATH)/avr-gcc
OBJCOPY=$(TOOLPATH)/avr-objcopy
CFLAGS= -x c -funsigned-char -funsigned-bitfields -DDEBUG -O$(OPTIMIZE) -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g$(DEBUG) -Wall -mmcu=$(MCU) -c -std=gnu99
ASFLAGS=-c -mmcu=$(MCU) 
LDFLAGS=-Wl,-Map="$(ODIR)/$(OUTPUT).map" -Wl,--start-group -Wl,--end-group -Wl,--gc-sections -mrelax -Wl,--section-start=.text=$(BOOTLOAD_SECTION) -mmcu=$(MCU)  
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
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "$(ODIR)/$(OUTPUT).elf" "$(ODIR)/$(OUTPUT)_S$(SLAVE_ID).hex"

#
# Link object files into the executable file
#
$(ODIR)/$(OUTPUT).elf: $(OBJ)
	$(CC) -o $(ODIR)/$(OUTPUT).elf $(OBJ) $(LDFLAGS) $(LIBS)
	


.PHONY: clean

#
# Clean does not delete the hex files as this
# system defines the I2C address as a compile
# time define.
#
clean:
	rm -f \
		$(ODIR)/*.o \
		$(ODIR)/*.d \
		$(ODIR)/*.elf \
		$(ODIR)/*.eep \
		$(ODIR)/*.map



