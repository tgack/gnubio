gnubio
======

Repository for gnubio source code and project files.

gpihexup - Python CLI for uploading firmware images.

Dependencies
    glob - Used by the serial port scanner when finding available ports.
    pySerial - Serial port interface between the host PC and the Mega2560 device
    argparse - Handles command line parameter parsing
	
======

bootloaders - Source code and firmware image for the TWI bootloader slave processors. 
    atmega2560_twi - Source code and make file
        Slave Processor 1: obj/twi_atmega2560_S2.hex
	Slave Processor 2: obj/twi_atmega2560_S3.hex
    
    atmega328_twi - Source code and make file
       Slave Processor 3: obj/twi_atmega328p_S4.hex

firmware - Source code and firmware image for the TWI bootloader master processor
    load_host_firmware.sh - Simple script for loading the host bootload firmware.
    Note: The master processor is dependent on the Arduino bootloader. Details are 
              outlined in the slave documentation.
    mega2560/gnubio_host_loader - Arduino sketch (source file)
    mega2560/obj/gnubio_host_loader.cpp.hex - Firmware image of host bootload firmware

docs - Text file descriptions for each of the slave procoessor.




    