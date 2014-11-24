gnubio
======

Repository for gnubio source code and project files.

Dependencies
	glob - Used by the serial port scanner when finding available ports.
	pySerial - Serial port interface between the host PC and the Mega2560 device
	argparse - Handles command line parameter parsing
	

Sub-projects

	gpihexup - Python CLI for uploading firmware images.

	    usage: gbihexup.py [-h] [-s] [-p PORT] [-b BAUD] [-f FILE] [-t TARGET] [-v]
	    
	    gnubio Arduino / Atmel firmware upload utility
	    
	    optional arguments:
	      -h, --help            show this help message and exit
	      -s, --scan            print a list of available serial ports
	      -p PORT, --port PORT  serial port name
	      -b BAUD, --baud BAUD  serial port communication speed
	      -f FILE, --file FILE  firmware image file name
	      -t TARGET, --target TARGET
				    target address
	      -v, --verbose         print extra information

	    
	    Example:
		gbihexup --baud=115200 --port=/dev/ttyACM0 --file=fw_image.hex --target=1 -v
	    
		Updates device at i2c address1 using the fw_image.hex file (in the cwd).
		Serial port /dev/ttyACM0 is opend at 115200 bps and extra debugging information
		is printed. 

AVR Toochain
	Atmel AVR 8-bit Toolchain 3.4.5 - Linux 64-bit
	avrdude: Version 6.1, compiled from source on 11-24-2014
	JTAG: AVR JTAGICE3 (03eb:2140)
	
	Host OS: Ubuntu 14.04 LTS (64-bit)
	
