'''
Version History:
Created on Nov 13, 2014

@author: tgack

@summary: 
    This module us used to upload firmware images to the GNU-BIO arduino
    boards using Atmel's STK-500 protocol. 
    
    Uploading firmware to module extensions is accomplished by providing
    an address, which is used to directly address a slave by its I2C address.  

    This module expects one positional command each_line argument, the firmware
    image file name. All other command each_line parameters are switches which
    may or may not require additional parameters.
    
    Command Line Parameters:
    
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
        Serial port /dev/ttyACM0 is opened at 115200 bps and extra debugging information
        is printed. 
    
'''

#
#    Import modules 
#
import argparse
#import time

from com.gnubio.gbihexup.errorcodes import ERROR_CODE
from com.gnubio.gbihexup.hostaction import HostAction



def build_cli_arguments():
    
    '''
        Create and initialize the command each_line parser
    '''     
    parser = argparse.ArgumentParser(description = "gnubio Arduino / Atmel firmware upload utility")
    parser.add_argument("-s", "--scan", action="store_true", help="print a list of available serial ports")
    parser.add_argument("-p", "--port", default="/dev/ttyACM0", help="serial port name")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="serial port communication speed")
    parser.add_argument("-f", "--file", default="a.hex", help="firmware image file name")
    parser.add_argument("-t", "--target", type=int, default=1, help="target address")
    parser.add_argument("-u", "--uc", type=int, default=0, help="0=Atmel, 1=STM32")
    parser.add_argument("-v", "--verbose", action="store_true", help="print extra information")

    return(parser)


def print_extra_output(verbose, out_string):
    
    """
         Extra output print helper
    """
    if True == verbose:
        print(out_string)
    

'''

    gpihexup - main entry point

'''
if __name__ == "__main__":

    '''
        Setup some default values
    '''
    exitCode = ERROR_CODE.SUCCESS
    currentSerialPort = "/dev/ttyACM0"
    currentBaudRate = 115200
    lineCount = 0
     
    '''
        Instantiate the HostAction class
    '''
    hostAction = HostAction()
    
    cli_parser = build_cli_arguments()
    
    args = cli_parser.parse_args()
    
    verboseOutput = args.verbose
	
    ucType = args.uc
    

    if args.scan:
        print(hostAction.serial_port_list())
        exit (exitCode)
        
    #
    # Override the default serial port path and baud rate with 
    # cli parameters.
    #
    currentSerialPort = args.port
    currentBaudRate = args.baud
    
    
    #
    # Recover the target data
    #
    imageFileName = args.file
    targetAddress = args.target
    
    #
    # If the serial port opens, sign on and upload the firmware image
    #
    #
    if ERROR_CODE.SUCCESS == hostAction.open_serial(currentSerialPort, currentBaudRate):
        
        content = []
        
        print("Serial port {0} opened at {1} baud.".format(currentSerialPort, currentBaudRate))
        
        # Try to set the firmware image target address, 
        # if successful, read the image into a list of strings
        # and send the list
        print_extra_output(verboseOutput, "Erasing application flash for target address {0}".format(targetAddress))
        if True == hostAction.set_target_address(targetAddress):
            
            print_extra_output(verboseOutput, "Updating firmware for target address {0}".format(targetAddress))

            
            with open(imageFileName) as f:
                content = f.readlines()
                
            print_extra_output(verboseOutput, "Sending {0} lines".format(len(content)))
            
            
            exitCode = ERROR_CODE.SUCCESS # Hope for the best
            
            # Transmit the firmware image
            if ucType==0:
                for each_line in content:
                    if lineCount == 0:
                        '''
                            Save the first line in the file until the end. That way, the boot loader
                            will not lauch the application if something goes wrong during normal programming
                        '''
                        first_line = each_line
						
                    lineCount = lineCount + 1
					
                    '''
                        Don't write the first record until the rest of the application
                        is written
                    '''
                    if lineCount > 1:
                        print_extra_output(verboseOutput, "Line {0}: {1}".format(lineCount, each_line.strip(' \t\n\r')))

                        # ignore any zero length lines.
                        request = each_line.strip(' \t\n\r')
                        if len(request) > 0:
                            result = hostAction.send_image_line(each_line)

                        if False == result:
                            exitCode = ERROR_CODE.INVALID_PARAMETER
                            break;
            else:
                for each_line in content:
						
                    lineCount = lineCount + 1
					
                    '''
                        Don't write the first record until the rest of the application
                        is written
                    '''
                    print_extra_output(verboseOutput, "Line {0}: {1}".format(lineCount, each_line.strip(' \t\n\r')))
                    # ignore any zero length lines.
                    request = each_line.strip(' \t\n\r')

                    if len(request) > 0:
                        result = hostAction.send_image_line(each_line)

                    if False == result:
                        exitCode = ERROR_CODE.INVALID_PARAMETER
                        break;
			
							
							
            if 0 == ucType:				
                '''
				    This is an Atmel type, send the first line of the hex image now
					if the rest of the image was succesfully loaded
				'''
                if exitCode == ERROR_CODE.SUCCESS:
                    '''
                        The rest of the program has been successfully written.
                        Write the first record into the IVT. This will allow
                        the boot loader to launch the application on next reset
                    '''
                    print_extra_output(verboseOutput, "Line 1: {0}".format(first_line.strip(' \t\n\r')))
                    request = first_line.strip(' \t\n\r')
                    if len(request) > 0:
                        result = hostAction.send_image_line(first_line)
                    
            
            if False == result:
                print_extra_output(verboseOutput, "Application upload Failed: {0}".format(hostAction.lastErrorMessage))
                exitCode = ERROR_CODE.INVALID_PARAMETER
            else:
                print_extra_output(verboseOutput, "Application upload Success: Resetting target.")
                # Drive a reset to the target processor
                result = hostAction.reset_target_by_address(targetAddress)
                
                
        else:
            
            print("Unable to set target address")
            exitCode = ERROR_CODE.COMM_ERROR
    else:
        
        exitCode = ERROR_CODE.SERIAL_PORT_OPEN_ERR
        # print("could not open serial port %s: %s" % currentSerialPort, hostAction.lastExceptionMessage)
        print("could not open serial port: {0}".format(hostAction.lastExceptionMessage))
        
        
        
    print("Lines Processed: {0}".format(lineCount))
    print("Exiting with code: {0}".format(exitCode))
    exit(exitCode)
    
    
