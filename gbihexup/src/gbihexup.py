'''
Version History:
Created on Nov 13, 2014

@author: tgack

Description:
    This module us used to upload firmware images to the GNU-BIO arduino
    boards using Atmel's STK-500 protocol. 
    
    Uploading firmware to module extensions is accomplished by providing
    an address, which is used to directly address a slave by its I2C address.  

    This module expects one positional command line argument, the firmware
    image file name. All other command line parameters are switches which
    may or may not require additional parameters.
'''
from com.gnubio.gbihexup.hostaction import HostAction

'''
    --- System Imports Section --
'''
import argparse

'''
    --- Local Imports Section ---
'''
from com.gnubio.gbihexup.errorcodes import ERROR_CODE

import com.gnubio.gbihexup.hostaction




'''

    gpihexup - main entry point

'''
if __name__ == "__main__":

    '''
        
    '''
    exitCode = ERROR_CODE.SUCCESS
    
    hostAction = HostAction()
    
    
    '''
        Create the command line parser
    '''     
    parser = argparse.ArgumentParser(description = "gnubio Arduino / Atmel firmware upload utility")
    parser.add_argument("-s", "--scan", action="store_true", help="print a list of available serial ports")
    parser.add_argument("-p", "--port", default="/dev/ttyACM0", help="serial port name")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="serial port communication speed")
    
    args = parser.parse_args()

    if args.scan:
        print(hostAction.serial_port_list())
        exit (exitCode)
        
        
        
   
    


        
    exit(exitCode)
    
    
