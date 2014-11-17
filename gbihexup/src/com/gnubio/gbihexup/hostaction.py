'''
Created on Nov 13, 2014

@author: tgack

@attention: 
    The HostAction class is a wrapper for the Atmel STK500 protocol. It
    uses the pySerial module for physical communication to the module
    to be programmed. 
    
    
    
'''
import sys
import glob
import serial

from errorcodes import ERROR_CODE
from stk500 import STK500



class HostAction(STK500):
    
    def __init__(self):
        self.serialPort = serial.Serial()
    
    def serial_port_list(self):
        """
            List all available serial ports available on this host
            
            raise EnvironmentError
                On unsupported or unknown platforms
            
            returns
                A list of available serial ports
                
                Note: Linux and Cygwin platforms ignore terminals
        """    
        
        # Handle the various platforms appropriately
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i+1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startwith('cygwin'):
            # exclude terminals
            ports = glob.glob('/dev/tty[A-Az-z]*')
        elif sys.platform.startwith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        
        # Initialize the return list
        result = []
        
        # iterate the list of ports
        for port in ports:
            try:
                # Test for item to see if the serial port is available. An exception
                # will be thrown if it is not available
                s = serial.Serial(port)
                s.close()
                
                # If the process gets this far, the serial port is available
                # Add it to the list.
                result.append(port)
            except (OSError, serial.SerialException):
                pass
    
        return(result)
    
    def open_serial(self, port, baud):
        
        self.serialPort.port = port
        self.serialPort.baudrate = baud
        self.serialPort.bytesize = 8
        self.serialPort.stopbits = 1
        
        self.serialPort.open()

    def cmd_sign_on(self):
        rValue = False
        # if the serial port is open then continue
        if self.serialPort.isOpen():
            
            data = self.build_cmd_sign_on()
            
            if True == self.build_request_packet(data):
                self.serialPort.write(data)
                
            
                
        return rValue
            

if __name__ == "__main__":
    
    """
        Test and Debug area.
    """
    errorLevel = ERROR_CODE.SUCCESS
    
    
    hostAction = HostAction()
    print(hostAction.serial_port_list())
    
    exit(errorLevel)
