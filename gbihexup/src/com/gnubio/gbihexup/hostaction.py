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
from time import sleep

from errorcodes import ERROR_CODE

class HostAction(object):
    
    def __init__(self):
        self.serialPort = serial.Serial()
        self.lastExceptionMessage = ""
        self.lastErrorMessage = ""
    
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
    
    def open_serial(self, port='/dev/ttyACM0', baud=115200):
        
        """
            Open the serial port and get it ready for communication
        """
        
        rValue = ERROR_CODE.UNKNOWN
        
        try:
                
            self.serialPort = serial.Serial(port, baud, timeout=1)
            
            self.serialPort.write("\r")
            self.serialPort.flush()
            temp = self.serialPort.readlines()
                        
            rValue = ERROR_CODE.SUCCESS
            
        except Exception as msg:
            
            rValue = ERROR_CODE.SERIAL_PORT_OPEN_ERR
            self.lastExceptionMessage = msg
            
            
        return (rValue)
        
        
        
    def set_target_address(self, target):
        """
            Set the target address for firmware update
        """
        
        rValue = False
        
        try:
            self.serialPort.timeout = 5
            retries = ['attempt1', 'attempt2', 'attempt3', 'attempt4', 'attempt5']
            
            #
            # Try up to 5 times to get the brokers attention
            #
            for atempt in retries:
                self.serialPort.write("%{0}\r".format(target))
                self.serialPort.flush()
                response = self.serialPort.readline()
            
                if "OK" in response:
                    rValue = True 
                    break
                else:
                    sleep(1.0)
                
            
        except Exception as ex:
            rValue = ERROR_CODE.INVALID_PARAMETER
            self.lastExceptionMessage = ex
            rValue = False
            
        return rValue
    
    def reset_target_by_address(self, target):
        """
            Set the target address for firmware update
        """
        
        rValue = False
        
        try:
            
            retries = ['attempt1', 'attempt2', 'attempt3', 'attempt4', 'attempt5']
            
            #
            # Make up to 5 attempts before giving up
            #
            for atempt in retries:
                self.serialPort.timeout = 1
                self.serialPort.write("#{0}\r".format(target-1))
                self.serialPort.flush()
                response = self.serialPort.readline()
            
                if "OK" in response:
                    rValue = True
                    break
                else:
                    sleep(1.0)
                    
                 
            
        except Exception as ex:
            self.lastExceptionMessage = ex
            rValue = False
            
        return rValue
    
    def send_image_line(self, line):
        
        """
           Write a firmware image file line to the serial port.
           Do a basic format test and make sure the line isn't
           zero length. 
        """
        rValue = False
        sleep(0.05)
        try:
            
            self.serialPort.timeout = 5
            
            if line.startswith(":"):
                self.serialPort.write("{0}\r".format(line))
                self.serialPort.flush()
                response = self.serialPort.readline()
                
                if "OK" in response:
                    rValue = True
                    self.lastErrorMessage = "SUCCESS"
                else:
                    self.lastErrorMessage = response
            else:
                rValue = False
            
        except Exception as ex:
            #rValue = ERROR_CODE.INVALID_PARAMETER
            self.lastExceptionMessage = ex
            rValue = False
        
        
        return rValue

if __name__ == "__main__":
    
    """
        Test and Debug area.
    """
    errorLevel = ERROR_CODE.SUCCESS
    
    
    hostAction = HostAction()
    print(hostAction.serial_port_list())
    
    exit(errorLevel)
