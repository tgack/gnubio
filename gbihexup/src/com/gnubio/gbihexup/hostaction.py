'''
Created on Nov 13, 2014

@author: tgack
'''
import sys
import glob
import serial

from errorcodes import ERROR_CODE

class HostAction:
    
    def serial_port_list(self):
        """
            List serial porrts
            
            rasises EnvironmentError
                On usupported or unknown platforms
            
            returns
                A list of availbale serial ports
        """    
        
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i+1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startwith('cygwin'):
            """ 
                exclude terminals
            """
            ports = glob.glob('/dev/tty[A-Az-z]*')
        elif sys.platform.startwith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        
        result = []
        
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
    
        return(result)



if __name__ == "__main__":
    
    errorLevel = ERROR_CODE.SUCCESS
    
    exit(errorLevel)

    