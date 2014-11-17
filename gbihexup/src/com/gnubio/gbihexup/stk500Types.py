'''
Created on Nov 17, 2014

@author: tgack
'''

class STK500_RX_STATE(object):
    WAIT_START = 1
    WAIT_SEQUENCE_NUMBER = 2
    WAIT_MSG_SIZE_MSB = 3
    WAIT_MSG_SIZE_LSB = 4
    WAIT_TOKEN = 5
    WAIT_DATA = 6
    WAIT_CHECKSUM = 7
    
    
    
    