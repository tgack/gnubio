'''
Created on Nov 17, 2014

@author: tgack
'''
from errorcodes import ERROR_CODE
from stk500Types import STK500_RX_STATE
from stk500Types import STK500_COMMANDS
class STK500:
    def __init__(self):
        self.packetBuffer = []
        self.requestSequenceNumber = 0
        self.responseSequenceNumber = 0
        self.responseDataLength = 0
        self.responsePacketLenght = 0
        self.timeout = 1000
        self.token = 0x0E
        self.messageStart = 0x1B
        self.maxMessageSize = 275
        self.lastError = ERROR_CODE.SUCCESS
        self.receiveState = STK500_RX_STATE.WAIT_START
        self.packetTimer = 0
        self.checksum = 0
        
        
    def build_cmd_sign_on(self):
        return (bytearray([STK500_COMMANDS.CMD_SIGN_ON]))   
         
    def build_request_packet(self, payload):
        '''
            Build the STK500 request packet from
            the pay load data and command
            
            @param payload: This must be a zero length or greater bytearray object
            
            @return: Success: bytearray containing the request packet
                    Error: Zero length byte array
        '''
        
        request = bytearray()
        
        messageSize = payload.length
        if messageSize > self.maxMessageSize:
            self.lastError = ERROR_CODE.STK500_INVALID_LENGTH
            return request
        
        
        # Increment the sequence number
        self.requestSequenceNumber = self.requestSequenceNumber + 1
        
        # Build the request packet
        request.append(self.messageStart)
        request.append(self.requestSequenceNumber)
        request.append( (messageSize >> 8) & 0x00FF)
        request.append( (messageSize & 0xFF))
        request.append(self.token)
        request.append(payload)
        
        # Calculate the checksum
        checksum = 0
        for value in request:
            checksum = checksum ^ value
            
        request.append(checksum & 0x00FF)
        
        return request
    
    
    def parse_response_stream(self, byteValue):
        
        """
            Parse an incoming stream of characters, attempt to rebuild
            a STK500 response packet.
            
            @param byteValue: next character in the response stream
            
            @return: True = Packet Received, False = Error receiving packet
            
        """
        
        rValue = False
        
        if STK500_RX_STATE.WAIT_START == self.receiveState:
            #
            # Wait for start character
            #
            if byteValue == self.messageStart:
                self.packetBuffer = bytearray()
                self.packetTimer = 0
                self.checksum = byteValue
                self.responsePacketLenght = 0
                self.receiveState = STK500_RX_STATE.WAIT_SEQUENCE_NUMBER
                
        elif STK500_RX_STATE.WAIT_SEQUENCE_NUMBER == self.receiveState:
            #
            # Test the sequence number, if it matches the request sequence
            # number then proceed with packet reception, else
            # go back to start state and wait for the next start
            # sequence to be received
            #
            self.responseSequenceNumber = byteValue
            if self.responseSequenceNumber == self.requestSequenceNumber:
                self.checksum = self.checksum ^ byteValue
                self.receiveState = STK500_RX_STATE.WAIT_MSG_SIZE_MSB
            else:
                self.receiveState = STK500_RX_STATE.WAIT_START
                
        elif STK500_RX_STATE.WAIT_MSG_SIZE_MSB == self.receiveState:
            #
            # Capture the MSB of the message length
            #
            self.responseDataLength = (byteValue << 8)
            self.checksum = self.checksum ^ byteValue
            self.receiveState = STK500_RX_STATE.WAIT_MSG_SIZE_LSB
            
        elif STK500_RX_STATE.WAIT_MSG_SIZE_LSB == self.receiveState:
            #
            # Capture the LSB of the message length
            #
            self.responseDataLength = self.responseDataLength + byteValue
            self.checksum = self.checksum ^ byteValue
            self.receiveState = STK500_RX_STATE.WAIT_TOKEN
        elif STK500_RX_STATE.WAIT_TOKEN == self.receiveState:
            #
            # Test the token value, if it's correct, continue
            # receiving the packet, else return to start state.
            #
            if byteValue == self.token:
                self.receiveState = STK500_RX_STATE.WAIT_DATA
                self.checksum = self.checksum ^ byteValue
            else:
                self.receiveState = STK500_RX_STATE.WAIT_START
        elif STK500_RX_STATE.WAIT_DATA == self.receiveState:
            if self.responsePacketLenght < self.responseDataLength:
                self.packetBuffer.append(byteValue)
                self.checksum = self.checksum ^ byteValue
            else:
                self.receiveState = STK500_RX_STATE.WAIT_CHECKSUM
        elif STK500_RX_STATE.WAIT_CHECKSUM == self.receiveState:
            if self.checksum == byteValue:
                rValue = True
            else:
                rValue = False
                
            self.receiveState = STK500_RX_STATE.WAIT_START
        else:
            self.receiveState = STK500_RX_STATE.WAIT_START
            
        return(rValue)
        
    
    
    