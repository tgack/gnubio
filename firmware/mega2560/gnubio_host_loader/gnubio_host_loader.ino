
/*
 * GNUBIO - Arduino Mega2560 Host Loader program
 *
 *  This module functions as a Serial to I2C converter and router.
 *
 *  Setting the target address: 
 *    Send a [\r] terminated string starting with a percent symbol "%" followed
 *    byte the target address. Note addresses are always decimal.
 *
 *    Example: Set target address 10
 *        %10[\r]
 *
 * Sending firmware image data:
 *    Firmware image data must be sent in the intel hex format. Line by line. This 
 *    sketch will processes each line as it is received. Following setting of a new
 *    target address, the remote unit will be erased when the first line of intel
 *    hex data is received.
 *
 *  Sending line ends [\n] is acceptable, this character is simply ignored in all cases.
 *
 */
#include <Wire.h>


/*
 * States used in the receive state machine
 */
#define	ST_START	0
#define	ST_GET_SEQ_NUM	1
#define ST_MSG_SIZE_1	2
#define ST_MSG_SIZE_2	3
#define ST_GET_TOKEN	4
#define ST_GET_DATA	5
#define	ST_GET_CHECK	6
#define	ST_PROCESS	7

#define FLASH_ERASE_CMD_SIZE   6
#define FLASH_ERASE_RSP_SIZE   8
#define FLASH_ERASE_DELAY      10
#define FLASH_POLL_METHOD      1      // Use RDY/BSY
#define FLASH_ERASE_CMD1       0
#define FLASH_ERASE_CMD2       0
#define FLASH_ERASE_CMD3       0
#define FLASH_ERASE_CMD4       0

#define LOAD_ADDRESS_CMD_SIZE  5
#define LOAD_ADDRESS_RESP_SIZE  8
#define PROG_FLASH_RESP_SIZE  8
#define READ_FLASH_REQUEST_SIZE 4
#define READ_FLASH_RESP_SIZE 9



#define MESSAGE_START                       0x1B        //= ESC = 27 decimal
#define TOKEN                               0x0E

#define CMD_LOAD_ADDRESS                    0x06
#define CMD_CHIP_ERASE_ISP                  0x12

#define CMD_PROGRAM_FLASH_ISP               0x13
#define CMD_READ_FLASH_ISP                  0x14
#define CMD_PROGRAM_EEPROM_ISP              0x15

// Success
#define STATUS_CMD_OK                       0x00

// Warnings
#define STATUS_CMD_TOUT                     0x80
#define STATUS_RDY_BSY_TOUT                 0x81
#define STATUS_SET_PARAM_MISSING            0x82

// Errors
#define STATUS_CMD_FAILED                   0xC0
#define STATUS_CKSUM_ERROR                  0xC1
#define STATUS_CMD_UNKNOWN                  0xC9



#define STATUS_STK500_WRITE_ERROR    0xD0
#define STATUS_STK500_READ_ERROR     0xD1

#define LINE_CHECKSUM_ERROR          0x10
#define LINE_WRITE_ERROR             0x11
#define LINE_VERIFY_ERROR            0x12
#define ERROR_ADDRESS_WRITE          0x13
#define LINE_ADDRESS_RESP_ERROR      0x14
#define LINE_PROGRAM_TIMEOUT         0x15
#define LINE_WRITE_REQUEST_ERROR     0x16

#define LINE_BUFFER_SIZE  256
#define FLASHING_LED 41
#define COMM_LED  40

#define RESET_SLAVE_1  25
#define RESET_SLAVE_2  24
#define RESET_SLAVE_3  23

#define IIC_BUFFER_SIZE 64

#define IIC_SUCCESS 0

String lineBuffer;
unsigned char twi_buffer[IIC_BUFFER_SIZE];

boolean streamEvent = false;
unsigned int lastError;
unsigned char targetAddress;
unsigned char slaveID;
unsigned long previousTickCount;
unsigned long ledFlashPeriod;
unsigned char ledState;

unsigned char sequenceNumber;

unsigned long	boot_timeout;
unsigned long	boot_timer;
unsigned int	boot_state;

unsigned int flashAddressExtension;

/*
 * Forward function definitions
 */
boolean processSerialStream(unsigned char value);
void sendOKResponse();
void sendErrorResponse();
boolean processRequest();
int stringToInt(char *buffer);
void flashLED();
boolean verifyLineChecksum();

boolean requestTargetErase(unsigned char deviceAddress);
boolean requestRecordWrite(unsigned char deviceAddress, unsigned int flashAddress, byte *dataBuffer, unsigned int bufferSize) ;


unsigned char getIHexByteValue(String ihexLine, unsigned int linePosition, unsigned char defaultValue);
unsigned char dataLength(String ihexLine);
unsigned int recordAddress(String ihexLine);
unsigned char recordType(String ihexLine);
unsigned int getAddressExtension(String ihexLine);
boolean resetProcess(unsigned char target);
/*
 * Setup section.
 * Setup serial communcation.
 */
void setup() {
  


  //  pinMode(FLASHING_LED, OUTPUT);
  /* Set PORTG.0 and PORTG.1 direction to output */
  pinMode(FLASHING_LED, OUTPUT);
  pinMode(COMM_LED, OUTPUT);

  pinMode(RESET_SLAVE_1, OUTPUT);
  pinMode(RESET_SLAVE_2, OUTPUT);
  pinMode(RESET_SLAVE_3, OUTPUT);


  resetProcess(1);
  resetProcess(2);
  resetProcess(3);




  /* Start the local serial port. */
  Serial.begin(115200);

  /* Start the I2C system in master mode. */
  Wire.begin();


  /* Initialize global variables. */
  streamEvent = false;
  lastError = 0;
  targetAddress = 0;
  //  eraseTargetFlag = false;

  ledFlashPeriod = 250;
  ledState = 0;
  lineBuffer = "";
  

}


/*
 * Main program loop
 */
void loop() {
   

  flashLED();

  /* Read characters in from the serial port. */
  while( (Serial.available() > 0)) {
    streamEvent = processSerialStream(Serial.read());
  }


  /* Monitor incomming requests */
  if(streamEvent) {

    digitalWrite(COMM_LED, HIGH);

    if(processRequest()) {
      sendOKResponse();

    }
    else {
      sendErrorResponse();

    }

    streamEvent = false;
    lineBuffer = "";    

    digitalWrite(COMM_LED, LOW);    
  }




}



/*
 * Function: processRequest
 *
 * Input: None
 *
 * Output: None
 * 
 * Returns: true - Request processed OK, false - Request process failed.
 */
boolean processRequest() {
  boolean rValue;
  unsigned char byteCount;
  unsigned int address;
  unsigned char type;
  unsigned char dataBuffer[16];
  unsigned int i;

  lineBuffer.trim();


  rValue = false;

  if(lineBuffer.startsWith(":")) {


    /* Check for errors in the line. */
    if(false == verifyLineChecksum()) {
      lastError = LINE_CHECKSUM_ERROR;
      return false;
    }


    /* This a line from an intel hex file, process as appropriate. */
    if(0 != targetAddress) {

      /* Checksum has already been verified, it is safe process the record */

      /* Recover the bytecount */
      byteCount = dataLength(lineBuffer);

      /* Recover the address */
      address = recordAddress(lineBuffer);

      /* Recover the record type */
      type = recordType(lineBuffer);

      /* Recover the data */
      for(i=0; i<byteCount; i++) {
        dataBuffer[i] = getIHexByteValue(lineBuffer, 9 + (i*2), 0);
      }

      /* Only process data record types */
      switch(type)
      {
      case 0:      /* Data type */
        if(requestRecordWrite(targetAddress, address, dataBuffer, byteCount)) {
          /* Verify the record was written correctly. */
          rValue = requestRecordVerify(targetAddress, address, dataBuffer, byteCount);

          if(false == rValue) {
            ledFlashPeriod = 50;

          }
          else {
            ledFlashPeriod = 250;
          }
        } 
        else {
          rValue = false;
          ledFlashPeriod = 100;
        }
        break;
      case 2:    /* Extended Setment Address */
        flashAddressExtension = getAddressExtension(lineBuffer) >> 12;
        rValue = true;
        break;
      case 4:
        flashAddressExtension = getAddressExtension(lineBuffer);  
        rValue = true;
      default:
        rValue = true;
        break;
      }

    }

  }
  else if(lineBuffer.startsWith("%")) {
    /* This is a routing control line from the host application. */
    /* Recover the target address */

    lineBuffer.replace("%", "");

    targetAddress = (lineBuffer.toInt());

    if( (targetAddress > 0) && (targetAddress < 128) ) {
      /* rValue = true; */
      rValue = requestTargetErase(targetAddress);
      flashAddressExtension = 0;
    }
    else  {
      rValue = false;
      //      eraseTargetFlag = false;
    }
    
    
  }
  else if(lineBuffer.startsWith("#") ) {
    /* This is a reset control request from the host application. */
    /* Recover the Slave ID and process the reset request. */
    lineBuffer.replace("#", "");

    slaveID = (lineBuffer.toInt());

    rValue = resetProcess(slaveID);

  }
  else if(lineBuffer.startsWith("*"))
  {
    dataBuffer[0] = 1;
    dataBuffer[1] = 2;
    Wire.beginTransmission(5);
    Wire.write(dataBuffer, 2);
    Wire.endTransmission(true);
  }
  else {
    if(lineBuffer.length() == 0) {
      rValue = true;
    }
    else {
      rValue = false;
    }
  }


  return rValue;
}


unsigned char getIHexByteValue(String ihexLine, unsigned int linePosition, unsigned char defaultValue)
{
  byte buffer[4];
  unsigned int rValue;

  /* Default value */
  rValue = (unsigned int) defaultValue;


  /* Make sure the input length is appropriate for extracting the data */
  if(ihexLine.length() > linePosition+2) {

    /* Extract the data length value from an intel hex record as ASCII */
    ihexLine.substring(linePosition, linePosition + 2).getBytes(buffer, sizeof(buffer));

    /* Convert ASCII hex to binary. */
    sscanf((const char*)buffer, "%x", &rValue);
  }

  return (unsigned char)(rValue & 0x00FF);

}

/*
 * Function: resetProcess
 * Input:
 *    1 - Slave 1 (address 2),
 *    2 - Slave 2 (address 3),
 *    3 - Slave 3 (address 4)
 * Output:
 *    None
 * Returns:
 *    true - Valid target received, reset process executed
 *    false - Unknown slave ID
 */
boolean resetProcess(unsigned char target)
{
  boolean rValue;


  rValue = false;


  switch(target) {
  case 1:

    digitalWrite(RESET_SLAVE_1, LOW);
    delay(100);
    digitalWrite(RESET_SLAVE_1, HIGH);
    delay(100);
    digitalWrite(RESET_SLAVE_1, LOW);
    rValue = true;
    break;
  case 2:

    digitalWrite(RESET_SLAVE_2, LOW);
    delay(100);
    digitalWrite(RESET_SLAVE_2, HIGH);
    delay(100);
    digitalWrite(RESET_SLAVE_2, LOW);
    rValue = true;
    break;
  case 3:

    digitalWrite(RESET_SLAVE_3, LOW);
    delay(100);
    digitalWrite(RESET_SLAVE_3, HIGH);
    delay(100);
    digitalWrite(RESET_SLAVE_3, LOW);

    rValue = true;
    break;
  default:
    rValue = false;
    break;
  }
  return rValue;
}

/* 
 * Function: dataLength
 * 
 * Input:
 *    ihexLine - One record from an intel hex file.
 * Output:
 *    none
 * Returns:
 *    Record data length, or 0
 */
unsigned char dataLength(String ihexLine)
{
  return getIHexByteValue(ihexLine, 1, 0);
}


/*
 * Function: getAddressExtension
 * Input:
 *    ihexLine - One record (assumes record type is 4)
 * Output:
 *    none
 * Returns:
 *    Extended Linear Address
 */
unsigned int getAddressExtension(String ihexLine)
{
  unsigned int rValue;

  rValue = getIHexByteValue(ihexLine, 9, 0) << 8;
  rValue |= getIHexByteValue(ihexLine, 11, 0);

  return rValue;
}

/* 
 * Function: recordAddress
 * 
 * Input:
 *    ihexLine - One record from an intel hex file.
 * Output:
 *    none
 * Returns:
 *    Address field in ihex record
 */
unsigned int recordAddress(String ihexLine)
{
  unsigned int rValue;


  rValue = getIHexByteValue(ihexLine, 3, 0) << 8;
  rValue |= getIHexByteValue(ihexLine, 5, 0);


  return rValue;
}


/* 
 * Function: recordType
 * 
 * Input:
 *    ihexLine - One record from an intel hex file.
 * Output:
 *    none
 * Returns:
 *    Type of intel hex record
 */
unsigned char recordType(String ihexLine)
{
  return getIHexByteValue(ihexLine, 7, 0);
}




/*
 * Function: processSerialStream
 *
 * Input: character to process
 *
 * Returns: True - Line of text recevied, False - line pending
 */
boolean processSerialStream(unsigned char value) {

  boolean rValue;

  rValue = false;
  switch(value) {
  case '\r':
    /* Signal calling function that a request has been received */
    rValue = true;
    break;
  case '\n':
    /* Ignore \n characters. */
    break;
  default:

    /* Store characters for later processing. */
    lineBuffer += (char)value;

    break;  
  }

  return rValue;
}

/*
 * Function: sendOKResponse
 *
 * Input: None
 *
 * Output: None
 * 
 * Returns: None
 *
 * Description:
 *    Sends an OK<\n> to the host, if the serial hardware is ready
 */
void sendOKResponse() {
  Serial.println("OK");
  Serial.flush();  
}

/*
 * Function: sendErrorResponse
 *
 * Input: None
 *
 * Output: None
 * 
 * Returns: None
 *
 * Description:
 *    Sends an ERROR: N<\n> to the host, if the serial hardware is ready.
 *    N Indicates the last known error.
 *    Error Codes:
 *      TBD
 */
void sendErrorResponse() {
  Serial.print("ERROR: ");
  Serial.print(lastError, HEX);
  Serial.println("h");
  Serial.flush();
}



/*
 * Function: verifyLineChecksum
 * Input:
 *    none
 * Output:
 *   None
 * Returns:
 *    true = no errors, false = errors in line
 */
boolean verifyLineChecksum() 
{
  boolean rValue = false;
  char buffer[4];
  int bufferIndex;
  int dataLen;
  unsigned int temp;
  unsigned int checksum;


  bufferIndex = 1;
  dataLen = (lineBuffer.length()-2);


  checksum = 0;
  while(bufferIndex < dataLen) {
    lineBuffer.substring(bufferIndex, bufferIndex+2).getBytes((byte*)buffer, sizeof(buffer));
    sscanf(buffer, "%x", &temp);
    checksum += temp;

    bufferIndex += 2;
  }  

  lineBuffer.substring(bufferIndex, bufferIndex+2).getBytes((byte*)buffer, sizeof(buffer));
  sscanf(buffer, "%x", &temp);

  checksum = ((checksum ^ 0x00FF) + 1) & 0x00FF;

  if(checksum == temp) {
    rValue = true;
  }

  return rValue;  

}


void flashLED() {
  unsigned long currentTickCount;

  currentTickCount = millis();

  /*
   * Toggle the LED at the programmed rate
   */
  if( (currentTickCount - previousTickCount) > ledFlashPeriod) {
    previousTickCount = currentTickCount;

    ledState = 1 - ledState;


    /* Update the LED on/off state */
    if(ledState) {
      digitalWrite(FLASHING_LED, HIGH);
    } 
    else {
      digitalWrite(FLASHING_LED, LOW);
    }


  }


}


/*
 * Function: readSTK500V2Response
 *
 * Input:
 *    device - Target device address
 *    expectSize - Number of bytes to expect from the target device
 * 
 * Output:
 *    buffer - Data storeage for received data
 *            Note: this buffer must be large enough to hold the data
 *            plus the STK500 overhead
 *
 * Returns: True = success, false = faile
 */
boolean readSTK500V2Response(unsigned char device, unsigned char *buffer, unsigned char expectSize) {
  boolean rValue;
  int index, i;
  unsigned char checkSum;

  i = 0;
  /* Request bytes from target device */
  Wire.requestFrom(device, expectSize);

  /* Read the bytes from TWI */
  index = 0;
  while(Wire.available()) {
    buffer[index] = Wire.read();
    index++;
  }


  /* Check the data for errors */
  rValue = false;
  checkSum = 0;
  if(index == expectSize) {
    for(i=0; i<index-1; i++) {
      checkSum ^= buffer[i];
    }
  }

  if((buffer[i] == checkSum) && (index == expectSize)) {
    rValue = true;
  }

  return rValue;

}


/*
 * Function: writeSTK500V2Command
 * 
 * Input: 
 *    device - Target device address
 *    buffer - Data to send using STK500 V2 Protocol
 *    messageSize - Number of bytes in buffer
 *
 * Output:
 *    None
 *
 * Returns: true - Packet sent, False - an error occurred
 */
boolean writeSTK500V2Command(unsigned char device, unsigned char *buffer, unsigned int messageSize) {
  unsigned char cmd_buffer[IIC_BUFFER_SIZE];
  unsigned char i, j;
  unsigned char checkSum;
  boolean rValue;

  rValue = false;

  sequenceNumber = sequenceNumber + 1;
  j = 0;
  cmd_buffer[j] = MESSAGE_START;
  j++;
  cmd_buffer[j] = sequenceNumber;
  j++;
  cmd_buffer[j] = messageSize >> 8;
  j++;
  cmd_buffer[j] = messageSize & 0xFF;
  j++;
  cmd_buffer[j] = TOKEN;
  j++;


  for(i=0; i<messageSize; i++) {
    cmd_buffer[j] = buffer[i];
    j++;
  }


  checkSum = cmd_buffer[0];
  for(i=1; i< j; i++) {
    checkSum ^= cmd_buffer[i];
  }

  cmd_buffer[j] = checkSum;
  j++;


  Wire.beginTransmission(device);

  Wire.write(cmd_buffer, j);

  if(IIC_SUCCESS == Wire.endTransmission(true) ) {
    rValue = true;
  }



  return rValue;


}

/*
 * Function: requestRecordWrite
 * Input:
 *    address - Starting address for record write
 *    dataBuffer - points to a block of data bytes to be written
 *    bufferSize - number of bytes in the data buffer.
 * Output:
 *    none
 * Returns:
 *    true - success
 *    false - error
 */
boolean requestRecordWrite(unsigned char deviceAddress, unsigned int flashAddress, byte *dataBuffer, unsigned int bufferSize) 
{
  unsigned char request_buffer[64];
  unsigned char response_buffer[16];
  unsigned char retries;
  unsigned int sourceIndex, destIndex;
  boolean rValue;
  rValue = false;

  if(deviceAddress > 0 && deviceAddress < 128) {

    /* Start by issuing a load address command */
    request_buffer[0] = CMD_LOAD_ADDRESS;
    request_buffer[1] = (flashAddressExtension >> 8);
    request_buffer[2] = (flashAddressExtension & 0x00FF);    
    request_buffer[3] = (flashAddress >> 8);    
    request_buffer[4] = (flashAddress & 0xFF); 


    /* Make the Load Address request */
    if(true == writeSTK500V2Command(deviceAddress, request_buffer, LOAD_ADDRESS_CMD_SIZE)) {
      if(true == readSTK500V2Response(deviceAddress, response_buffer,LOAD_ADDRESS_RESP_SIZE)) {
        /* Check the pass fail response */
        if(response_buffer[5] == CMD_LOAD_ADDRESS && response_buffer[6] == STATUS_CMD_OK) {
          rValue = true;
        } 
        else {
          lastError = LINE_WRITE_ERROR;
          return false;

        }
      } 
      else {
        lastError = LINE_ADDRESS_RESP_ERROR;
        return false;
      }
    }
    else {
      lastError = ERROR_ADDRESS_WRITE;
    }


    /* If the process makes it this far, the load address command was exectuted properly. */
    /* Write the intel hex record to the tartget device */

    /* Write data from one record to the target device. */
    request_buffer[0] = CMD_PROGRAM_FLASH_ISP;
    request_buffer[1] = (bufferSize >> 8);
    request_buffer[2] = (bufferSize & 0x00FF);    
    request_buffer[3] = 0;    
    request_buffer[4] = 0;    
    request_buffer[5] = 0;    
    request_buffer[6] = 0;    
    request_buffer[7] = 0;    
    request_buffer[8] = 0;    
    request_buffer[9] = 0;    

    destIndex = 10;
    for(sourceIndex = 0; sourceIndex < bufferSize; sourceIndex++) {
      request_buffer[destIndex] = dataBuffer[sourceIndex];
      destIndex++;
    }


    rValue = false;
    if(true == writeSTK500V2Command(deviceAddress, request_buffer, destIndex)) {
      /* Request was written properly */

      /* Give the target time to erase a page if necessary and write the data block to flash. */
      /* The data sheet implies 4.5mS maximum for page erase/write, use 25mS for wait time */
      /* delay(50); */

      /* Read back the response */
      for(retries = 0; retries < 100; retries++) { 
        delay(10);
        if(true == readSTK500V2Response(deviceAddress, response_buffer,PROG_FLASH_RESP_SIZE)) {
          /* Check the pass fail response */
          if(response_buffer[5] == CMD_PROGRAM_FLASH_ISP && response_buffer[6] == STATUS_CMD_OK) {
            rValue=true;
            break;
          }
        }
        
      }
      
      
      if(retries >= 100) {
        lastError = LINE_PROGRAM_TIMEOUT;
      }


    } 
    else {
      lastError = LINE_WRITE_REQUEST_ERROR;
      rValue = false;  
    }



    return rValue;

  } 



  return rValue;
}

/*
 * Function: requestRecordVerify
 * Input:
 *    address - Starting address for record verify
 *    dataBuffer - points to a block of data bytes to be written
 *    bufferSize - number of bytes in the data buffer.
 * Output:
 *    none
 * Returns:
 *    true - success
 *    false - error
 */
boolean requestRecordVerify(unsigned char deviceAddress, unsigned int flashAddress, byte *dataBuffer, unsigned int bufferSize)
{
  boolean rValue;
  unsigned char request_buffer[16];
  unsigned char response_buffer[64];

  unsigned int i, localIndex, remoteIndex;
  rValue = false;

  if(deviceAddress > 0 && deviceAddress < 128) {
    /* Start by issuing a load address command */
    request_buffer[0] = CMD_LOAD_ADDRESS;
    request_buffer[1] = (flashAddressExtension >> 8);
    request_buffer[2] = (flashAddressExtension & 0x00FF);    
    request_buffer[3] = (flashAddress >> 8);    
    request_buffer[4] = (flashAddress & 0x00FF);    

    /* Make the Load Address request */
    if(true == writeSTK500V2Command(deviceAddress, request_buffer, LOAD_ADDRESS_CMD_SIZE)) {
      if(true == readSTK500V2Response(deviceAddress, response_buffer,LOAD_ADDRESS_RESP_SIZE)) {
        /* Check the pass fail response */
        if(response_buffer[5] == CMD_LOAD_ADDRESS && response_buffer[6] == STATUS_CMD_OK) {
          rValue = true;
        } 
        else {
          return false;
        }
      } 
      else {
        return false;
      }
    }

    /* Read back the defined record. */
    request_buffer[0] = CMD_READ_FLASH_ISP;
    request_buffer[1] = (bufferSize >> 8);
    request_buffer[2] = (bufferSize & 0x00FF);
    request_buffer[3] = 0;

    if(true == writeSTK500V2Command(deviceAddress, request_buffer, READ_FLASH_REQUEST_SIZE)) {
      delay(5);
      if(true == readSTK500V2Response(deviceAddress, response_buffer, READ_FLASH_RESP_SIZE + bufferSize)) {
        rValue = true;
      } 
      else {
        rValue = false;
      }
    } 
    else {
      rValue = false;
    }
  }

  /* If the communication requests succeed, compare the two buffers. */
  if(rValue) {

    /* set up some data indexes */
    localIndex = 0;
    remoteIndex = 7;

    /* Compare the remote data to the local data */
//    Serial.print("Line: ");    
    for(i=0; i<bufferSize; i++)
    {
//      Serial.print(dataBuffer[localIndex], HEX);
//      Serial.print(":");
//      Serial.print(response_buffer[remoteIndex], HEX);
//      Serial.print(", ");
      if(dataBuffer[localIndex] != response_buffer[remoteIndex]) {
        /* If there's an error, fail the function */
        rValue = false;
        break;
      }
      localIndex++;
      remoteIndex++;
    }
  }


  return rValue;
}

/*
 * Function: requestTargetErase
 *
 * Input: address: I2C address of target device
 *
 * Output: none
 *
 * Returns: true = target is erase, false = target erase failed.
 *
 */
boolean requestTargetErase(unsigned char deviceAddress) {

  unsigned char request_buffer[64];
  unsigned char response_buffer[16];

  boolean rValue;
  rValue = false;
  if( (deviceAddress > 0) && (deviceAddress < 128) ) {


    request_buffer[0] = CMD_CHIP_ERASE_ISP;
    request_buffer[1] = FLASH_POLL_METHOD;
    request_buffer[2] = FLASH_ERASE_CMD1;    
    request_buffer[3] = FLASH_ERASE_CMD2;    
    request_buffer[4] = FLASH_ERASE_CMD3;    
    request_buffer[5] = FLASH_ERASE_CMD4;    


    /* Make the Flash Erase Request */
    if(true == writeSTK500V2Command(deviceAddress, request_buffer, FLASH_ERASE_CMD_SIZE)) {
      delay(50);
      if( true == readSTK500V2Response(deviceAddress, response_buffer,FLASH_ERASE_RSP_SIZE)) {
        /* Check the pass fail response */
        if(response_buffer[5] == CMD_CHIP_ERASE_ISP && response_buffer[6] == STATUS_CMD_OK) {
          rValue = true;
        } 
        else {
          rValue = false;
        }
      } 
      else {
        lastError = STATUS_STK500_READ_ERROR;
      }


    } 
    else {
      lastError = STATUS_STK500_WRITE_ERROR;
    }
  }

  return rValue;  
}











