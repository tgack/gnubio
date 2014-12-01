
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

#define LINE_BUFFER_SIZE  256
#define FLASHING_LED 13
String lineBuffer;
boolean streamEvent = false;
unsigned int lastError;
unsigned char targetAddress;
boolean eraseTargetFlag;
unsigned long previousTickCount;
unsigned long ledFlashPeriod;
unsigned char ledState;

/*
 * Forward function definitions
 */
boolean processSerialStream(unsigned char value);
void sendOKResponse();
void sendErrorResponse();
boolean processRequest();
int stringToInt(char *buffer);
void flashLED();

/*
 * Setup section.
 * Setup serial communcation.
 */
void setup() {

  pinMode(FLASHING_LED, OUTPUT);

  Serial.begin(115200);

  /* Initialize global variables. */
  streamEvent = false;
  lastError = 0;
  targetAddress = 0;
  eraseTargetFlag = false;

  ledFlashPeriod = 250;
  ledState = 0;
  lineBuffer = "";

}


/*
 * Main program loop
 */
void loop() {

  /* Monitor incomming requests */
  if(streamEvent) {

    if(processRequest()) {
      sendOKResponse();
    }
    else {
      sendErrorResponse();
    }

    streamEvent = false;
    lineBuffer = "";

  }


  flashLED();


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
  boolean rValue = false;    // Assume the worst

  lineBuffer.trim();

  if(lineBuffer.startsWith(":")) {
    /* This a line from an intel hex file, process as appropriate. */
    if(eraseTargetFlag) {
      /* TODO: Implement target erase logic here */
      eraseTargetFlag = false;
    }


    if(0 != targetAddress) {
      /* TODO: Implement target program flash/eeprom here. */
      rValue = true;
    }
  }
  else if(lineBuffer.startsWith("%")) {
    /* This is a routing control line from the host application. */
    /* Recover the target address */

    lineBuffer.replace("%", "");

    targetAddress = (lineBuffer.toInt());

    if( (targetAddress > 0) && (targetAddress < 128) ) {
      rValue = true;
      eraseTargetFlag = true;
    }
    else  {
      rValue = false;
      eraseTargetFlag = false;
    }
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




/* 
 * Function: serialEvent - 
 *    Function called whenever a serial character is received
 *
 * Parameters: None
 *
 * Return: nothing
 */
void serialEvent() {

  while( (Serial.available() > 0)) {
    streamEvent = processSerialStream(Serial.read());
  }
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
  Serial.print(lastError, DEC);
  Serial.println("");
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

    digitalWrite(FLASHING_LED, ledState);

  }


}





