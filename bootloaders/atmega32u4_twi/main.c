/*
 * twi_leonardo.c
 *
 * Created: 11/20/2014 9:59:44 AM
 *  Author: tgack
 *
 * Inputs:
 *	PortD.6 - Mode Select
 *		Open - Run Boot loader
 *		Short to Ground - Run a loaded application.
 *	SDA/SCL - I2C communication from host. 
 *	
 * Memory Map (word address):
 *     |--------|------------------------------------------|
 *     | 0x0000 | Reset ISR                                |
 *     | 0x0004 | Application interrupt vector             |
 *     | ...    |                                          |
 *     | 0x00D0 | TWI interrupt Vector                     |
 *     | 0x00D4 | Application interrupt vector             |
 *     | ....   | Application space and interrupt vectors  | 
 *     | 0x377F | End of Application Space                 |
 *     | 0x3780 | Start of boot loader extended space      |
 *     | 0x37FF | End of boot loader extended space        |
 *     | 0x3800 | Boot loader interrupt vector table       |
 *     | ....   | Boot loader application space            | 
 *     | 0x3FFF | End of flash memory segment              |
 *     |--------|------------------------------------------|
 * 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <string.h>

#include "twi.h"
#include "flash_utilities.h"


#define APP_END 0x6E00
#define EXTENDED_SECTION __attribute__((section(".text_extended")))
#define TWI_SLAVE_ADDRESS 1
#define RECEIVE_BUF_SIZE 80

#define MESSAGE_START	0x1B	// = ESC = 27 decimal
#define TOKEN		0x0E


#define CMD_LOAD_ADDRESS 0x06
#define CMD_CHIP_ERASE	0x12
#define CMD_PROGRAM_FLASH_ISP 0x13
#define CMD_READ_FLASH_ISP 0x14

#define ST_START	0
#define ST_GET_SEQ_NUM	1
#define ST_MSG_SIZE_1	2
#define ST_MSG_SIZE_2	3
#define ST_GET_TOKEN	4
#define ST_GET_DATA	5
#define ST_GET_CHECK	6
#define ST_PROCESS	7

#define STATUS_CMD_OK		0x00
#define STATUS_CMD_FAILED	0xC0

#define BOOT_MODE_APPLICATION	0x00
#define BOOT_MODE_STK500	0x01

uint8_t TWI_slaveAddress = TWI_SLAVE_ADDRESS;

static uint8_t	st_sequence_number;
static uint16_t st_message_size;

static uint16_t eraseAddress;
static uint16_t address;

static bool requestFlag;
static bool responseFlag;

uint8_t twi_message_buffer[RECEIVE_BUF_SIZE];
uint8_t twi_message_length;

void onTWIReceive(uint8_t *buffer, int length);
void onTWIReadRequest(void);
static void processRequest(void);
static uint8_t calculateSTK500Checksum(uint8_t* buffer, uint16_t length);
static void build_response_buffer(uint8_t responseCommand, uint8_t responseValue) ;
static void commandProcessor(uint8_t* commandBuffer, uint16_t size);
static bool processEraseRequest(uint8_t* commandBuffer, uint16_t size);
static bool processLoadAddressRequest(uint8_t* commandBuffer, uint16_t size);
static bool processProgramFlashIsp(uint8_t* commandBuffer, uint16_t size);
static bool processReadFlashIsp(uint8_t* commandBuffer, uint16_t size);
static void buildMessageHeader(uint16_t length);

static uint16_t resetVector;
static uint16_t twiVector;
static uint16_t appResetVector;
static uint16_t appTWIVector;

/* Block out application EEPROM space */
static uint8_t EEMEM __attribute__((unused))reservedForApplication[1024-4];
static uint16_t EEMEM applicationResetVector;
uint16_t EEMEM applicationTWIResetVector;
extern uint16_t twi_isr_vector_address;

static uint8_t bootMode = BOOT_MODE_STK500;

int main(void)
{
	/* Use PortD.6 for boot load signal */
	/* Enable the pull up. If the pin is shorted to ground ... */
	/* ... run the application. */
	DDRD &= ~0x40;
	
	/* Enable pull up resistor. */	
	PORTD |= 0x40;
	/* MCUCR Pull up resistors are enabled by default. */
	
	/* If the jumper is on, then run the application. */
	if(0 == (PIND & 0x40) ) {
		bootMode = BOOT_MODE_APPLICATION;
	}
	
	requestFlag = false;	
	responseFlag = false;

	/* Store the current reset and TWI vectors */
	/* These two vectors will be replaced in the incoming boot load */
	/* data stream. */
	memcpy_P(&resetVector, (const void*)0x0002, 2);
	memcpy_P(&twiVector, (const void*)0x0092, 2);
	
	appResetVector = eeprom_read_word(&applicationResetVector);
	appTWIVector = eeprom_read_word(&applicationTWIResetVector);

	

	if(BOOT_MODE_APPLICATION == bootMode) {
		
		/* Setup the TWI interrupt vector address */
		twi_isr_vector_address = appTWIVector;
		
		/* This places the jump to address into the Z register (R30:R31) */
		/* Then executes an indirect jump to that location. This function */
		/* emulates what happens on a system reset. */
		asm volatile (
			"ijmp\n\t"	\
			:
			: "z" (appResetVector)
		);	
	}
	
	/* Initialize the I2C peripheral in slave mode. */
	twi_init();
	twi_setAddress(TWI_slaveAddress);
	twi_attachSlaveRxEvent(onTWIReceive);
	twi_attachSlaveTxEvent(onTWIReadRequest);
	
	
	/* Start interrupts, Note: USE ONLY TWI INTERRUPT. */
	sei();
	
	while(1) {
		
		if(requestFlag) {
			cli();
			requestFlag = false;
			sei();
			
			/* Process the incoming request */
			processRequest();
		}
	}
}

/*
 * Event Handler - Called from TWI ISR
 * Defer processing any received data under interrupt.
 * Signals the main loop to process the current buffer
 */
void onTWIReceive(uint8_t *buffer, int length)
{
	memcpy(twi_message_buffer, buffer, length);
	twi_message_length = length;
	requestFlag = true;
}

/*
 * Event Handler - Called from TWI ISR
 * 
 * This function is called when an IIC master makes a request
 * to this device. If there data pending as a result of
 * the previous request, send it here. If there is no data
 * return an empty packet to signal NULL function.
 */
void onTWIReadRequest(void) 
{
	if(!responseFlag) {
		/* Build up a NULL response */
		build_response_buffer(0, 0); 	
		twi_transmit(twi_message_buffer, twi_message_length);
	} else {
		twi_transmit(twi_message_buffer, twi_message_length);
		responseFlag = false;
	} 
	
}


/*
 * Function: processRequest
 * Input:
 *	none
 * Output:
 *	none
 * Returns:
 *	none
 * Description:
 *	Verifies the checksum on the current request. If no errors
 *	are detected, save the sequence numeber, extract the data
 *	segment of the request, and pass the request to the command
 *	processor.
 */
static void processRequest(void) 
{
	uint8_t bufferChecksum;
	
	/* Perform some basic tests to insure that this is a STK500 V2 request. */
	if(twi_message_buffer[0] != MESSAGE_START) {
		return;
	}
	
	bufferChecksum = calculateSTK500Checksum(twi_message_buffer, twi_message_length-1);
	
	if((bufferChecksum == twi_message_buffer[twi_message_length-1]) && (twi_message_buffer[4] == TOKEN)) {
		/* Checksum is good and the TOKEN mark is present. Decode this request */
		st_sequence_number = twi_message_buffer[1];
		st_message_size = ((uint16_t)twi_message_buffer[2] << 8) + (uint16_t)twi_message_buffer[3];
		commandProcessor(&twi_message_buffer[5], st_message_size);
	}	
}





/*
 * Function: processEraseRequest
 * Input:
 *	commandBuffer - Points to request commands
 *	size - number of bytes in the command buffer
 * Output:
 *	none
 * Returns:
 *	always returns true
 *
 * Description:
 *	The process erase request should always succeed, and must be called
 *	before attempting to write flash. This function simply resets the
 *	eraseAddress variable.
 */
static bool processEraseRequest(uint8_t* commandBuffer, uint16_t size) 
{
	eraseAddress = 0;
	
	/* Build up an OK response */	
	build_response_buffer(CMD_CHIP_ERASE, STATUS_CMD_OK);

	return true;	
}



/*
 * Function: processProgramFlashIsp
 * Input:
 *	commandBuffer - Points to request commands
 *	size - number of bytes in the command buffer
 * Output:
 *	none
 * Returns:
 *	true - function finished
 *	false - always returns true
 *
 * Description:
 *	This function writes a block of data to Flash memory. Since the
 *	return flag is an indicator that the function finished, not 
 *	whether or not the flash write succeeded, the host application
 *	needs to perform verification by reading back the contents of
 *	flash.
 */
static bool processProgramFlashIsp(uint8_t* commandBuffer, uint16_t size)
{
	uint8_t temp[4];
	uint8_t *p;
	uint16_t bufSize;
	
	bufSize = (((commandBuffer[1]) << 8) | (commandBuffer[2]));
	p = &commandBuffer[10];
	
	/* Check page boundary and erase pages as the address comes around */
	if(address == eraseAddress) {
		/* Perform basic bounds check and don't erase this boot loader */
		if(eraseAddress <= (APP_END - SPM_PAGESIZE)) {
			erase_flash_page(eraseAddress);
			eraseAddress += SPM_PAGESIZE;
		}
		
		/* Handle the TWI ISR Page erase */
		if((address >= 0x0080) && (address < 0x0100)) {
			// Replace the TWI ISR, it is currently blank
			temp[0] = 0x0C;
			temp[1] = 0x94;
			temp[2] = (twiVector & 0x00FF);
			temp[3] = (twiVector >> 8);
			write_flash_page(temp, 0x0090, 4);
		}
	}
	
	
	/* The reset vector is on an even record boundary. */
	/* No further logic is required for processing this vector. */
	if(address == 0x0000) {
		
		appResetVector = p[2];
		appResetVector |= (p[3]) << 8;
		
		//p[0] = 0x0C;
		//p[1] = 0x94;
		p[2] = (uint8_t)(resetVector & 0x00FF);
		p[3] = (uint8_t)(resetVector >> 8);
		
		
		eeprom_update_word(&applicationResetVector, appResetVector);
	}
	
	
	/* The TWI vector is on an even record boundary. */
	/* No further logic is required for processing this vector. */
	if(address == 0x0090) {
		appTWIVector = p[2];
		appTWIVector |= (p[3]) << 8;
		
		//p[0] = 0x0C;
		//p[1] = 0x94;
		p[2] = (uint8_t)(twiVector & 0x00FF);
		p[3] = (uint8_t)(twiVector >> 8);
		
		eeprom_update_word(&applicationTWIResetVector, appTWIVector);
		
	} 
	
		
	write_flash_page(p, address, bufSize);	
	
	
	/* Build up an OK response */
	build_response_buffer(CMD_PROGRAM_FLASH_ISP, STATUS_CMD_OK);

	return true;
}

/*
 * Function: processReadFlashIsp
 * Input:
 *	commandBuffer - buffer containing command to process
 *	size - number of bytes in the block of memory
 * Output:
 *	none
 * Returns:
 *	true - success
 *	false - fail
 * Description:
 *	Builds a response packet that will be returned on the next TWI
 *	read request.
 */
static bool processReadFlashIsp(uint8_t* commandBuffer, uint16_t size)
{
	bool rValue = false;
	uint16_t bufSize;
	uint16_t temp;
	
	bufSize = (((commandBuffer[1]) << 8) | (commandBuffer[2]));
	
	if(bufSize <= 16) {
		
		
		
		rValue = true;
		
		memcpy_P(&twi_message_buffer[7], (const void*)address, bufSize);

		/* If the reset address vector is in this packet, then replace the FLASH */
		/* vector with the vector address stored in EEPROM */
		if((address  == 0x0000) && (bufSize >= 4)) {
			temp = eeprom_read_word(&applicationResetVector);
			twi_message_buffer[9] = (temp & 0x00FF);
			twi_message_buffer[10] = (temp >> 8);
		}
		
		if((address == 0x0090) && (bufSize >= 4)) {
			temp = eeprom_read_word(&applicationTWIResetVector);
			twi_message_buffer[9] = (temp & 0x00FF);
			twi_message_buffer[10] = (temp >> 8);
			
		}

		address += bufSize;
		
		buildMessageHeader(bufSize);
		
		
		
		twi_message_buffer[5] = CMD_READ_FLASH_ISP;
		twi_message_buffer[6] = STATUS_CMD_OK;
		
		twi_message_buffer[7 + bufSize] = STATUS_CMD_OK;
		twi_message_buffer[8 + bufSize] = calculateSTK500Checksum(twi_message_buffer, 7 + bufSize);
		twi_message_length = 9 + bufSize;
	}
	
	
	return rValue;
}

/*
 * Function: calculateSTK500Checksum
 * 
 * Input: 
 *	buffer - Pointer to buffer containing source data 
 *	length - Number of bytes in the source data buffer
 * Output:
 *	none
 * Returns:
 *	XOR Checksum based on source buffer.
 */	
static uint8_t calculateSTK500Checksum(uint8_t* buffer, uint16_t length) 
{
	uint16_t index;
	uint8_t checksum;
	checksum = buffer[0];
	for(index=1; index < length; index++) {
		checksum ^= buffer[index];
	}
	return checksum;
}


/* 
 * Function: commandProcessor
 * Input:
 *	commandBuffer - Block containing request command
 *	size - number of bytes in the request 
 * Output:
 *	none
 * Returns:
 *	none
 * Description:
 *	Routes requests to the appropriate command processing function
 */
static void EXTENDED_SECTION commandProcessor(uint8_t *commandBuffer, uint16_t size)
{
	switch(commandBuffer[0])
	{
	case CMD_CHIP_ERASE:
		responseFlag = processEraseRequest(commandBuffer, size);
		break;
	case CMD_LOAD_ADDRESS:
		responseFlag = processLoadAddressRequest(commandBuffer, size);
		break;
	case CMD_PROGRAM_FLASH_ISP:
		responseFlag = processProgramFlashIsp(commandBuffer, size);
		break;
	case CMD_READ_FLASH_ISP:
		responseFlag = processReadFlashIsp(commandBuffer, size);
		break;
	default:
		responseFlag = false;
		break;	
	}
	
}
/*
 * Function: processLoadAddressRequest
 * Input:
 *	commandBuffer - points to the data segment of a received packet
 *	size - Number of bytes in the data segment
 * Output:
 *	none
 * Returns:
 *	true = success
 *	false - Address out of bounds.
 */
static bool EXTENDED_SECTION processLoadAddressRequest(uint8_t* commandBuffer, uint16_t size)
{
	bool rValue;
	rValue = false;
	
	address = (((commandBuffer[3]) << 8) | (commandBuffer[4])); // Convert the word address to byte address
	
	if(address < (APP_END)) {
		build_response_buffer(CMD_LOAD_ADDRESS, STATUS_CMD_OK);
		rValue= true;
	} else {
		build_response_buffer(CMD_LOAD_ADDRESS, STATUS_CMD_FAILED);
	}
	
	
	return rValue;
}


/* 
 * Function: build_response_buffer
 * Input:
 *	responseCommand: command response is generated for
 *	responseValue: Pass / Fail indicator for response
 * Output:
 *	none
 * Returns:
 *	none
 * Description:
 *	This function simply builds an STK500 response packet. This
 *	response will be returned to the host on the next TWI read
 *	request.
 */
static void  EXTENDED_SECTION build_response_buffer(uint8_t responseCommand, uint8_t responseValue) 
{
	buildMessageHeader(2);
	
	twi_message_buffer[5] = responseCommand;
	twi_message_buffer[6] = responseValue;
	
	twi_message_buffer[7] = calculateSTK500Checksum(twi_message_buffer, 7);
	twi_message_length = 8;
}


static void EXTENDED_SECTION buildMessageHeader(uint16_t length)
{
	twi_message_buffer[0] = MESSAGE_START;
	twi_message_buffer[1] = st_sequence_number;
 	twi_message_buffer[2] = (length >> 8);
 	twi_message_buffer[3] = (length & 0x00FF);
	twi_message_buffer[4] = TOKEN;
}