/*
 * twi_mega2560.c
 *
 * Created: 12/19/2014 2:53:36 PM
 *  Author: Lenovo
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <string.h>

#include "twi.h"
#include "flash_utilities.h"

#define APP_END 0x1F000
#define EXTENDED_SECTION
#define TWI_SLAVE_ADDRESS 2
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

#define BOOT_MODE_APPLICATION	0xA5
#define BOOT_MODE_STK500	0xFF

uint8_t TWI_slaveAddress = TWI_SLAVE_ADDRESS;

static uint8_t	st_sequence_number;
static uint16_t st_message_size;

static uint32_t eraseAddress;
static uint32_t address;

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


/* Block out application EEPROM space */
#define BOOT_MODE_ADDRESS ((uint8_t*)4095)

static uint8_t bootLoadMode = BOOT_MODE_STK500;


int main(void)
{
	
	requestFlag = false;
	responseFlag = false;
	
	/* Test the boot load mode. */
	bootLoadMode = eeprom_read_byte(BOOT_MODE_ADDRESS);


	if(bootLoadMode == BOOT_MODE_APPLICATION) {
		/* Jump to absolute address 0x00000000 */
		/* Starts the loaded application.      */
		asm volatile(
			"JMP	0x00000000\r\t"
		);
	}




	/*
	 * This inline assembly moves the IVT from Application Flash to 
	 * Bootloader Flash. Assembly is required due to timing restrictions
	 * of the part. The C compiler doesn't guarantee that timing for this
	 * function can be met at any optimization level. DO NOT try to 
	 * accomplish this with the C equivalent as documented in the reference 
	 * manual. It could cost you many hours of troubleshooting. 
	 */
	asm volatile (
		/* Setup indirect addressing of the MCUCR register */
		"ldi	r30, 0x55\n\t"		
		"ldi	r31, 0x00\n\t" \
		
		/* Unlock for change of IVT Address */
		"ldi	r24, 0x01\n\t" \
		"st	z, r24\n\t" \
		
		/* Move the IVT to boot Flash */
		"ldi	r24, 0x02\n\t" \
		"st	z, r24\n\t"
	::);


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
//	uint8_t temp[4];
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
		
		
	}
	
		
	write_flash_page(p, address, bufSize);	
	
	
	/* Build up an OK response */
	build_response_buffer(CMD_PROGRAM_FLASH_ISP, STATUS_CMD_OK);


	if(0x00000000 == address) {
		/* If the reset vector gets re-written, assume next reset is a boot app action. */
		eeprom_write_byte(BOOT_MODE_ADDRESS, BOOT_MODE_APPLICATION);			
	}



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
//	uint16_t temp;
	
	bufSize = (((commandBuffer[1]) << 8) | (commandBuffer[2]));
	
	if(bufSize <= 16) {
		
		
		
		rValue = true;
		
		//memcpy_P(&twi_message_buffer[7], (const void*)address, bufSize);
		memcpy_PF(&twi_message_buffer[7], (uint_farptr_t)address, bufSize);
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
static void commandProcessor(uint8_t *commandBuffer, uint16_t size)
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
static bool processLoadAddressRequest(uint8_t* commandBuffer, uint16_t size)
{
	bool rValue;
	rValue = false;
	
	address = ((((uint32_t)commandBuffer[1]) << 24) | (((uint32_t)commandBuffer[2]) << 16) | 
		(((uint32_t)commandBuffer[3]) << 8) | ((uint32_t)commandBuffer[4])); // Convert the word address to byte address
	
	if(address < (APP_END << 1)) {
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
static void build_response_buffer(uint8_t responseCommand, uint8_t responseValue) 
{
	buildMessageHeader(2);
	
	twi_message_buffer[5] = responseCommand;
	twi_message_buffer[6] = responseValue;
	
	twi_message_buffer[7] = calculateSTK500Checksum(twi_message_buffer, 7);
	twi_message_length = 8;
}


static void buildMessageHeader(uint16_t length)
{
	twi_message_buffer[0] = MESSAGE_START;
	twi_message_buffer[1] = st_sequence_number;
 	twi_message_buffer[2] = (length >> 8);
 	twi_message_buffer[3] = (length & 0x00FF);
	twi_message_buffer[4] = TOKEN;
}