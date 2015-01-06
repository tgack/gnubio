/*
 * flash_utilities.c
 *
 * Created: 12/9/2014 11:22:24 AM
 *  Author: Lenovo
 */ 


#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>

typedef uint16_t address_t;

void erase_flash_page(uint16_t flashPage)
{
	 uint8_t sreg;
	 
	
	sreg = SREG;
	
	cli();
	eeprom_busy_wait ();
        boot_page_erase (flashPage);
        boot_spm_busy_wait ();      // Wait until the memory is erased.
	
	
        boot_rww_enable ();

	
	
	
	SREG = sreg;
}


void write_flash_page(uint8_t* buffer, uint16_t address, uint16_t len)
{
	uint16_t size;
	uint8_t sreg;
	uint8_t *p;
	uint16_t data;
	uint16_t lowByte, highByte;
	address_t tempAddress;
	
	tempAddress = address;
	p = buffer;
	size = len;
	
	sreg = SREG;
	cli();
	eeprom_busy_wait ();
	
	
	while(size) {
		lowByte = *p++;
		highByte = *p++;
		
		data = (highByte << 8) | lowByte;
		boot_page_fill(address, data);	
		address += 2;
		size-=2;
	}
	
	boot_page_write(tempAddress);
	boot_spm_busy_wait();
	boot_rww_enable();
	
	SREG = sreg;
	
}