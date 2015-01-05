/*
 * flash_utilities.h
 *
 * Created: 12/9/2014 11:22:39 AM
 *  Author: Lenovo
 */ 


#ifndef FLASH_UTILITIES_H_
#define FLASH_UTILITIES_H_

void erase_flash_page(uint32_t flashPage);

void write_flash_page(uint8_t* buffer, uint32_t address, uint16_t len);

#endif /* FLASH_UTILITIES_H_ */