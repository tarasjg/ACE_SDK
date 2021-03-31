/*
 * mem.h
 *
 *  Created on: Mar 28, 2021
 *      Author: John Tarasidis
 */

#ifndef INC_MEM_H_
#define INC_MEM_H_

#define MEM_INIT_SUCCESS 0x01
#define MEM_INIT_FAIL	 0x00

#define MEM_ERASE_SUCCESS 0x01
#define MEM_ERASE_TIMEOUT 0x00

#define MEM_WRITE_SUCCESS 		 0x01
#define MEM_WRITE_INVALID_LENGTH 0x02
#define MEM_WRITE_TIMEOUT  		 0x00

uint8_t mem_init(void);

void mem_read(uint32_t, uint8_t*);

uint8_t mem_write(uint32_t, uint8_t*, uint8_t);

uint8_t mem_chip_erase(void);

#endif /* INC_MEM_H_ */
