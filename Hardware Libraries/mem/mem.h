/*
 * mem.h
 *
 *  Created on: Mar 28, 2021
 *      Author: John Tarasidis
 */

#ifndef INC_MEM_H_
#define INC_MEM_H_

uint8_t mem_init(void);

void mem_read(uint32_t, uint8_t*);

uint8_t mem_write(uint32_t, uint8_t*, uint8_t);

uint8_t mem_chip_erase(void);

#endif /* INC_MEM_H_ */
