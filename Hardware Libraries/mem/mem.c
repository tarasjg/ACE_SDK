/*
 * mem.c
 *
 *  Created on: Mar 28, 2021
 *      Author: John Tarasidis
 */
#include "main.h"
#include "mem.h"

/*
 * This function performs the following set up flow:
 *
 * Write Enable - single cmd
 * Check Write Enable Latch - autopoll; if timeout return fail init
 * Write Status Register Quad Enable - single cmd
 * Check Write In-Progress -autopoll; if timeout return fail init
 * Read Status Register Quad Enable - single cmd
 *
 * if QE Bit high, return 1 and exit
 */
uint8_t mem_init(void) {

}

/*
 * This function performs the following:
 *
 * Input: a 32 bit uint32_t address
 * 		  a 32 byte uint8_t array
 *
 * A Quad Read Sequence will execute at the address provided for 32 bytes
 *
 * Output: void; data_r will be populated with the data from the external memory
 */
void mem_read(uint32_t addr, uint8_t* data_r) {

}

/*
 * This function performs the following:
 *
 * Input: a 32 bit uint32_t address
 * 		  a n byte uint8_t array
 * 		  a uint8_t length
 *
 * Input safe guards: A random address cannot proceed past the page boundary.
 * 					  Thus, if the least significant address byte plus the length is >255,
 * 					  the function will not execute and return 0
 * 					  If a 256 byte write is desired, the least significant address byte must be 0.
 *
 * A Quad Page Program Sequence will execute at the address provided.
 *
 * Output: return 1 on completion
 */
uint8_t mem_write(uint32_t addr, uint8_t* data_w, uint8_t len) {

}

/*
 * This function performs the following:
 *
 * A Chip Erase Sequence will execute
 *
 * A blocking check on the WIP bit will execute in autopoll mode
 *
 * Output: return 0 if autopoll times out
 * 		   return 1 if WIP match
 */
uint8_t mem_chip_erase(void) {

}
