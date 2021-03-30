/*
 * mem.c
 *
 *  Created on: Mar 28, 2021
 *      Author: John Tarasidis
 */
#include "main.h"
#include "mem.h"

extern QSPI_HandleTypeDef hqspi;

/*
 * This function performs the following set up flow:
 *
 * Write Enable - single cmd
 * Check Write Enable Latch (bit1) - autopoll; if timeout return fail init
 * Write Status Register Quad Enable - single cmd
 * Check Write In-Progress (bit0) -autopoll; if timeout return fail init
 * Read Status Register Quad Enable (bit6) - single cmd
 *
 * if QE Bit high, return 1 and exit
 */
uint8_t mem_init(void) {
	//WREN - p20 on datasheet
	QSPI_CommandTypeDef write_enable;
	write_enable.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	write_enable.Instruction = 0x06;
	write_enable.AddressMode = QSPI_ADDRESS_NONE;
	write_enable.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	write_enable.DataMode = QSPI_DATA_NONE;
	write_enable.NbData = 0;
	write_enable.DummyCycles = 0;
	write_enable.DdrMode = QSPI_DDR_MODE_DISABLE;
	write_enable.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	HAL_QSPI_Command(&hqspi, &write_enable, HAL_MAX_DELAY);

	QSPI_CommandTypeDef read_status_register;
	read_status_register.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	read_status_register.Instruction = 0x05;
	read_status_register.AddressMode = QSPI_ADDRESS_NONE;
	read_status_register.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	read_status_register.DataMode = QSPI_DATA_1_LINE;
	read_status_register.NbData = 1;
	read_status_register.DummyCycles = 0;
	read_status_register.DdrMode = QSPI_DDR_MODE_DISABLE;
	read_status_register.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	QSPI_AutoPollingTypeDef RDSR_WEL;
	RDSR_WEL.Match = 0x02;
	RDSR_WEL.Mask = 0x02;
	RDSR_WEL.Interval = 5;
	RDSR_WEL.StatusBytesSize = 0x01;
	RDSR_WEL.MatchMode = QSPI_MATCH_MODE_AND;
	RDSR_WEL.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WEL, 15) == HAL_ERROR) {
		return MEM_INIT_FAIL;
	}

	QSPI_CommandTypeDef write_status_register;
	write_status_register.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	write_status_register.Instruction = 0x01;
	write_status_register.AddressMode = QSPI_ADDRESS_NONE;
	write_status_register.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	write_status_register.DataMode = QSPI_DATA_1_LINE;
	write_status_register.NbData = 3;
	write_status_register.DummyCycles = 0;
	write_status_register.DdrMode = QSPI_DDR_MODE_DISABLE;
	write_status_register.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	uint8_t QEN[3] = {0x40, 0x00, 0x00};
	HAL_QSPI_Command(&hqspi, &write_status_register, HAL_MAX_DELAY);
	HAL_QSPI_Transmit(&hqspi, QEN, HAL_MAX_DELAY);
	HAL_Delay(1);

	QSPI_AutoPollingTypeDef RDSR_WIP;
	RDSR_WIP.Match = 0x00;
	RDSR_WIP.Mask = 0x01;
	RDSR_WIP.Interval = 5;
	RDSR_WIP.StatusBytesSize = 0x01;
	RDSR_WIP.MatchMode = QSPI_MATCH_MODE_AND;
	RDSR_WIP.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WIP, 15) == HAL_ERROR) {
		return MEM_INIT_FAIL;
	}

	HAL_QSPI_Command(&hqspi, &read_status_register, HAL_MAX_DELAY);
	uint8_t stat = 0;
	HAL_QSPI_Receive(&hqspi, &stat, HAL_MAX_DELAY);

	if (stat & QEN[0]) {
		return MEM_INIT_SUCCESS;
	} else {
		return MEM_INIT_FAIL;
	}
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
