/*
 * mem.c
 *
 *  Created on: Mar 28, 2021
 *      Author: John Tarasidis
 */
#include "main.h"
#include "mem.h"

extern QSPI_HandleTypeDef hqspi;

//global command structures


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
	QSPI_CommandTypeDef write_enable = {0};
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

	QSPI_CommandTypeDef read_status_register = {0};
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
	RDSR_WEL.Interval = 1;
	RDSR_WEL.StatusBytesSize = 0x01;
	RDSR_WEL.MatchMode = QSPI_MATCH_MODE_AND;
	RDSR_WEL.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WEL, 35) == HAL_ERROR) {
		return MEM_INIT_FAIL;
	}

	QSPI_CommandTypeDef write_status_register = {0};
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
	RDSR_WIP.Interval = 1;
	RDSR_WIP.StatusBytesSize = 0x01;
	RDSR_WIP.MatchMode = QSPI_MATCH_MODE_AND;
	RDSR_WIP.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WIP, 35) == HAL_ERROR) {
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
	QSPI_CommandTypeDef quad_read = {0};
	quad_read.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	quad_read.Instruction = 0x6B;
	quad_read.AddressMode = QSPI_ADDRESS_1_LINE;
	quad_read.Address = addr;
	quad_read.AddressSize = QSPI_ADDRESS_24_BITS;
	quad_read.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	quad_read.DataMode = QSPI_DATA_4_LINES;
	quad_read.NbData = 1;
	quad_read.DummyCycles = 8;
	quad_read.DdrMode = QSPI_DDR_MODE_DISABLE;
	quad_read.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	//quad_read.SIOOMode = QSPI_SIOO_INST_ONLY_FIRST_CMD;

	HAL_QSPI_Command(&hqspi, &quad_read, HAL_MAX_DELAY);

	HAL_QSPI_Receive(&hqspi, data_r, HAL_MAX_DELAY);
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
uint8_t mem_write(uint32_t addr, uint8_t *data_w, uint16_t len) {
	uint8_t addr_LSB = addr & 0xFF;
	uint16_t len_for_cur_page = (uint16_t) addr_LSB + (uint16_t) len;
	if (len_for_cur_page > 256) {
		len_for_cur_page = 256 - addr_LSB;
	}
	QSPI_CommandTypeDef write_enable = { 0 };
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

	QSPI_CommandTypeDef read_status_register = { 0 };
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
	RDSR_WEL.Interval = 1;
	RDSR_WEL.StatusBytesSize = 0x01;
	RDSR_WEL.MatchMode = QSPI_MATCH_MODE_AND;
	RDSR_WEL.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WEL, 35)
			== HAL_ERROR) {
		return MEM_WRITE_TIMEOUT;
	}

	QSPI_CommandTypeDef four_page_program = { 0 };
	four_page_program.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	four_page_program.Instruction = 0x38;
	four_page_program.AddressMode = QSPI_ADDRESS_4_LINES;
	four_page_program.Address = addr;
	four_page_program.AddressSize = QSPI_ADDRESS_24_BITS;
	four_page_program.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	four_page_program.DataMode = QSPI_DATA_4_LINES;
	four_page_program.NbData = len_for_cur_page;
	four_page_program.DummyCycles = 0;
	four_page_program.DdrMode = QSPI_DDR_MODE_DISABLE;
	four_page_program.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	//four_page_program.SIOOMode = QSPI_SIOO_INST_ONLY_FIRST_CMD;

	HAL_QSPI_Command(&hqspi, &four_page_program, HAL_MAX_DELAY);

	HAL_QSPI_Transmit(&hqspi, data_w, HAL_MAX_DELAY);

	//update values
	len -= len_for_cur_page;
	addr += len_for_cur_page;
	if (len == 0) {
		return MEM_WRITE_SUCCESS;
	} else {
		QSPI_AutoPollingTypeDef RDSR_WIP;
		RDSR_WIP.Match = 0x00;
		RDSR_WIP.Mask = 0x01;
		RDSR_WIP.Interval = 5;
		RDSR_WIP.StatusBytesSize = 0x01;
		RDSR_WIP.MatchMode = QSPI_MATCH_MODE_AND;
		RDSR_WIP.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
		if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WIP, 35) == HAL_ERROR) {
			return MEM_ERASE_TIMEOUT;
		} else {
			data_w += len_for_cur_page;
			mem_write(addr, data_w, len);
		}
	}
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
	QSPI_CommandTypeDef write_enable = {0};
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

	QSPI_CommandTypeDef read_status_register = {0};
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

	if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WEL, 35) == HAL_ERROR) {
		return MEM_ERASE_TIMEOUT;
	}

	QSPI_CommandTypeDef chip_erase = {0};
	chip_erase.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	chip_erase.Instruction = 0xC7;
	chip_erase.AddressMode = QSPI_ADDRESS_NONE;
	chip_erase.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	chip_erase.DataMode = QSPI_DATA_NONE;
	chip_erase.NbData = 0;
	chip_erase.DummyCycles = 0;
	chip_erase.DdrMode = QSPI_DDR_MODE_DISABLE;
	chip_erase.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	HAL_QSPI_Command(&hqspi, &chip_erase, HAL_MAX_DELAY);

	QSPI_AutoPollingTypeDef RDSR_WIP;
	RDSR_WIP.Match = 0x00;
	RDSR_WIP.Mask = 0x01;
	RDSR_WIP.Interval = 5;
	RDSR_WIP.StatusBytesSize = 0x01;
	RDSR_WIP.MatchMode = QSPI_MATCH_MODE_AND;
	RDSR_WIP.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&hqspi, &read_status_register, &RDSR_WIP, 250000) == HAL_ERROR) {
		return MEM_ERASE_TIMEOUT;
	} else {
		return MEM_ERASE_SUCCESS;
	}
}
