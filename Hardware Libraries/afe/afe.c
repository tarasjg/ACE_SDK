/*
 * afe.c
 *
 *  Created on: Jan 26, 2021
 *      Author: John Tarasidis
 */
#include "main.h"

extern SPI_HandleTypeDef hspi3;

void afe_init(void) {

}

static void afe_write_reg(uint8_t reg, uint8_t value) {
	uint8_t cmd_seq[2] = {0x20 || reg, 0x01, value};

}

static void afe_write_ch(uint8_t value) {
	uint8_t cmd_seq[] = {0b01000101, 0x09, value, value, value, value,
										   value, value, value, value};

	};
}

void afe_sdatac(void) {
	uint8_t sdatac = 0b00010001;

}

void afe_rdatac(void) {
	uint8_r rdatac = 0b00010000;
}

uint8_t * afe_rdata(void) {
	uint8_t rdata = 0b00010010;
	uint8_t sample[27] = {0};
}

void afe_set_dr(void) {

}

void afe_set_bias(void) {

}

void afe_set_srb(void) {

}
