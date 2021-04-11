/*
 * afe.c
 *
 *  Created on: Jan 26, 2021
 *      Author: John Tarasidis
 */
#include "main.h"
#include "afe.h"

extern SPI_HandleTypeDef hspi3;

static void afe_write_reg(uint8_t reg, uint8_t value) {
	uint8_t cmd_seq[3] = {0x40 | reg, 0x00, value};
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
	HAL_SPI_Transmit(&hspi3, cmd_seq, 3, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
}

static void afe_write_ch(uint8_t value) {
	uint8_t cmd_seq[] = {0b01000101, 0x09, value, value, value, value,
										   value, value, value, value};
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
	HAL_SPI_Transmit(&hspi3, cmd_seq, 10, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
}

void afe_init(void) {
	HAL_Delay(200); //tpor
	HAL_GPIO_TogglePin(ADS_RST_GPIO_Port, ADS_RST_Pin);
	HAL_Delay(1);
	HAL_GPIO_TogglePin(ADS_RST_GPIO_Port, ADS_RST_Pin);
	HAL_Delay(1);
	afe_sdatac(); 	//enable reg writes
	afe_write_reg(0x03, 0xE0);
	HAL_Delay(10);
	afe_write_ch(0b01100001);
	HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_SET); //start conversion
	afe_rdatac();
	afe_sdatac();
	afe_write_reg(0x02, 0xC0);
	afe_write_ch(0b01100000);
	afe_set_dr();	//set data rate 1000kSps
	afe_set_srb();	//mux reference to CHn
	afe_set_bias(); //enable internal reference, enable bias amp, mux CHp&n to bias inverting
	afe_rdatac();



	//afe_write_ch(0b01100000); //set gain of 24, enable normal electrode input

	//afe_rdatac();
}

void afe_sdatac(void) {
	uint8_t sdatac = 0b00010001;
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
	HAL_SPI_Transmit(&hspi3, &sdatac, 1, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
}

void afe_rdatac(void) {
	uint8_t rdatac = 0b00010000;
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
	HAL_SPI_Transmit(&hspi3, &rdatac, 1, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
}

void afe_rdata(uint8_t* sample) {
	//uint8_t rdata = 0b00010010;
	uint8_t zero[27] = {0};
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
	//HAL_SPI_Transmit(&hspi3, &rdata, 1, HAL_MAX_DELAY);//read data
	HAL_SPI_TransmitReceive(&hspi3, zero, sample, 27, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(ADS_CS_GPIO_Port, ADS_CS_Pin);
}

void afe_set_dr(void) {
	afe_write_reg(0x01, 0b10010101);
}

void afe_set_bias(void) {
	afe_write_reg(0x0D, 0xFF); //route all 8 channels positive to bias inverting input
	afe_write_reg(0x0E, 0xFF); //route all 8 channels negative to bias inverting,
							   //which is SRB1 for all channels
	afe_write_reg(0x03, 0b11101100); //enable bias amp and internal ref, route internal ref to bias
}

void afe_set_srb(void) {
	afe_write_reg(0x15, 0b00100000);
}
