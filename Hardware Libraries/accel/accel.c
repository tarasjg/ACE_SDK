/*
 * accel.c
 *
 *  Created on: Jan 26, 2021
 *      Author: John Tarasidis
 */


#include "accel.h"

extern hspi2;

GPIO_Pin chip_select = {
		.port = GPIOB,
		.pin = GPIO_PIN_6
};

SPI_Comm accel_spi = {
		.handle = &hspi2,
		.cs = &chip_select
};



uint8_t reg_read(SPI_Comm spi, uint8_t addr)
{
	uint8_t rx; //Data to receive
	uint8_t tx = (addr << 1) | 0x01;

	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_SPI_Receive(spi.handle, (uint8_t *)&rx, sizeof(rx), HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);

	return rx;
}

void reg_write(SPI_Comm spi, uint8_t addr, uint8_t payload)
{

	uint8_t tx[] = {(addr << 1), payload};

	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);
}

//dat_boi.bitch = 5;
