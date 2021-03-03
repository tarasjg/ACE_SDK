/*
 * accel.c
 *
 *  Created on: Jan 26, 2021
 *      Author: John Tarasidis
 */


#include "accel.h"

//extern SPI_HandleTypeDef hspi1;




void reg_read(SPI_Comm spi, uint8_t addr, uint8_t* rx)
{
	uint8_t tx = (addr << 1) | 0x01;

	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_SPI_Receive(spi.handle, rx, sizeof(*rx), HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);

	return;
}

void reg_write(SPI_Comm spi, uint8_t addr, uint8_t payload)
{
	uint8_t tx[] = {(addr << 1), payload};

	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);

	return;
}
