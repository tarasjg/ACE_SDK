/*
 * accel.c
 *
 *  Created on: Jan 26, 2021
 *      Author: John Tarasidis
 */


#include "accel.h"

//extern SPI_HandleTypeDef hspi1;

static void reg_write(SPI_Comm, uint8_t, uint8_t);
static void fifo_init(SPI_Comm, int);
static void int1_init(SPI_Comm);

void reg_read_IT(SPI_Comm spi, uint8_t addr, uint8_t* rx, size_t size)
{
	uint8_t tx = (addr << 1) | 0x01;

	HAL_GPIO_WritePin(spi.cs->port, spi.cs->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_SPI_Receive_IT(spi.handle, rx, size);
	return;
}

void reg_read(SPI_Comm spi, uint8_t addr, uint8_t* rx, size_t size)
{
	uint8_t tx = (addr << 1) | 0x01;

	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_SPI_Receive(spi.handle, rx, size, HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);

	return;
}

void reg_read_CS(SPI_Comm spi, uint8_t addr, uint8_t* rx, size_t size)
{
	uint8_t tx = (addr << 1) | 0x01;

	HAL_GPIO_WritePin(spi.cs->port, spi.cs->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_SPI_Receive(spi.handle, rx, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(spi.cs->port, spi.cs->pin, GPIO_PIN_SET);

	return;
}

static void reg_write(SPI_Comm spi, uint8_t addr, uint8_t payload)
{
	uint8_t tx[] = {(addr << 1), payload};

	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);
	HAL_SPI_Transmit(spi.handle, (uint8_t *)&tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(spi.cs->port, spi.cs->pin);

	return;
}


static void fifo_init(SPI_Comm spi, int samples) {
	uint8_t samples_MSB = samples >> 8;
	uint8_t samples_LSB = samples & 0xFF;
	uint8_t payload_FIFO_CTL = samples_MSB | 0x02;
	reg_write(spi, ADXL372_FIFO_SAMPLES, 0b11111000); //0b1_00000000 -> 256 samples
	reg_write(spi, ADXL372_FIFO_CTL, 0b00000011); //0b00000011 -> stream mode; sample MSB
}

static void int1_init(SPI_Comm spi) {
	reg_write(spi, ADXL372_INT1_MAP, 0b00000100); //0b00000100 -> INT1 on FIFO Full Condition
}

void stream_start(SPI_Comm spi, int samples) {
	reg_write(spi, 0x41, 0x52);
	HAL_Delay(100);
	reg_write(spi, ADXL372_POWER_CTL, 0x00); // set standby mode before changing settings
	reg_write(spi, 0x3D, 0x00);
	reg_write(spi, 0x3E, 0b00000100);
	fifo_init(spi, samples);
	int1_init(spi);
	uint8_t r = {0};
	reg_read(spi, 0x3D, &r, 1);
	reg_read(spi, 0x3E, &r, 1);
	reg_write(spi, ADXL372_POWER_CTL, 0b00001111); //full bandwidth mode; HPF/LPF enabled
	HAL_Delay(370);
}

void fifo_data(SPI_Comm spi, uint8_t* data, size_t data_size) {
	uint8_t dummy = 0;
	uint8_t fifo_entries_msb = 0;
	uint8_t fifo_entries_lsb = 0;
	reg_read(spi, 0x06, &fifo_entries_msb, 1);
	reg_read(spi, 0x07, &fifo_entries_lsb, 1);
	uint16_t valid = fifo_entries_msb << 8 | fifo_entries_lsb;
	reg_read(spi, ADXL372_FIFO_DATA, data, data_size);
	reg_read(spi, ADXL372_STATUS_1, &dummy, sizeof(dummy));
}


