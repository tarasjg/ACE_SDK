/*
 * accel.h
 *
 *  Created on: Jan 26, 2021
 *      Author: John Tarasidis
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include "stm32l4xx_hal.h"

/*
 * ADXL372 registers definition
 */
#define ADXL372_DEVID				0x00u   /* Analog Devices, Inc., accelerometer ID */
#define ADXL372_DEVID_MST			0x01u   /* Analog Devices MEMS device ID */
#define ADXL372_PARTID				0x02u   /* Device ID */
#define ADXL372_REVID				0x03u   /* product revision ID*/
#define ADXL372_STATUS_1			0x04u   /* Status register 1 */
#define ADXL372_STATUS_2			0x05u   /* Status register 2 */
#define ADXL372_FIFO_ENTRIES_2		0x06u   /* Valid data samples in the FIFO */
#define ADXL372_FIFO_ENTRIES_1		0x07u   /* Valid data samples in the FIFO */
#define ADXL372_X_DATA_H			0x08u   /* X-axis acceleration data [11:4] */
#define ADXL372_X_DATA_L			0x09u   /* X-axis acceleration data [3:0] */
#define ADXL372_Y_DATA_H			0x0Au   /* Y-axis acceleration data [11:4] */
#define ADXL372_Y_DATA_L			0x0Bu   /* Y-axis acceleration data [3:0] */
#define ADXL372_Z_DATA_H			0x0Cu   /* Z-axis acceleration data [11:4] */
#define ADXL372_Z_DATA_L			0x0Du   /* Z-axis acceleration data [3:0] */
#define ADXL372_X_MAXPEAK_H			0x15u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADXL372_X_MAXPEAK_L			0x16u   /* X-axis MaxPeak acceleration data [7:0] */
#define ADXL372_Y_MAXPEAK_H			0x17u   /* Y-axis MaxPeak acceleration data [15:8] */
#define ADXL372_Y_MAXPEAK_L			0x18u   /* Y-axis MaxPeak acceleration data [7:0] */
#define ADXL372_Z_MAXPEAK_H			0x19u   /* Z-axis MaxPeak acceleration data [15:8] */
#define ADXL372_Z_MAXPEAK_L			0x1Au   /* Z-axis MaxPeak acceleration data [7:0] */
#define ADXL372_OFFSET_X			0x20u   /* X axis offset */
#define ADXL372_OFFSET_Y			0x21u   /* Y axis offset */
#define ADXL372_OFFSET_Z			0x22u   /* Z axis offset */
#define ADXL372_X_THRESH_ACT_H		0x23u   /* X axis Activity Threshold [15:8] */
#define ADXL372_X_THRESH_ACT_L		0x24u   /* X axis Activity Threshold [7:0] */
#define ADXL372_Y_THRESH_ACT_H		0x25u   /* Y axis Activity Threshold [15:8] */
#define ADXL372_Y_THRESH_ACT_L 		0x26u   /* Y axis Activity Threshold [7:0] */
#define ADXL372_Z_THRESH_ACT_H		0x27u   /* Z axis Activity Threshold [15:8] */
#define ADXL372_Z_THRESH_ACT_L		0x28u   /* Z axis Activity Threshold [7:0] */
#define ADXL372_TIME_ACT			0x29u   /* Activity Time */
#define ADXL372_X_THRESH_INACT_H	0x2Au   /* X axis Inactivity Threshold [15:8] */
#define ADXL372_X_THRESH_INACT_L	0x2Bu   /* X axis Inactivity Threshold [7:0] */
#define ADXL372_Y_THRESH_INACT_H    0x2Cu   /* Y axis Inactivity Threshold [15:8] */
#define ADXL372_Y_THRESH_INACT_L    0x2Du   /* Y axis Inactivity Threshold [7:0] */
#define ADXL372_Z_THRESH_INACT_H	0x2Eu   /* Z axis Inactivity Threshold [15:8] */
#define ADXL372_Z_THRESH_INACT_L	0x2Fu   /* Z axis Inactivity Threshold [7:0] */
#define ADXL372_TIME_INACT_H       	0x30u   /* Inactivity Time [15:8] */
#define ADXL372_TIME_INACT_L        0x31u   /* Inactivity Time [7:0] */
#define ADXL372_X_THRESH_ACT2_H     0x32u   /* X axis Activity2 Threshold [15:8] */
#define ADXL372_X_THRESH_ACT2_L		0x33u   /* X axis Activity2 Threshold [7:0] */
#define ADXL372_Y_THRESH_ACT2_H     0x34u   /* Y axis Activity2 Threshold [15:8] */
#define ADXL372_Y_THRESH_ACT2_L     0x35u   /* Y axis Activity2 Threshold [7:0] */
#define ADXL372_Z_THRESH_ACT2_H		0x36u   /* Z axis Activity2 Threshold [15:8] */
#define ADXL372_Z_THRESH_ACT2_L		0x37u   /* Z axis Activity2 Threshold [7:0] */
#define ADXL372_HPF					0x38u   /* High Pass Filter */
#define ADXL372_FIFO_SAMPLES        0x39u   /* FIFO Samples */
#define ADXL372_FIFO_CTL	    	0x3Au   /* FIFO Control */
#define ADXL372_INT1_MAP			0x3Bu   /* Interrupt 1 mapping control */
#define ADXL372_INT2_MAP       		0x3Cu   /* Interrupt 2 mapping control */
#define ADXL372_TIMING	       		0x3Du   /* Timing */
#define ADXL372_MEASURE				0x3Eu   /* Measure */
#define ADXL372_POWER_CTL       	0x3Fu   /* Power control */
#define ADXL372_SELF_TEST       	0x40u   /* Self Test */
#define ADXL372_RESET           	0x41u   /* Reset */
#define ADXL372_FIFO_DATA			0x42u   /* FIFO Data */

#define ADXL372_DEVID_VAL       	0xADu   /* Analog Devices, Inc., accelerometer ID */
#define ADXL372_MST_DEVID_VAL   	0x1Du   /* Analog Devices MEMS device ID */
#define ADXL372_PARTID_VAL      	0xFAu   /* Device ID */
#define ADXL372_REVID_VAL       	0x02u   /* product revision ID*/
#define ADXL372_RESET_CODE			0x52u	/* Writing code 0x52 resets the device */


typedef struct gpio_pins {
	GPIO_TypeDef* port;
	uint16_t pin;
} GPIO_Pin;

typedef struct spi_comms {
	SPI_HandleTypeDef* handle;
	GPIO_Pin* cs;
} SPI_Comm;



uint8_t reg_read(SPI_Comm, uint8_t);
void reg_write(SPI_Comm, uint8_t, uint8_t);

#endif /* INC_ACCEL_H_ */
