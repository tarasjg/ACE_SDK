/*
 * selftest.c
 *
 *  Created on: Jan 31, 2021
 *      Author: John Tarasidis
 */


/*******************************
Discussion and Theory of Operation:

This self test program is designed to NOT rely on any other tool in
the ACE SDK.  Rather, only ST HAL.  Primarily, this is done to test
the device prior to the completion of the SDK.  Upon SDK completion,
modes.c will feature a selftest mode, that will enter and return to
idle after reporting success/failure. (note made on 2/1)

The self test tests the following things:

	The ADXL372 Accelerometer, via SPI1
	The ADS1299 Analog front-end, via SPI3
	The MX25R64 Flash Memory, via QSPI
	The notification LEDs
	The analog front-end power control
	The external communication ports, USART and USB

Upon start, the STM32 initializes SPI1 and SPI3, as well as enables
APWR_EN and PWDN to ungate the ADS1299.  Following, it queries device
ID from both the ADXL372 via SPI1 and the ADS1299 via SPI3.  Next,
APWR_EN and PWDN with be gated, effectively cutting power to the
front-end.  The ADS1299 device ID will be queried again, this time
where no repsonse is a pass.  The ADXL372 and MX25R64 share a bus, so
SPI1 must be deinitialized, and QSPI must be initialized.  The device
does not support concurrent use between these two devices.  Once this
is done, device ID is queried of the MX25R64 flash memory.  LEDs are
then utilized to report test results before serial communication is
set to the external PC.  A self test bit field contains all of the
test information.  It is as follows:

				xxxx0000; MSB first

bit 3 - MX25R64 query ID success
bit[2:1] - ADS1299 power control success, ADS1299 query ID success
bit 0 - ADXL372 query ID success

LEDs will flash the code, where on for 500ms is a 1 (success), and two
flashes (125ms on, 125ms off) is a 0.  1000ms indicates a break in between bits.

Reserved bits are not reported.  Thus, for xxxx0101, the sequence would be:

						0									1					...
on (125ms) off (125ms) on (125ms) off (125ms) (1000ms) on (500ms) off (1000ms), etc

Once reported via the LEDs, this single byte will be transmitted via
both USART and USB, every second, alternating, for one minute.

*******************************/