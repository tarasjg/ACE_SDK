/*
 * filters.h
 *
 * Holds function prototypes and #define's for filtering
 *
 *  Created on: Mar 9, 2021
 *      Author: Michael Lee
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#define ARM_MATH_CM4
#include <main.h>
#include <arm_math.h>

// Applies pre-selected bandpass filter to passed-in input array
// Result is then placed into passed-in output array
// Input and output arrays should both be longer than sample_length
void bandpass_filter(float32_t *input, float32_t *output, uint32_t sample_length);

// Calculates the signal to noise ratio between reference data and test data
// This snr value should be used to compare to a threshold value
float arm_snr_f32(float *pRef, float *pTest, uint32_t buffSize);

#endif /* INC_FILTERS_H_ */
