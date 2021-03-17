/*
 * filters.c
 *
 * Source code for filters.h
 *
 *  Created on: Mar 9, 2021
 *      Author: Michael Lee
 */


#include <filters.h>

#define BLOCK_SIZE			32
#define BP_NUM_TAPS			28

static float32_t bp_fir_state[BLOCK_SIZE + BP_NUM_TAPS];
static q15_t bp_fir_state_q15[BLOCK_SIZE + BP_NUM_TAPS];
const float32_t bp_fir_coeffs[BP_NUM_TAPS] = {
		-0.000891f, 0.000596f, 0.002751f, 0.004725f, 0.003639f, -0.003719f, -0.017233f,
		-0.030494f, -0.031590f, -0.008658f, 0.042314f, 0.112221f, 0.180248f, 0.222285f,
		0.222285f, 0.180248f, 0.112221f, 0.042314f, -0.008658f, -0.031590f, -0.030494f,
		-0.017233f, -0.003719f, 0.003639f, 0.004725f, 0.002751f, 0.000596f, -0.000891f
};

void bandpass_filter(float32_t *input, float32_t *output, uint32_t sample_length) {
	uint32_t blockSize = BLOCK_SIZE;
	uint32_t numBlocks = sample_length / BLOCK_SIZE;

	uint32_t i;
	arm_fir_instance_f32 S;

	float32_t *inputF32, *outputF32;
	// Initialize input and output buffer pointers
	inputF32 = &input[0];
	outputF32 = &output[0];

	// Call FIR init function to initialize the instance structure
	arm_fir_init_f32(&S, BP_NUM_TAPS, (float32_t *)&bp_fir_coeffs[0], &bp_fir_state[0], blockSize);

	// Call the FIR process function for every blockSize samples
	for (i = 0; i < numBlocks; ++i) {
		arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	}
}

void bandpass_filter_q15(float32_t *input, float32_t *output, uint32_t sample_length) {
	uint32_t blockSize = BLOCK_SIZE;
	uint32_t numBlocks = sample_length / BLOCK_SIZE;

	uint32_t i;
	arm_fir_instance_q15 S;
	arm_status status;

	q15_t inputQ15[sample_length];
	q15_t outputQ15[sample_length];
	arm_float_to_q15(input, &inputQ15[0], sample_length);

	q15_t coeffsQ15[BP_NUM_TAPS];
	arm_float_to_q15((float32_t *)&bp_fir_coeffs[0], &coeffsQ15[0], BP_NUM_TAPS);

	// Call FIR init function to initialize the instance structure
	status = arm_fir_init_q15(&S, BP_NUM_TAPS, (q15_t *)&coeffsQ15[0], &bp_fir_state_q15[0], blockSize);
	if (status == ARM_MATH_ARGUMENT_ERROR) {
		return;
	} else if (status != ARM_MATH_SUCCESS) {
		return;
	}

	q15_t *pInputQ15, *pOutputQ15;
	pInputQ15 = &inputQ15[0];
	pOutputQ15 = &outputQ15[0];
	// Call the FIR process function for every blockSize samples
	for (i = 0; i < numBlocks; ++i) {
		arm_fir_q15(&S, pInputQ15 + (i * blockSize), pOutputQ15 + (i * blockSize), blockSize);
	}

	arm_q15_to_float(&outputQ15[0], output, sample_length);
}

float arm_snr_f32(float *pRef, float *pTest, uint32_t buffSize) {
  float EnergySignal = 0.0, EnergyError = 0.0;
  uint32_t i;
  float SNR;
  int temp;
  int *test;

  for (i = 0; i < buffSize; i++) {
 	  // Checking for a NAN value in pRef array
	  test =   (int *)(&pRef[i]);
      temp =  *test;

	  if (temp == 0x7FC00000) {
	  		return(0);
	  }

	  // Checking for a NAN value in pTest array
	  test =   (int *)(&pTest[i]);
      temp =  *test;

	  if (temp == 0x7FC00000) {
	  		return(0);
	  }
      EnergySignal += pRef[i] * pRef[i];
      EnergyError += (pRef[i] - pTest[i]) * (pRef[i] - pTest[i]);
    }

	// Checking for a NAN value in EnergyError
	test =   (int *)(&EnergyError);
    temp =  *test;

    if (temp == 0x7FC00000) {
  		return(0);
    }


  SNR = 10 * log10 (EnergySignal / EnergyError);

  return (SNR);
}





