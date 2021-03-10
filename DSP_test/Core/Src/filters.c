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
#define BP_NUM_TAPS			29

static float32_t bp_fir_state[BLOCK_SIZE + BP_NUM_TAPS - 1];
const float32_t bp_fir_coeffs[BP_NUM_TAPS] = {
		-0.000000f, 0.003130f, 0.006635f, 0.007290f, -0.000000f, -0.015670f,
		-0.030045f, -0.027445f, -0.000000f, 0.040380f, 0.065543f, 0.051462f,
		-0.000000f, -0.057857f, 0.913154f, -0.057857f, -0.000000f, 0.051462f,
		0.065543f, 0.040380f, -0.000000f, -0.027445f, -0.030045f, -0.015670f,
		-0.000000f, 0.007290f, 0.006635f, 0.003130f, -0.000000f
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





