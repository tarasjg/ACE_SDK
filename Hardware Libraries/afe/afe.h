/*
 * afe.h
 *
 *  Created on: Jan 26, 2021
 *      Author: John Tarasidis
 */

#ifndef INC_AFE_H_
#define INC_AFE_H_

void afe_init(void);
void afe_sdatac(void);
void afe_rdatac(void);
void afe_rdata(uint8_t*);
void afe_set_dr(void);
void afe_set_bias(void);
void afe_set_srb(void);

#endif /* INC_AFE_H_ */
