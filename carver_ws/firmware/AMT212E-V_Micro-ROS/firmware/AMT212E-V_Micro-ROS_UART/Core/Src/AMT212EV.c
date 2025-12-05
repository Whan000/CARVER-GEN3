/*
 * AMT212EV.c
 *
 *  Created on: Sep 21, 2024
 *      Author: itim_
 */

#include "AMT212EV.h"

/*
 * AMT21XX-X absolute encoder 12/14-bit with 2nd order IIR Low-pass Filter. To utilize, pass the sampling rate,
 * cut-off frequency, coefficient a (input weight), coefficient b (output weight), and count per revolute
 * (total count per revolute) that depend on the resolution.
 */


void AMT212EV_Init(AMT212EV *amt212ev ,UART_HandleTypeDef *huartx,uint16_t freq,int16_t cpr)
{
	amt212ev->huartx = huartx;
	amt212ev->freq = freq;
	amt212ev->cpr = cpr;
	amt212ev->left_limit = 0;
	amt212ev->right_limit = 0;
	amt212ev->dir = 1;
	amt212ev->raw_count = 0;
	amt212ev->enc_home_pos = 0;
	amt212ev->degree = 0;
	amt212ev->c[1] = 0;
	amt212ev->c[0] = 0;
	amt212ev->diff_count = 0;
	amt212ev->pulses = 0;
	amt212ev->revs = 0;
	amt212ev->rads = 0;
	amt212ev->pps = 0;
	amt212ev->radps = 0;
	amt212ev->rpm = 0;
	amt212ev->rx[0] = 0;
	amt212ev->rx[1] = 0;
	amt212ev->timeout = 0;
	amt212ev->error_caught = 0;

	// Start using UART DMA
	HAL_UART_Receive_DMA(huartx, amt212ev->rx, sizeof(amt212ev->rx));



}

void AMT212EV_Flip(AMT212EV *amt212ev){
	amt212ev->dir = -1;
}

void AMT212EV_ReadPosition(AMT212EV *amt212ev){

	// Arrays to command an encoder
	uint8_t read_pos[1] = {0x54}; // Read Raw count data

		if (HAL_UART_Transmit_DMA(amt212ev->huartx, read_pos, sizeof(read_pos)) != HAL_OK)
		{
			// Handle transmission error if necessary
			amt212ev->error_caught = READPOS;
		}

}



void AMT212EV_SetZero(AMT212EV *amt212ev){
	uint8_t set_zero[2] = {0x56, 0x5E}; // Set zero count directly to the encoder

	if (HAL_UART_Transmit_DMA(amt212ev->huartx, set_zero, sizeof(set_zero)) != HAL_OK){

		// Handle transmission error if necessary
		 amt212ev->error_caught = SETZERO;

	}
}


uint16_t AMT212EV_Processing_Data(AMT212EV *amt212ev){
	 // Extract encoder data
	    uint16_t data = (amt212ev->rx[1] << 8) | amt212ev->rx[0];

	    // Verify the checksum
	    if (verifyChecksumRS485(data)) {
	    	if(amt212ev->cpr == 4096){
	    		 return data &= 0x0FFF;  // Mask to keep only 12 bits (valid data)

	    	}

	    	else if(amt212ev->cpr == 16384){
	    		 return data &= 0x3FFF; // Mask to keep only 14 bits (valid data)
	    	}

	        return data;

	    } else {
	        // Handle checksum error by counting timeout
	    	amt212ev->timeout += 1;
	    }
}


void AMT212EV_DiffCount(AMT212EV *amt212ev){

	amt212ev->raw_count = AMT212EV_Processing_Data(amt212ev);

	amt212ev->c[1] = (amt212ev->raw_count - amt212ev->enc_home_pos) * amt212ev->dir;

	// Difference of Raw Position
	int32_t diff_count = amt212ev->c[1] - amt212ev->c[0];

	if (diff_count > (amt212ev->cpr / 2)) {
		diff_count -= amt212ev->cpr;
	} else if (diff_count < -(amt212ev->cpr / 2)) {
		diff_count += amt212ev->cpr;
	}
	amt212ev->diff_count = diff_count;
	amt212ev->pulses += amt212ev->diff_count;
	amt212ev->revs += amt212ev->diff_count / (float)amt212ev->cpr;
	amt212ev->degree += ((amt212ev->diff_count * 360.0) /(float) amt212ev->cpr) ;
	amt212ev->rads += ((amt212ev->diff_count * 2.0 * M_PI) /(float) amt212ev->cpr);
	amt212ev->c[0] = amt212ev->c[1];

}

void AMT212EV_Compute(AMT212EV *amt212ev){


	amt212ev->pps = (amt212ev->diff_count * (float)amt212ev->freq) ; // Pulse per second
	amt212ev->radps = amt212ev->pps * 2 * M_PI / (float)amt212ev->cpr;; // Radian per second
	amt212ev->rpm = amt212ev->pps * 60.0 / (float)amt212ev->cpr; // Round per second



}

void AMT212EV_SetLimit(AMT212EV *amt212ev, int16_t left_limit,int16_t right_limit ,int16_t enc_home_pos){
	// Store the Limit each side and center position (count)
	amt212ev->left_limit = left_limit * amt212ev->dir ;
	amt212ev->right_limit = right_limit * amt212ev->dir ;
	amt212ev->enc_home_pos = enc_home_pos * amt212ev->dir;


}

void AMT212EV_Reset(AMT212EV *amt212ev){
	// Reset Encoder Data
		amt212ev->pps = 0;
		amt212ev->rpm = 0;
		amt212ev->radps = 0;
		amt212ev->pulses = 0;
		amt212ev->revs = 0;
		amt212ev->rads = 0;

}



bool verifyChecksumRS485(uint16_t currentPosition) {
    uint16_t checksum = 0x3;

    // XOR 2-bit pairs
    for (int i = 0; i < 14; i += 2) {
        checksum ^= (currentPosition >> i) & 0x3;
    }
    return (checksum == (currentPosition >> 14));
}

int16_t map(int16_t value, int16_t input_min, int16_t input_max, int16_t output_min, int16_t output_max) {
    // Prevent division by zero
    if (input_max == input_min) {
        return output_min;
    }
    // Perform the mapping
    return (value - input_min) * (output_max - output_min) / (input_max - input_min) + output_min;
}


