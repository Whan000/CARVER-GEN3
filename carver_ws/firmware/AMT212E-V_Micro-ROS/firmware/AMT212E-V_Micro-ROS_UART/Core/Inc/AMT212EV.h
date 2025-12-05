/*
 * AMT212EV.h
 *
 *  Created on: Sep 21, 2024
 *      Author: itim_
 */

#ifndef INC_AMT212EV_H_
#define INC_AMT212EV_H_

#include "main.h"
#include "stdbool.h"
#include "MathOperation.h"

typedef struct{
	UART_HandleTypeDef *huartx; // UART for Transmit and Receive the data (** Use USART1/2/3 **)
	uint16_t freq;
	int16_t cpr; // Count per revolute (12-bit: 4095, 14-bit: 16383)
	uint8_t cmd; // To get the data from an AMT21XX-X absolute encoder
	uint32_t timeout; // Debugger to show timeout when checksum has failed
	int16_t c[2]; // Dummy count of encoder
	int16_t raw_count; // Raw current count of encoder
	int16_t left_limit; // Steering left side limit (raw count)
	int16_t right_limit; // Steering right side limit (raw count)
	int16_t enc_home_pos; // Endcoder home position (raw count)
	int8_t dir;
	int32_t diff_count;
	int32_t pulses;
	float degree; // Angle position of encoder
	float revs;
	float rads;
	float pps;
	float radps;
	float rpm;
	uint8_t rx[2]; // Array to receive the data from encoder
	uint8_t error_caught; // Debugger to show an error when transmission has failed

}AMT212EV;

enum{
	READPOS,
	SETZERO
};

// Initial the encoder profile
void AMT212EV_Init(AMT212EV *amt212ev ,UART_HandleTypeDef *huartx, uint16_t freq,int16_t cpr);

// Flip an encoder
void AMT212EV_Flip(AMT212EV *amt212ev);

// Read position
void AMT212EV_ReadPosition(AMT212EV *amt212ev);

// Set value of encoder as zero at current position of an encoder
void AMT212EV_SetZero(AMT212EV *amt212ev);

// Process the data and return the data depend on the command you've sent. (in case, it has only position)
uint16_t AMT212EV_Processing_Data(AMT212EV *amt212ev);

// Inspect data
void AMT212EV_Compute(AMT212EV *amt212ev);

// Compute the data
void AMT212EV_DiffCount(AMT212EV *amt212ev);

// Software limit setting and Home position (Raw)
void AMT212EV_SetLimit(AMT212EV *amt212ev, int16_t left_limit,int16_t right_limit,int16_t enc_home_pos);

// Encoder Reset
void AMT212EV_Reset(AMT212EV *amt212ev);

// Boolean for checksum data that make sure the data is correct
bool verifyChecksumRS485(uint16_t currentPosition);

// Mapping value
int16_t map(int16_t value, int16_t input_min, int16_t input_max, int16_t output_min, int16_t output_max);


#endif /* INC_AMT212EV_H_ */
