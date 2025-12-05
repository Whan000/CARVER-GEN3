/*
 * Calculator.c
 *
 *  Created on: May 1, 2024
 *      Author: User
 */

#include <MathOperation.h>

// include math.h already
// Calculate average value

float calculate_average(uint16_t *array, uint32_t length) {
	uint32_t sum = 0;
	for (uint32_t i = 0; i < length; i++) {
		sum += array[i];
	}
	return length > 0 ? (float) sum / length : 0.0f;

}

//Scaling a range of value
float mapf(float input, float min_input, float max_input, float min_output,
		float max_output) {

	// First, find the ratio of the input within the input range
	float input_ratio = (input - min_input) / (max_input - min_input);
	// Then, scale this ratio to the output range
	float output = (input_ratio * (max_output - min_output)) + min_output;

	return output;
}
