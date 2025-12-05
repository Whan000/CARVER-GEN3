/*
 * PWM.h
 *
 *  Created on: Mar 31, 2024
 *      Author: beamk
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"
#include "math.h"



typedef struct {

    TIM_HandleTypeDef* htimx; // Timer Pointer
    uint16_t tim_chx; // Timer Channel
    uint32_t period_cyc ; //The number of timer ticks in one PWM cycle
    uint16_t prescaler ; //Prescaler
    uint16_t overflow ; //ARR: Auto Reload Register (Counter Period)
    uint32_t CPU_FREQ ; // CPU frequency
    uint32_t OC;

} PWM;

// Function prototypes
void PWM_init(PWM* pwm, TIM_HandleTypeDef* htimx, uint16_t tim_chx);
void PWM_write_duty(PWM* pwm, float freq, float percent_duty);
void PWM_write_range(PWM* pwm, float freq, float duty);

#endif /* INC_PWM_H_ */
