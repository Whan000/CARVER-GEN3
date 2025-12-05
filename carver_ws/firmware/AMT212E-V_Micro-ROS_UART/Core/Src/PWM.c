/*
 * PWM.c
 *
 *  Created on: Mar 31, 2024
 *      Author: beamk
 */

#include "PWM.h"

void PWM_init(PWM* pwm, TIM_HandleTypeDef* htimx, uint16_t tim_chx){
	pwm->CPU_FREQ = 170e6;
	pwm->htimx = htimx;
	pwm->tim_chx = tim_chx;
	pwm->OC = 0;
	HAL_TIM_Base_Start(htimx);
	HAL_TIM_PWM_Start(htimx, tim_chx);
}

void PWM_write_duty(PWM* pwm, float freq, float percent_duty){
    if (freq == 0) {
        __HAL_TIM_SET_COMPARE(pwm->htimx, pwm->tim_chx, 0);
    } else {
    	pwm->period_cyc = (uint32_t) (pwm->CPU_FREQ / freq);
    	pwm->prescaler = (uint16_t) ((pwm->period_cyc + 65535 - 1) / 65535) - 1; // Subtracting 1 since prescaler is zero-based
    	pwm->overflow = (uint16_t) ((pwm->CPU_FREQ / (float)(pwm->prescaler + 1) / freq) - 1);
    	pwm->OC = (uint16_t) (pwm->overflow * fabs(percent_duty) / 100.0);
        __HAL_TIM_SET_PRESCALER(pwm->htimx, pwm->prescaler);
        __HAL_TIM_SET_AUTORELOAD(pwm->htimx, pwm->overflow);
        __HAL_TIM_SET_COMPARE(pwm->htimx, pwm->tim_chx, pwm->OC);
    }
}

void PWM_write_range(PWM* pwm, float freq, float duty){
    if (freq == 0) {
        __HAL_TIM_SET_COMPARE(pwm->htimx, pwm->tim_chx, 0);
    } else {
    	pwm->period_cyc = (uint32_t) (pwm->CPU_FREQ / freq);
    	pwm->prescaler = (uint16_t) ((pwm->period_cyc + 65535 - 1) / 65535) - 1; // Subtracting 1 since prescaler is zero-based
    	pwm->overflow = (uint16_t) ((pwm->CPU_FREQ / (float)(pwm->prescaler + 1) / freq) - 1);
    	pwm->OC = (uint16_t) (pwm->overflow * fabs(duty) / 65535.0);
        __HAL_TIM_SET_PRESCALER(pwm->htimx, pwm->prescaler);
        __HAL_TIM_SET_AUTORELOAD(pwm->htimx, pwm->overflow);
        __HAL_TIM_SET_COMPARE(pwm->htimx, pwm->tim_chx, pwm->OC);
    }
}
