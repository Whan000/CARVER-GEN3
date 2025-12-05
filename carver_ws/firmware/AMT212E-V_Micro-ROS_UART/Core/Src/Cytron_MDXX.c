/*
 * Cytron_MDXX.c
 *
 *  Created on: Mar 31, 2024
 *      Author: beamk
 */
#include "Cytron_MDXX.h"

void MDXX_init(MDXX* mdxx, TIM_HandleTypeDef* htimd, uint16_t timd_chx, TIM_HandleTypeDef* htimp, uint16_t timp_chx){
	mdxx->htimd = htimd;
	mdxx->htimp = htimp;
	mdxx->timd_chx = timd_chx;
	mdxx->timp_chx = timp_chx;
	PWM_init(&(mdxx->dir), htimd, timd_chx);
	PWM_init(&(mdxx->pwm), htimp, timp_chx);

}

void MDXX_set_duty(MDXX* mdxx, float freq, float duty){
	if(duty==0){
		PWM_write_duty(&(mdxx->dir),freq, 0);
		PWM_write_duty(&(mdxx->pwm),freq, 0);
		mdxx->cmd = 0;
	}
	else if(duty>0){
		PWM_write_duty(&(mdxx->dir),freq, 0);
		PWM_write_duty(&(mdxx->pwm),freq, duty);
		mdxx->cmd = duty;
	}
	else{
		PWM_write_duty(&(mdxx->dir),freq, 100);
		PWM_write_duty(&(mdxx->pwm),freq, fabs(duty));
		mdxx->cmd = fabs(duty);
	}
}

void MDXX_set_range(MDXX* mdxx, float freq, float duty){
	if(duty==0){
		PWM_write_range(&(mdxx->dir),freq, 0);
		PWM_write_range(&(mdxx->pwm),freq, 0);
		mdxx->cmd = 0;
	}
	else if(duty>0){
		PWM_write_range(&(mdxx->dir),freq, 0);
		PWM_write_range(&(mdxx->pwm),freq, duty);
		mdxx->cmd = duty;
	}
	else {
		PWM_write_range(&(mdxx->dir),freq, 65535);
		PWM_write_range(&(mdxx->pwm),freq, fabs(duty));
		mdxx->cmd = fabs(duty);
	}
}
