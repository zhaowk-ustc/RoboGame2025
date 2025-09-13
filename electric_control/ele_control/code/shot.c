/*
 * shot.c
 *
 *  Created on: 2025年9月13日
 *      Author: Pluviophile-TAO
 */
#include "shot.h"

// 定义变量（在.c文件中）
const pwm_channel_enum channel_list[4] = {SHOT_FRONT_LEFT, SHOT_FRONT_RIGHT, SHOT_BACK_LEFT, SHOT_BACK_RIGHT};
uint16_t esc_throttle[4] = {0, 0, 0, 0};

void bldc_lauch_pwm_init(void){
    pwm_init(SHOT_FRONT_LEFT, PWM_FREQ, 0);
    pwm_init(SHOT_FRONT_RIGHT, PWM_FREQ, 0);
    pwm_init(SHOT_BACK_LEFT, PWM_FREQ, 0);
    pwm_init(SHOT_BACK_RIGHT, PWM_FREQ, 0);
    pwm_init(LUANCH_SERVOR, PWM_FREQ, 0);
}

void bldc_init(void){
    for(int i = 0; i < 4; i++)
    {
        esc_throttle[i] = ESC_THROTTLE_MIN;
        pwm_set_duty(channel_list[i], ESC_PULSE_TO_DUTY(esc_throttle[i]));
    }
    system_delay_ms(2000);
}

void bldc_set_speed(uint16_t throttle){
    for(int i = 0; i < 4; i++)
    {
        esc_throttle[i] = throttle;
        pwm_set_duty(channel_list[i], ESC_PULSE_TO_DUTY(esc_throttle[i]));
    }
}

void launch_servor_place(void){
    pwm_set_duty(LUANCH_SERVOR, PWM_LUANCH_SERVOR_PLACE);
}

void launch_servor_shot(void){
    pwm_set_duty(LUANCH_SERVOR, PWM_LUANCH_SERVOR_SHOT);
}

