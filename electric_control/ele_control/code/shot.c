/*
 * shot.c
 *
 *  Created on: 2025年9月13日
 *      Author: Pluviophile-TAO
 */
#include "shot.h"
#include "push.h"

// 定义变量（在.c文件中）
const pwm_channel_enum channel_list[4] = {SHOT_FRONT_LEFT, SHOT_FRONT_RIGHT, SHOT_BACK_LEFT, SHOT_BACK_RIGHT};
uint16_t esc_throttle[4] = {0, 0, 0, 0};

void shot_init(void)
{
    pwm_init(SHOT_FRONT_LEFT, PWM_FREQ, 0);
    pwm_init(SHOT_FRONT_RIGHT, PWM_FREQ, 0);
    pwm_init(SHOT_BACK_LEFT, PWM_FREQ, 0);
    pwm_init(SHOT_BACK_RIGHT, PWM_FREQ, 0);
    pwm_init(LUANCH_SERVOR, PWM_FREQ, 0);
}

void bldc_init(void)
{
    for (int i = 0; i < 4; i++)
    {
        esc_throttle[i] = ESC_THROTTLE_MIN;
        pwm_set_duty(channel_list[i], ESC_PULSE_TO_DUTY(esc_throttle[i]));
    }
    system_delay_ms(2000);
}

void bldc_set_speed(uint16_t throttle)
{
    for (int i = 0; i < 4; i++)
    {
        esc_throttle[i] = throttle;
        pwm_set_duty(channel_list[i], ESC_PULSE_TO_DUTY(esc_throttle[i]));
    }
}

// 设置发射架角度函数
void set_launch_angle(float normalized_input)
{
    int pwm_value;

    // 输入范围检查
    if (normalized_input < -1.0f)
    {
        normalized_input = -1.0f;
    }
    else if (normalized_input > 1.0f)
    {
        normalized_input = 1.0f;
    }

    // 计算PWM值
    // -1到1映射到300-1100
    // 公式: pwm = 300 + (normalized_input + 1.0f) * 400
    pwm_value = 300 + (normalized_input + 1.0f) * 400;

    pwm_set_duty(LUANCH_SERVOR, pwm_value);
}

void shot_fire_once(void)
{
    bldc_set_speed(1200);
    push_forward_and_back();
    bldc_set_speed(1000);
}
