/*
 * shot.h
 *
 *  Created on: 2025年9月13日
 *      Author: Pluviophile-TAO
 */

#ifndef CODE_SHOT_H_
#define CODE_SHOT_H_

#include "zf_common_headfile.h"

#define SHOT_FRONT_LEFT (ATOM2_CH2_P11_3)
#define SHOT_FRONT_RIGHT (ATOM2_CH1_P11_2)
#define SHOT_BACK_LEFT (ATOM2_CH6_P13_1)
#define SHOT_BACK_RIGHT (ATOM2_CH5_P13_0)

#define ESC_THROTTLE_MIN        (1000)                            // 最小油门值 (对应1000us脉宽)
#define ESC_THROTTLE_MAX        (2000)                            // 最大油门值 (对应2000us脉宽)
#define ESC_THROTTLE_IDLE       (1000)                            // 怠速油门值

// 固定油门值 - 设置为稳定运行值 (仅用于初始化)
#define ESC_FIXED_THROTTLE      (1400)                            // 初始油门值，1500是中值

//发射架舵机
#define LUANCH_SERVOR (ATOM0_CH1_P00_1)
#define PWM_LUANCH_SERVOR_PLACE 300
#define PWM_LUANCH_SERVOR_SHOT 1000

// ------------------ PWM占空比计算方式 ------------------
//
// 标准PWM协议: 1000us-2000us 脉宽对应 0-100% 油门
// 在50Hz PWM下，周期为20000us
// 占空比计算: duty = (pulse_width / 20000) * PWM_DUTY_MAX
// 例如: 1000us -> 5%占空比, 1500us -> 7.5%占空比, 2000us -> 10%占空比
//
// ------------------ PWM占空比计算方式 ------------------
#define PWM_FREQ                (50)                              // PWM频率，单位Hz
#define PWM_PERIOD_US           (20000)                           // PWM周期，单位us
// 脉宽转换为占空比 - 确保计算准确
#define ESC_PULSE_TO_DUTY(pulse_width)   ((pulse_width * PWM_DUTY_MAX) / PWM_PERIOD_US)

extern const pwm_channel_enum channel_list[4];  // 改为声明

/* ===================== 外部变量声明 ===================== */
// 4个PWM端口的油门值数组
extern uint16_t esc_throttle[4];  // 改为声明，去掉初始化

/* ===================== 函数声明 ===================== */
void bldc_lauch_pwm_init(void);
void bldc_init(void);
void bldc_set_speed(uint16_t throttle);
void launch_servor_place(void);
void launch_servor_shot(void);

#endif /* CODE_SHOT_H_ */
