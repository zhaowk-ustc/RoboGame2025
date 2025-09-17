#ifndef PUSH_H
#define PUSH_H

#include "zf_common_headfile.h"

/* ===================== 硬件引脚定义 ===================== */
#define DIRECTION_PIN P13_2
#define SPEED_PWM ATOM3_CH0_P13_3

#define ENCODER_MODULE (TIM2_ENCODER)
#define ENCODER_A_PIN (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_B_PIN (TIM2_ENCODER_CH2_P33_6)

/* ===================== 运动控制参数 ===================== */
#define TARGET_POSITION 6000 // 目标位置（编码器计数）
#define MOVE_SPEED 6000      // PWM占空比（速度）
#define DIRECTION_FORWARD GPIO_HIGH
#define DIRECTION_BACKWARD GPIO_LOW

/* ===================== 时间参数 ===================== */
#define LOOP_DELAY_MS 100      // 主循环延迟时间
#define ENDPOINT_DELAY_MS 1000 // 起点/终点暂停时间

/* ===================== 类型定义 ===================== */

#define MODE_STOP 0
#define MODE_FORWARD 1
#define MODE_RETURN 2

typedef uint8_t MovementMode;

/* ===================== 外部变量声明 ===================== */
extern int16 encoder_count;
extern MovementMode current_mode;

/* ===================== 函数声明 ===================== */

void push_init(void);
void push_update(void);
void push_stop(void);

#endif // PUSH_H
