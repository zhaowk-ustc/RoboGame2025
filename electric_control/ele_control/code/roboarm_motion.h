#ifndef ROBOARM_MOTION_H
#define ROBOARM_MOTION_H

#include "zf_common_headfile.h"
#include <stdio.h>

// 舵机引脚定义
#define DI_PWM (ATOM0_CH2_P00_3)
#define DABI_PWM (ATOM0_CH3_P00_4)
#define ZHONGBI_PWM (ATOM0_CH4_P00_5)
#define XIAOBI_PWM (ATOM0_CH5_P00_6)
#define SHOUWAN_PWM (ATOM0_CH6_P00_7)
#define GRIPPER_PWM (ATOM0_CH7_P00_8)

// 预设位置参数
#define VERTICAL_DABI 580
#define GRIP_DI 970
#define GRIP_PREPARE_DABI 630
#define GRIP_DABI 680
#define GRIP_PREPARE_ZHONGBI 420
#define GRIP_ZHONGBI 510
#define GRIP_XIAOBI 1060
#define GRIP_SHOUWAN 680
#define PROCESS_ZHONGBI 1000
#define PROCESS2_ZHONGBI 700
#define SHOT_DI 510
#define SHOT_DABI 540
#define SHOT_ZHONGBI 740
#define SHOT_XIAOBI 370
#define SHOT_SHOUWAN 680
#define GRIPPER_CLOSE 460
#define GRIPPER_OPEN 250

// 运动控制参数
#define MOVE_SPEED 5
#define DELAY_MS 20
#define STEP_DELAY_MS 1000

// 舵机位置结构体
typedef struct
{
    int di;
    int dabi;
    int zhongbi;
    int xiaobi;
    int shouwan;
    int gripper;
} ServoPositions;

extern ServoPositions current_pos;

void roboarm_init(void);
void set_all_pwm(int duty);
void test_servo_positions(void);
void move_servo_smoothly(int *current_pos, int target_pos, int pwm_pin);
void delay_step(void);

void arm_init(void);
void arm_relax(void);
void move_to_grip_prepare(void);
void grip_prepare_to_grip(void);
void grip_to_shot(void);
void shot_to_grip_prepare(void);

#endif // ROBOARM_MOTION_H
