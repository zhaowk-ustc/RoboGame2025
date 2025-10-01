#ifndef ROBOARM_MOTION_H
#define ROBOARM_MOTION_H

#include "zf_common_headfile.h"

#define DI_PWM (ATOM0_CH2_P00_3)
#define DABI_PWM (ATOM0_CH3_P00_4)
#define ZHONGBI_PWM (ATOM0_CH4_P00_5)
#define XIAOBI_PWM (ATOM0_CH5_P00_6)
#define SHOUWAN_PWM (ATOM0_CH6_P00_7)
#define GRIPPER_PWM (ATOM0_CH7_P00_8)

// 预设位置参数
#define GRIPPER_CLOSE 460
#define GRIPPER_OPEN 250
#define GRIPPER_MOVE_TIME_MS 300 // 爪子单独运动时间
#define SHOUWAN 680

#define VERTICAL_DABI 580

#define PROCESS_ZHONGBI 1160
#define PROCESS_XIAOBI 1100
#define PROCESS2_ZHONGBI 860 // 因为机械限位
#define PROCESS3_ZHONGBI 960 // 也是机械限位

// 运动控制参数
#define SMOOTH_DELAY_MS 20
#define STEP_DELAY_MS 400

// 舵机引脚定义
typedef struct
{
    int di;
    int dabi;
    int zhongbi;
    int xiaobi;
    int shouwan;
    int gripper;
} ServoPositions;

void roboarm_init(void);
void arm_relax(void);
void arm_reset(void);

void arm_reset_to_prepare(void);
void arm_reset_to_high_prepare(void);
void arm_prepare_to_grip(void);
void arm_high_prepare_to_grip(void);
void arm_grip_to_shot(void);
void arm_grip_to_store(void);
void arm_grip_to_wait_shot(void);
void arm_wait_shot_to_shot(void);
void arm_shot_to_reset(void);
void arm_reset_to_store(void);
void arm_store_to_reset(void);
void arm_store_to_shot(void);

void arm_high_grip_to_shot(void);
void arm_high_grip_to_store(void);

#endif // ROBOARM_MOTION_H
