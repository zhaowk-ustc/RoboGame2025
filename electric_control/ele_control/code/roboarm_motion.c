#include "roboarm_motion.h"
#include <stdio.h>
#include "zf_common_headfile.h"

// 舵机引脚定义
#define DI_PWM (ATOM0_CH2_P00_3)
#define DABI_PWM (ATOM0_CH3_P00_4)
#define ZHONGBI_PWM (ATOM0_CH4_P00_5)
#define XIAOBI_PWM (ATOM0_CH5_P00_6)
#define SHOUWAN_PWM (ATOM0_CH6_P00_7)
#define GRIPPER_PWM (ATOM0_CH7_P00_8)

// 预设位置参数
#define GRIPPER_CLOSE 460
#define GRIPPER_OPEN 250

#define VERTICAL_DABI 580
#define GRIP_DI 970
#define GRIP_PREPARE_DABI 626
#define GRIP_DABI 670
#define GRIP_PREPARE_ZHONGBI 430
#define GRIP_ZHONGBI 520
#define GRIP_XIAOBI 1060
#define GRIP_SHOUWAN 680
#define PROCESS_ZHONGBI 1000
#define PROCESS2_ZHONGBI 700
#define SHOT_DI 510
#define SHOT_DABI 540
#define SHOT_ZHONGBI 740
#define SHOT_XIAOBI 370
#define SHOT_SHOUWAN 680

// 运动控制参数
#define SMOOTH_DELAY_MS 20
#define STEP_DELAY_MS 1000
typedef struct
{
    int di;
    int dabi;
    int zhongbi;
    int xiaobi;
    int shouwan;
    int gripper;
} ServoPositions;

// 当前位置全局变量
static ServoPositions current_pos = {
    .di = 0,
    .dabi = 0,
    .zhongbi = 0,
    .xiaobi = 0,
    .shouwan = 0,
    .gripper = 0};

static const ServoPositions RESET_POS = {
    .di = 970,
    .dabi = 540,
    .zhongbi = 1000,
    .xiaobi = 370,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS = {
    .di = 970,
    .dabi = 570,
    .zhongbi = 350,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_1 = {
    .di = 970,
    .dabi = 630,
    .zhongbi = 430,
    .xiaobi = 1080,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_2 = {
    .di = 970,
    .dabi = 680,
    .zhongbi = 510,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_3 = {
    .di = 970,
    .dabi = 710,
    .zhongbi = 590,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_4 = {
    .di = 970,
    .dabi = 750,
    .zhongbi = 690,
    .xiaobi = 1040,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions GRASP_POS = {
    .di = 970,
    .dabi = 710,
    .zhongbi = 590,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_CLOSE};

static const ServoPositions SHOT_POS = {
    .di = 510,
    .dabi = 540,
    .zhongbi = 740,
    .xiaobi = 370,
    .shouwan = 680,
    .gripper = GRIPPER_CLOSE};

static void arm_set_pwm(ServoPositions pos)
{
    pwm_set_duty(DI_PWM, pos.di);
    pwm_set_duty(DABI_PWM, pos.dabi);
    pwm_set_duty(ZHONGBI_PWM, pos.zhongbi);
    pwm_set_duty(XIAOBI_PWM, pos.xiaobi);
    pwm_set_duty(SHOUWAN_PWM, pos.shouwan);
    pwm_set_duty(GRIPPER_PWM, pos.gripper);
}

static void move_pose_smoothly(ServoPositions target_pos, int16 time_ms)
{
    // 计算每个舵机的移动距离
    int diff_di = target_pos.di - current_pos.di;
    int diff_dabi = target_pos.dabi - current_pos.dabi;
    int diff_zhongbi = target_pos.zhongbi - current_pos.zhongbi;
    int diff_xiaobi = target_pos.xiaobi - current_pos.xiaobi;
    int diff_shouwan = target_pos.shouwan - current_pos.shouwan;
    int diff_gripper = target_pos.gripper - current_pos.gripper;

    // 计算步数（基于时间和延迟）
    int total_steps = time_ms / SMOOTH_DELAY_MS;
    if (total_steps <= 0)
        total_steps = 1;

    // 计算每步的移动量
    float step_di = (float)diff_di / total_steps;
    float step_dabi = (float)diff_dabi / total_steps;
    float step_zhongbi = (float)diff_zhongbi / total_steps;
    float step_xiaobi = (float)diff_xiaobi / total_steps;
    float step_shouwan = (float)diff_shouwan / total_steps;
    float step_gripper = (float)diff_gripper / total_steps;

    // 记录起始位置
    ServoPositions start_pos = current_pos;

    // 逐步移动到目标位置
    for (int step = 1; step <= total_steps; step++)
    {
        // 计算当前步的目标位置
        int target_di = start_pos.di + (int)(step_di * step);
        int target_dabi = start_pos.dabi + (int)(step_dabi * step);
        int target_zhongbi = start_pos.zhongbi + (int)(step_zhongbi * step);
        int target_xiaobi = start_pos.xiaobi + (int)(step_xiaobi * step);
        int target_shouwan = start_pos.shouwan + (int)(step_shouwan * step);
        int target_gripper = start_pos.gripper + (int)(step_gripper * step);

        // 在最后一步，确保到达精确的目标位置
        if (step == total_steps)
        {
            target_di = target_pos.di;
            target_dabi = target_pos.dabi;
            target_zhongbi = target_pos.zhongbi;
            target_xiaobi = target_pos.xiaobi;
            target_shouwan = target_pos.shouwan;
            target_gripper = target_pos.gripper;
        }

        // 更新当前位置并设置PWM
        current_pos.di = target_di;
        current_pos.dabi = target_dabi;
        current_pos.zhongbi = target_zhongbi;
        current_pos.xiaobi = target_xiaobi;
        current_pos.shouwan = target_shouwan;
        current_pos.gripper = target_gripper;

        arm_set_pwm(current_pos);

        // 延迟
        system_delay_ms(SMOOTH_DELAY_MS);
    }
}

static void delay_step(void)
{
    system_delay_ms(STEP_DELAY_MS);
}

void roboarm_init(void)
{
    pwm_init(DI_PWM, 50, 0);
    pwm_init(DABI_PWM, 50, 0);
    pwm_init(ZHONGBI_PWM, 50, 0);
    pwm_init(XIAOBI_PWM, 50, 0);
    pwm_init(SHOUWAN_PWM, 50, 0);
    pwm_init(GRIPPER_PWM, 50, 0);
}

void arm_reset(void)
{
    if (current_pos.di == 0 ||
        current_pos.dabi == 0 ||
        current_pos.zhongbi == 0 ||
        current_pos.xiaobi == 0 ||
        current_pos.shouwan == 0 ||
        current_pos.gripper == 0)
    {
        current_pos = RESET_POS;
        arm_set_pwm(RESET_POS);
    }
    else
    {
        move_pose_smoothly(RESET_POS, 500);
    }
}

void arm_relax(void)
{
    current_pos.di = 0;
    current_pos.dabi = 0;
    current_pos.zhongbi = 0;
    current_pos.xiaobi = 0;
    current_pos.shouwan = 0;
    current_pos.gripper = 0;

    arm_set_pwm(current_pos);
}

void arm_reset_to_prepare(void)
{
    printf("=== Starting reset to prepare ===\r\n");

    // 1) 底座旋转到准备位置
    ServoPositions p = current_pos;
    p.di = PREPARE_POS.di;
    printf("Step 1: Moving base to prepare position\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    // 2) 其他臂同步平滑旋转到准备位置
    p = PREPARE_POS;
    printf("Step 2: Moving all arms to prepare position\r\n");
    move_pose_smoothly(p, 1000);
    delay_step();

    printf("=== Reset to prepare complete ===\r\n");
}

void arm_prepare_to_grip(void)
{
    printf("=== Starting prepare to grip ===\r\n");

    // 1) 同步将大臂/中臂移动到抓取位（其他关节保持不变）
    ServoPositions p = current_pos;
    p = PREPARE_POS_4;
    printf("Step 1: Moving arm to grip position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 2) 夹爪闭合形成抓取
    printf("Step 2: Closing gripper\r\n");
    p = current_pos;
    p.gripper = GRIPPER_CLOSE;
    move_pose_smoothly(p, 300);
    delay_step();

    printf("=== Prepare to grip complete ===\r\n");
}

void arm_grip_to_shot(void)
{
    printf("=== Starting grip to shot ===\r\n");

    ServoPositions p;

    // 1) 大臂到投掷位，准备投掷
    p = current_pos;
    p.dabi = SHOT_POS.dabi;
    p.zhongbi = PROCESS2_ZHONGBI;
    printf("Step 1: Moving to shot dabi position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 2) 小臂到投掷位
    p = current_pos;
    p.xiaobi = SHOT_POS.xiaobi;
    printf("Step 3: Moving to shot xiaobi position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 3) 中臂到过程位，底座到投掷位
    p = current_pos;
    p.zhongbi = PROCESS_ZHONGBI;
    p.di = SHOT_POS.di;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 4) 中臂到投掷位，准备放飞镖
    p = current_pos;
    p.zhongbi = SHOT_POS.zhongbi;
    printf("Step 6: Moving to shot zhongbi position for dart placement\r\n");
    move_pose_smoothly(p, 1000);
    delay_step();

    // 5) 打开夹爪，放飞镖
    p = current_pos;
    p.gripper = GRIPPER_OPEN;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 7: Opening gripper\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    // // 6) 回到等待位置（可选）
    // p = current_pos;
    // p.zhongbi = PROCESS_ZHONGBI;
    // printf("Step 8: Moving to wait position\r\n");
    // move_pose_smoothly(p, 400);
    // delay_step();

    printf("=== Grip to shot complete ===\r\n");
}

void arm_shot_to_reset(void)
{
    printf("=== Starting shot to reset ===\r\n");

    ServoPositions p;

    // 1) 回到复位位置，准备下一次抓取
    p = current_pos;
    p = RESET_POS;
    printf("Step 1: Moving to grip di position\r\n");
    move_pose_smoothly(p, 400);
    delay_step();

    printf("=== Shot to grip prepare complete ===\r\n");
}
