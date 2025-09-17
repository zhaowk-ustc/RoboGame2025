#include "roboarm_motion.h"
#include "zf_common_headfile.h"

ServoPositions current_pos = {
    .di = GRIP_DI,
    .dabi = GRIP_PREPARE_DABI,
    .zhongbi = GRIP_PREPARE_ZHONGBI,
    .xiaobi = GRIP_XIAOBI,
    .shouwan = GRIP_SHOUWAN,
    .gripper = GRIPPER_OPEN};

void roboarm_init(void)
{
    pwm_init(DI_PWM, 50, 0);
    pwm_init(DABI_PWM, 50, 0);
    pwm_init(ZHONGBI_PWM, 50, 0);
    pwm_init(XIAOBI_PWM, 50, 0);
    pwm_init(SHOUWAN_PWM, 50, 0);
    pwm_init(GRIPPER_PWM, 50, 0);
}

void set_all_pwm(int duty)
{
    pwm_set_duty(DI_PWM, duty);
    pwm_set_duty(DABI_PWM, duty);
    pwm_set_duty(ZHONGBI_PWM, duty);
    pwm_set_duty(XIAOBI_PWM, duty);
    pwm_set_duty(SHOUWAN_PWM, duty);
    pwm_set_duty(GRIPPER_PWM, duty);
}

void test_servo_positions(void)
{
    int duty = 250;
    set_all_pwm(duty);
    system_delay_ms(2000);
    while (duty <= 1250)
    {
        set_all_pwm(duty);
        duty++;
        printf("duty: %d\r\n", duty);
        system_delay_ms(20);
    }
    system_delay_ms(2000);
}

void move_servo_smoothly(int *current_pos, int target_pos, int pwm_pin)
{
    while (*current_pos != target_pos)
    {
        if (*current_pos < target_pos)
        {
            *current_pos += MOVE_SPEED;
            if (*current_pos > target_pos)
                *current_pos = target_pos;
        }
        else
        {
            *current_pos -= MOVE_SPEED;
            if (*current_pos < target_pos)
                *current_pos = target_pos;
        }
        pwm_set_duty(pwm_pin, *current_pos);
        system_delay_ms(DELAY_MS);
    }
}

void delay_step(void)
{
    system_delay_ms(STEP_DELAY_MS);
}

void move_to_grip_prepare(void)
{
    printf("Moving to grip_prepare position...\r\n");
    move_servo_smoothly(&current_pos.dabi, VERTICAL_DABI, DABI_PWM);
    move_servo_smoothly(&current_pos.zhongbi, PROCESS2_ZHONGBI, ZHONGBI_PWM);

    move_servo_smoothly(&current_pos.di, GRIP_DI, DI_PWM);
    move_servo_smoothly(&current_pos.dabi, GRIP_PREPARE_DABI, DABI_PWM);
    move_servo_smoothly(&current_pos.zhongbi, GRIP_PREPARE_ZHONGBI, ZHONGBI_PWM);
    move_servo_smoothly(&current_pos.xiaobi, GRIP_XIAOBI, XIAOBI_PWM);
    move_servo_smoothly(&current_pos.shouwan, GRIP_SHOUWAN, SHOUWAN_PWM);
    move_servo_smoothly(&current_pos.gripper, GRIPPER_OPEN, GRIPPER_PWM);
}

void arm_init(void)
{
    pwm_set_duty(DI_PWM, 510);
    current_pos.di = 510;
    pwm_set_duty(DABI_PWM, 540);
    current_pos.dabi = 540;
    pwm_set_duty(ZHONGBI_PWM, 1000);
    current_pos.zhongbi = 1000;
    pwm_set_duty(XIAOBI_PWM, 370);
    current_pos.xiaobi = 370;
    pwm_set_duty(SHOUWAN_PWM, 680);
    current_pos.shouwan = 680;
    pwm_set_duty(GRIPPER_PWM, 250);
    current_pos.gripper = 250;
}

void arm_relax(void)
{
    pwm_set_duty(DI_PWM, 0);
    pwm_set_duty(DABI_PWM, 0);
    pwm_set_duty(ZHONGBI_PWM, 0);
    pwm_set_duty(XIAOBI_PWM, 0);
    pwm_set_duty(SHOUWAN_PWM, 0);
    pwm_set_duty(GRIPPER_PWM, 0);
    
    // 更新当前位置状态
    current_pos.di = 0;
    current_pos.dabi = 0;
    current_pos.zhongbi = 0;
    current_pos.xiaobi = 0;
    current_pos.shouwan = 0;
    current_pos.gripper = 0;
}

void grip_prepare_to_grip(void)
{
    printf("=== Start grip ===\r\n");
    // 大臂和小臂同时移动到GRIP_DABI和GRIP_ZHONGBI
    int max_steps = 0;
    int start_dabi = current_pos.dabi;
    int start_zhongbi = current_pos.zhongbi;
    int diff_dabi = GRIP_DABI - start_dabi;
    int diff_zhongbi = GRIP_ZHONGBI - start_zhongbi;
    int steps_dabi = (diff_dabi > 0 ? diff_dabi : -diff_dabi) / MOVE_SPEED;
    int steps_zhongbi = (diff_zhongbi > 0 ? diff_zhongbi : -diff_zhongbi) / MOVE_SPEED;
    max_steps = (steps_dabi > steps_zhongbi) ? steps_dabi : steps_zhongbi;
    if (max_steps == 0)
        max_steps = 1;
    for (int i = 0; i < max_steps; i++)
    {
        if (current_pos.dabi != GRIP_DABI)
        {
            if (current_pos.dabi < GRIP_DABI)
            {
                current_pos.dabi += MOVE_SPEED;
                if (current_pos.dabi > GRIP_DABI)
                    current_pos.dabi = GRIP_DABI;
            }
            else
            {
                current_pos.dabi -= MOVE_SPEED;
                if (current_pos.dabi < GRIP_DABI)
                    current_pos.dabi = GRIP_DABI;
            }
            pwm_set_duty(DABI_PWM, current_pos.dabi);
        }
        if (current_pos.zhongbi != GRIP_ZHONGBI)
        {
            if (current_pos.zhongbi < GRIP_ZHONGBI)
            {
                current_pos.zhongbi += MOVE_SPEED;
                if (current_pos.zhongbi > GRIP_ZHONGBI)
                    current_pos.zhongbi = GRIP_ZHONGBI;
            }
            else
            {
                current_pos.zhongbi -= MOVE_SPEED;
                if (current_pos.zhongbi < GRIP_ZHONGBI)
                    current_pos.zhongbi = GRIP_ZHONGBI;
            }
            pwm_set_duty(ZHONGBI_PWM, current_pos.zhongbi);
        }
        system_delay_ms(DELAY_MS);
    }
    delay_step();
    // 夹爪动作
    printf("Step: Close gripper\r\n");
    move_servo_smoothly(&current_pos.gripper, GRIPPER_CLOSE, GRIPPER_PWM);
    delay_step();
}

void grip_to_shot(void)
{
    printf("=== Starting grip_to_shot ===\r\n");
    struct
    {
        int *target_ptr;
        int target_val;
        int pwm_pin;
        const char *description;
    } steps[] = {
        {&current_pos.dabi, SHOT_DABI, DABI_PWM, "Moving to SHOT_DABI"},
        {&current_pos.zhongbi, PROCESS2_ZHONGBI, ZHONGBI_PWM, "Moving to PROCESS2_ZHONGBI"},
        {&current_pos.xiaobi, SHOT_XIAOBI, XIAOBI_PWM, "Moving to SHOT_XIAOBI"},
        {&current_pos.zhongbi, PROCESS_ZHONGBI, ZHONGBI_PWM, "Moving to PROCESS_ZHONGBI"},
        {&current_pos.di, SHOT_DI, DI_PWM, "Moving to SHOT_DI"},
        {&current_pos.zhongbi, SHOT_ZHONGBI, ZHONGBI_PWM, "Placing the dart, Moving to SHOT_ZHONGBI"},
        {&current_pos.gripper, GRIPPER_OPEN, GRIPPER_PWM, "Opening gripper"},
        {&current_pos.zhongbi, PROCESS_ZHONGBI, ZHONGBI_PWM, "Wait for shot, Moving to PROCESS_ZHONGBI"}};
    const int step_count = sizeof(steps) / sizeof(steps[0]);
    for (int i = 0; i < step_count; i++)
    {
        printf("Step %d: %s\r\n", i + 1, steps[i].description);
        move_servo_smoothly(steps[i].target_ptr, steps[i].target_val, steps[i].pwm_pin);
        delay_step();
    }
    delay_step();
    printf("=== grip_to_shot Complete ===\r\n");
}

void shot_to_grip_prepare(void)
{
    printf("=== Starting shot_to_grip_prepare ===\r\n");
    struct
    {
        int *target_ptr;
        int target_val;
        int pwm_pin;
        const char *description;
    } steps[] = {
        {&current_pos.di, GRIP_DI, DI_PWM, "Moving to GRIP_DI"},
        {&current_pos.zhongbi, PROCESS2_ZHONGBI, ZHONGBI_PWM, "Moving to PROCESS2_ZHONGBI"},
        {&current_pos.xiaobi, GRIP_XIAOBI, XIAOBI_PWM, "Moving to GRIP_XIAOBI"},
        {&current_pos.zhongbi, GRIP_PREPARE_ZHONGBI, ZHONGBI_PWM, "Moving to GRIP_PREPARE_ZHONGBI"},
        {&current_pos.dabi, GRIP_PREPARE_DABI, DABI_PWM, "Moving to GRIP_PREPARE_DABI"},
        {&current_pos.shouwan, GRIP_SHOUWAN, SHOUWAN_PWM, "Maintaining wrist position"}};
    const int step_count = sizeof(steps) / sizeof(steps[0]);
    for (int i = 0; i < step_count; i++)
    {
        printf("Step %d: %s\r\n", i + 1, steps[i].description);
        move_servo_smoothly(steps[i].target_ptr, steps[i].target_val, steps[i].pwm_pin);
        delay_step();
    }
    delay_step();
    printf("=== shot_to_grip_prepare complete ===\r\n");
}
