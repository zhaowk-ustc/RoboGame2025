#include "push.h"

/* ===================== 全局变量定义 ===================== */
int16 encoder_count = 0;
MovementMode current_mode = MODE_STOP;

/* ===================== 私有函数声明 ===================== */
static void initialize_hardware(void);
static void handle_forward_movement(void);
static void handle_return_movement(void);
static void handle_stop(void);
static void print_status_info(void);

/* ===================== 公有函数实现 ===================== */

/**
 * @brief 初始化推送模块
 */
void push_init(void)
{
    initialize_hardware(); // 初始化所有硬件
    encoder_clear_count(ENCODER_MODULE);
    printf("Push module initialized.\r\n");
}

/**
 * @brief 更新推送模块状态（主循环中调用）
 */
void push_update(void)
{
    encoder_count = encoder_get_count(ENCODER_MODULE); // 更新编码器数据

    // 根据当前模式处理运动

    switch (current_mode)
    {
    case MODE_STOP:
        handle_stop();
        break;
    case MODE_FORWARD:
        handle_forward_movement();
        break;
    case MODE_RETURN:
        handle_return_movement();
        break;
    }

    print_status_info(); // 输出状态信息
    system_delay_ms(LOOP_DELAY_MS);
}

void push_stop(void)
{
    current_mode = MODE_STOP;
    pwm_set_duty(SPEED_PWM, 0);
    printf("Push stopped.\r\n");
}

/* ===================== 私有函数实现 ===================== */

/**
 * @brief 初始化所有硬件外设
 */
static void initialize_hardware(void)
{
    // 初始化编码器
    encoder_dir_init(ENCODER_MODULE, ENCODER_A_PIN, ENCODER_B_PIN);
    encoder_clear_count(ENCODER_MODULE);

    // 初始化PWM
    pwm_init(SPEED_PWM, 50, 0);

    // 初始化方向控制GPIO
    gpio_init(DIRECTION_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/**
 * @brief 处理向前运动模式
 */

static void handle_forward_movement(void)
{
    // 限定编码器范围
    if (encoder_count <= -TARGET_POSITION)
    {
        push_stop();
        printf("Reached negative limit, stopped.\r\n");
        return;
    }

    // 设置向前运动方向
    gpio_set_level(DIRECTION_PIN, DIRECTION_FORWARD);
    pwm_set_duty(SPEED_PWM, MOVE_SPEED);
}

/**
 * @brief 处理返回运动模式
 */

static void handle_return_movement(void)
{
    // 限定编码器范围
    if (encoder_count >= 0)
    {
        push_stop();
        printf("Reached positive limit, stopped.\r\n");
        return;
    }
    // 设置返回运动方向
    gpio_set_level(DIRECTION_PIN, DIRECTION_BACKWARD);
    pwm_set_duty(SPEED_PWM, MOVE_SPEED);
}

/**
 * @brief 打印状态信息
 */

static void handle_stop(void)
{
    pwm_set_duty(SPEED_PWM, 0);
    // 可选：可添加其他停止时的处理逻辑
}

static void print_status_info(void)
{
    const char *mode_str = "Unknown";
    switch (current_mode)
    {
    case MODE_STOP:
        mode_str = "Stop";
        break;
    case MODE_FORWARD:
        mode_str = "Forward";
        break;
    case MODE_RETURN:
        mode_str = "Return";
        break;
    }
    printf("Encoder: %6d, Mode: %s\r\n", encoder_count, mode_str);
}
