#include "push.h"

/* ===================== 全局变量定义 ===================== */
int16 encoder_count = 0;
MovementMode current_mode = MODE_FORWARD;

/* ===================== 私有函数声明 ===================== */
static void initialize_hardware(void);
static void update_encoder_data(void);
static void handle_forward_movement(void);
static void handle_return_movement(void);
static void print_status_info(void);

/* ===================== 公有函数实现 ===================== */

/**
 * @brief 初始化推送模块
 */
void push_init(void)
{
    initialize_hardware(); // 初始化所有硬件
    printf("Push module initialized.\r\n");
}

/**
 * @brief 更新推送模块状态（主循环中调用）
 */
void push_update(void)
{
    update_encoder_data(); // 更新编码器数据

    // 根据当前模式处理运动
    switch (current_mode)
    {
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

/**
 * @brief 设置运动模式
 * @param new_mode 新的运动模式
 */
void push_set_mode(MovementMode new_mode)
{
    current_mode = new_mode;
    encoder_clear_count(ENCODER_MODULE);
    printf("Mode changed to: %s\r\n",
           (new_mode == MODE_FORWARD) ? "Forward" : "Return");
}

/**
 * @brief 获取当前运动模式
 * @return 当前运动模式
 */
MovementMode push_get_mode(void)
{
    return current_mode;
}

/**
 * @brief 获取当前编码器计数
 * @return 编码器计数值
 */
int16 push_get_encoder_count(void)
{
    return encoder_count;
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
 * @brief 更新编码器数据
 */
static void update_encoder_data(void)
{
    encoder_count = encoder_get_count(ENCODER_MODULE);
}

/**
 * @brief 处理向前运动模式
 */
static void handle_forward_movement(void)
{
    // 设置向前运动方向
    gpio_set_level(DIRECTION_PIN, DIRECTION_FORWARD);
    pwm_set_duty(SPEED_PWM, MOVE_SPEED);

    // 检查是否到达目标位置
    if (encoder_count <= (-TARGET_POSITION))
    {
        push_set_mode(MODE_RETURN);
        printf("Reached target position, starting return.\r\n");
        system_delay_ms(ENDPOINT_DELAY_MS);
    }
}

/**
 * @brief 处理返回运动模式
 */
static void handle_return_movement(void)
{
    // 设置返回运动方向
    gpio_set_level(DIRECTION_PIN, DIRECTION_BACKWARD);
    pwm_set_duty(SPEED_PWM, MOVE_SPEED);

    // 检查是否退回起点
    if (encoder_count >= TARGET_POSITION)
    {
        push_set_mode(MODE_FORWARD);
        printf("Returned to start position.\r\n");
        system_delay_ms(ENDPOINT_DELAY_MS);
    }
}

/**
 * @brief 打印状态信息
 */
static void print_status_info(void)
{
    const char *mode_str = (current_mode == MODE_FORWARD) ? "Forward" : "Return";
    printf("Encoder: %6d, Mode: %s\r\n", encoder_count, mode_str);
}
