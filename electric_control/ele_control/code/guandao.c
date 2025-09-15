#include "guandao.h"

#pragma section all "cpu0_dsram"

/* ===================== 全局变量定义 ===================== */
uint8 raw_packet[GYRO_PACKET_SIZE];
uint8 packet_index = 0;
uint8 get_data = 0;

GyroData gyro_data;
INS_System ins;

fifo_struct uart_data_fifo;

/* ===================== 常量定义 ===================== */
// 新顺序：FRONT_LEFT_PIN1, FRONT_LEFT_PIN2, FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2, BACK_LEFT_PIN1, BACK_LEFT_PIN2, BACK_RIGHT_PIN1, BACK_RIGHT_PIN2
// 前进：所有轮前进
const MotorPattern PATTERN_FRONT = {{1, 0, 1, 0, 1, 0, 1, 0}};
// 后退：所有轮后退
const MotorPattern PATTERN_BACK = {{0, 1, 0, 1, 0, 1, 0, 1}};
// 逆时针旋转：左轮后退，右轮前进
const MotorPattern PATTERN_CCW = {{0, 1, 1, 0, 0, 1, 1, 0}};
// 顺时针旋转：左轮前进，右轮后退
const MotorPattern PATTERN_CW = {{1, 0, 0, 1, 1, 0, 0, 1}};
const MotorPattern PATTERN_LEFT = {{0, 1, 1, 0, 1, 0, 0, 1}};
const MotorPattern PATTERN_RIGHT = {{1, 0, 0, 1, 0, 1, 1, 0}};
// 停止
const MotorPattern PATTERN_STOP = {{0, 0, 0, 0, 0, 0, 0, 0}};

/* ===================== 初始化函数 ===================== */
void guandao_init(void)
{
    // 初始化导航系统
    INS_Init(&ins);
    gyro_data_init();
    motion_init();

    // 初始化串口
    uart_init(WRITE_UART, UART_BAUDRATE, TC264_TX_PIN, TC264_RX_PIN);
    uart_init(READ_UART, UART_BAUDRATE, IMU_TX_PIN, IMU_RX_PIN);

    // 要先初始化UART才能初始化IMU
    //imu_init();
    //uart_write_string(WRITE_UART, "Guandao System Initialized.\r\n");
}

void imu_init(void)
{
    uint8 tx_data[] = {0xFF, 0xAA, 0x67};
    uart_write_buffer(UART_3, tx_data, sizeof(tx_data));
}

void motion_init(void)
{
    // 初始化GPIO引脚
    const uint32 gpio_pins[8] = {
        FRONT_LEFT_PIN1, FRONT_LEFT_PIN2,
        FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2,
        BACK_LEFT_PIN1, BACK_LEFT_PIN2,
        BACK_RIGHT_PIN1, BACK_RIGHT_PIN2};
    for (int i = 0; i < 8; i++)
    {
        gpio_init(gpio_pins[i], GPO, GPIO_LOW, GPO_PUSH_PULL);
    }

    // 初始化PWM
    pwm_init(BACK_LEFT_PWM, 5000, 0);
    pwm_init(BACK_RIGHT_PWM, 5000, 0);
    pwm_init(FRONT_RIGHT_PWM, 5000, 0);
    pwm_init(FRONT_LEFT_PWM, 5000, 0);
}

void INS_Init(INS_System *ins)
{
    for (int i = 0; i < 3; i++)
    {
        ins->position[i] = 0.0f;
        ins->velocity[i] = 0.0f;
    }

    ins->last_update_time = system_getval_ms();
    ins->dt = 0.0f;
}

void gyro_data_init(void)
{
    gyro_data.accel_x = 0.0f;
    gyro_data.accel_y = 0.0f;
    gyro_data.accel_z = 0.0f;
    gyro_data.gyro_x = 0.0f;
    gyro_data.gyro_y = 0.0f;
    gyro_data.gyro_z = 0.0f;
    gyro_data.roll = 0.0f;
    gyro_data.pitch = 0.0f;
    gyro_data.yaw = 0.0f;
}

/* ===================== 数据处理函数 ===================== */
void unpack_and_analyze_imu_data(void)
{
    // 只在packet_index==0时判断0x55为包头
    uint8 byte = uart_read_byte(READ_UART);
    // printf("%d", byte);
    if (packet_index == 0)
    {
        if (byte != 0x55)
        {
            // 不是包头，丢弃
            return;
        }
    }
    raw_packet[packet_index++] = byte;
    if (packet_index == GYRO_PACKET_SIZE)
    {
        uint8_t packet_type = raw_packet[1];
        bool valid_packet = (packet_type == 0x51 || packet_type == 0x52 || packet_type == 0x53) &&
                            checksum(raw_packet, GYRO_PACKET_SIZE - 1, raw_packet[GYRO_PACKET_SIZE - 1]);
        if (valid_packet)
        {
            parse_gyro_data(raw_packet, packet_type);
            if (packet_type == 0x53)
            {
                print_gyro_data();
                INS_UpdatePosition(&ins, &gyro_data);
                INS_PrintData(&ins);
            }
        }
        packet_index = 0;
    }
}

void parse_gyro_data(uint8 *data, uint8 type)
{
    switch (type)
    {
    case 0x51:
    {
        int16 ax = (data[3] << 8) | data[2];
        int16 ay = (data[5] << 8) | data[4];
        int16 az = (data[7] << 8) | data[6];

        gyro_data.accel_x = (float)ax / 32768.0f * 16.0f;
        gyro_data.accel_y = (float)ay / 32768.0f * 16.0f;
        gyro_data.accel_z = (float)az / 32768.0f * 16.0f;
        break;
    }
    case 0x52:
    {
        int16 wx = (data[3] << 8) | data[2];
        int16 wy = (data[5] << 8) | data[4];
        int16 wz = (data[7] << 8) | data[6];

        gyro_data.gyro_x = (float)wx / 32768.0f * 2000.0f;
        gyro_data.gyro_y = (float)wy / 32768.0f * 2000.0f;
        gyro_data.gyro_z = (float)wz / 32768.0f * 2000.0f;
        break;
    }
    case 0x53:
    {
        int16 roll = (data[3] << 8) | data[2];
        int16 pitch = (data[5] << 8) | data[4];
        int16 yaw = (data[7] << 8) | data[6];

        gyro_data.roll = (float)roll / 32768.0f * 180.0f;
        gyro_data.pitch = (float)pitch / 32768.0f * 180.0f;
        gyro_data.yaw = (float)yaw / 32768.0f * 180.0f;
        break;
    }
    }
}

uint8 checksum(uint8 *data, uint8 len, uint8 sum)
{
    uint8 calc_sum = 0;
    for (uint8 i = 0; i < len; i++)
        calc_sum += data[i];
    return (calc_sum == sum);
}

/* ===================== 运动控制函数 ===================== */
void set_motion(const MotorPattern *pattern)
{
    const uint32 pins[8] = {
        FRONT_LEFT_PIN1, FRONT_LEFT_PIN2,
        FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2,
        BACK_LEFT_PIN1, BACK_LEFT_PIN2,
        BACK_RIGHT_PIN1, BACK_RIGHT_PIN2};
    for (int i = 0; i < 8; i++)
    {
        gpio_set_level(pins[i], pattern->in[i] ? GPIO_HIGH : GPIO_LOW);
    }
    // 判断是否为前进/后退模式
    int is_front = 1, is_back = 1;
    for (int i = 0; i < 8; i++)
    {
        if (pattern->in[i] != PATTERN_FRONT.in[i])
            is_front = 0;
        if (pattern->in[i] != PATTERN_BACK.in[i])
            is_back = 0;
    }
    if (is_front || is_back)
    {
        pwm_set_duty(FRONT_LEFT_PWM, 4000);
        pwm_set_duty(FRONT_RIGHT_PWM, 4000);
        pwm_set_duty(BACK_LEFT_PWM, 4000);
        pwm_set_duty(BACK_RIGHT_PWM, 4000);
    }
    else
    {
        pwm_set_duty(FRONT_LEFT_PWM, 5250);
        pwm_set_duty(FRONT_RIGHT_PWM, 5250);
        pwm_set_duty(BACK_LEFT_PWM, 3400);
        pwm_set_duty(BACK_RIGHT_PWM, 3400);
    }
}

/* ===================== 导航算法函数 ===================== */
void INS_UpdatePosition(INS_System *ins, GyroData *data)
{
    // 更新时间间隔dt
    uint32_t now = system_getval_ms();
    ins->dt = (now - ins->last_update_time) / 1000.0f;
    ins->last_update_time = now;

    // 1. 读取本体坐标系下的加速度（单位g，需转m/s^2），只保留小数点后三位
    float ax = (float)((int)(data->accel_x * 1000)) / 1000.0f * 9.8f;
    float ay = (float)((int)(data->accel_y * 1000)) / 1000.0f * 9.8f;
    // 2. 获取当前偏航角yaw（单位：度，需转弧度）
    float yaw_rad = data->yaw * M_PI / 180.0f;

    // 3. 坐标变换：本体加速度转到世界坐标系（地面xy）
    float a_world[3];
    float c = cosf(yaw_rad);
    float s = sinf(yaw_rad);

    a_world[0] = ax * c - ay * s;
    a_world[1] = ax * s + ay * c;
    a_world[2] = 0;

    // 4. 积分更新速度和位置（仅xy）
    for (int i = 0; i < 2; i++)
    {
        ins->velocity[i] += a_world[i] * ins->dt;
        ins->position[i] += ins->velocity[i] * ins->dt;
    }

    // z方向不变
    ins->velocity[2] = 0;
    ins->position[2] = 0;

    // 调试输出：打印dt、a_world、velocity、position
    char debug_buffer[256];
    sprintf(debug_buffer, "[DEBUG] dt=%.6f\r\n", ins->dt);
    uart_write_string(WRITE_UART, debug_buffer);
    sprintf(debug_buffer, "[DEBUG] a_world: X=%.6f Y=%.6f Z=%.6f\r\n", a_world[0], a_world[1], a_world[2]);
    uart_write_string(WRITE_UART, debug_buffer);
    sprintf(debug_buffer, "[DEBUG] yaw_rad = %.6f\r\n", yaw_rad);
    uart_write_string(WRITE_UART, debug_buffer);
    sprintf(debug_buffer, "[DEBUG] cos = %.6f sin = %.6f\r\n", c, s);
    uart_write_string(WRITE_UART, debug_buffer);
}

/* ===================== 调试输出函数 ===================== */
void print_gyro_data(void)
{
    char buffer[128];
    sprintf(buffer, "Accel: X=%.4f Y=%.4f Z=%.4f g\r\n", gyro_data.accel_x, gyro_data.accel_y, gyro_data.accel_z);
    uart_write_string(WRITE_UART, buffer);

    sprintf(buffer, "Gyro: X=%.2f Y=%.2f Z=%.2f °/s\r\n", gyro_data.gyro_x, gyro_data.gyro_y, gyro_data.gyro_z);
    uart_write_string(WRITE_UART, buffer);

    sprintf(buffer, "Angle: Roll=%.2f Pitch=%.2f Yaw=%.2f °\r\n\r\n", gyro_data.roll, gyro_data.pitch, gyro_data.yaw);
    uart_write_string(WRITE_UART, buffer);
}

void INS_PrintData(INS_System *ins)
{
    char buffer[128];

    sprintf(buffer, "Position: X=%.2f Y=%.2f Z=%.2f m\r\n",
            ins->position[0], ins->position[1], ins->position[2]);
    uart_write_string(WRITE_UART, buffer);

    sprintf(buffer, "Velocity: Vx=%.2f Vy=%.2f Vz=%.2f m/s\r\n\r\n",
            ins->velocity[0], ins->velocity[1], ins->velocity[2]);
    uart_write_string(WRITE_UART, buffer);
}

/* ===================== 中断处理函数 ===================== */
void uart_rx_interrupt_handler(void)
{
    get_data = uart_read_byte(READ_UART);
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);
}

#pragma section all restore
