#include "guandao.h"
#include <string.h> // for memcmp
#pragma section all "cpu0_dsram"

/* ===================== 全局变量定义 ===================== */
uint8 raw_packet[GYRO_PACKET_SIZE];
uint8 packet_index = 0;

GyroData gyro_data;
INS_System ins;

// 软 FIFO：中断写入，业务读取
fifo_struct uart_data_fifo;
static uint8 uart_fifo_buf[GYRO_DATA_BUFFER_SIZE]; // 缓冲区

/* ===================== 常量定义 ===================== */
// 新顺序：FRONT_LEFT_PIN1, FRONT_LEFT_PIN2, FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2, BACK_LEFT_PIN1, BACK_LEFT_PIN2, BACK_RIGHT_PIN1, BACK_RIGHT_PIN2
// 前进：所有轮前进
const MotorPattern PATTERN_FRONT = {{1, 0, 1, 0, 1, 0, 1, 0}};
// 后退：所有轮后退
const MotorPattern PATTERN_BACK = {{0, 1, 0, 1, 0, 1, 0, 1}};
const MotorPattern PATTERN_CCW = {{0, 1, 1, 0, 0, 1, 1, 0}}; // 逆时针
const MotorPattern PATTERN_CW = {{1, 0, 0, 1, 1, 0, 0, 1}};  // 顺时针
const MotorPattern PATTERN_LEFT = {{0, 1, 1, 0, 1, 0, 0, 1}};
const MotorPattern PATTERN_RIGHT = {{1, 0, 0, 1, 0, 1, 1, 0}};
// 停止
const MotorPattern PATTERN_STOP = {{0, 0, 0, 0, 0, 0, 0, 0}};

/* ===================== 内部小工具 ===================== */
// 有些平台仅提供 fifo_read_buffer，这里包装成读 1 字节的帮助函数
static inline bool fifo_read_byte(fifo_struct *f, uint8 *out)
{
    if (out == NULL)
        return false;
    uint32 length = 1;
    fifo_state_enum result = fifo_read_buffer(f, out, &length, FIFO_READ_AND_CLEAN);
    return (result == FIFO_SUCCESS && length == 1);
}

/* ===================== 初始化函数 ===================== */
void guandao_init(void)
{
    // 初始化导航系统
    INS_Init(&ins);
    gyro_data_init();
    motion_init();

    // 初始化串口（用于 IMU）
    uart_init(READ_UART, UART_BAUDRATE, IMU_TX_PIN, IMU_RX_PIN);

    // 初始化软件 FIFO（务必在使能中断前）
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_fifo_buf, GYRO_DATA_BUFFER_SIZE);

    // IMU 初始化（需要 UART 已就绪）
    imu_reset();

    // 如需打印：uart_write_string(READ_UART, "Guandao System Initialized.\r\n");
}

void imu_reset(void)
{
    // 与 guandao_init 中一致：走 READ_UART
    uint8 tx_data[] = {0xFF, 0xAA, 0x67};
    uart_write_buffer(READ_UART, tx_data, sizeof(tx_data)); // 复位加速度
    uint8 tx_data2[] = {0xFF, 0xAA, 0x52};
    uart_write_buffer(READ_UART, tx_data2, sizeof(tx_data2)); // 复位角度
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

void INS_Init(INS_System *ins_sys)
{
    for (int i = 0; i < 3; i++)
    {
        ins_sys->position[i] = 0.0f;
        ins_sys->velocity[i] = 0.0f;
    }

    ins_sys->last_update_time = system_getval_ms();
    ins_sys->dt = 0.0f;
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
bool unpack_imu_data(void)
{
    uint8 byte;
    byte = uart_read_byte(READ_UART);

    if (packet_index == 0)
    {
        if (byte != 0x55)
        {
            // 不是包头，忽略
            return false;
        }
    }

    raw_packet[packet_index++] = byte;

    if (packet_index == GYRO_PACKET_SIZE)
    {
        uint8 packet_type = raw_packet[1];
        bool valid_packet =
            (packet_type == 0x51 || packet_type == 0x52 || packet_type == 0x53) &&
            checksum(raw_packet, GYRO_PACKET_SIZE - 1, raw_packet[GYRO_PACKET_SIZE - 1]);

        if (valid_packet)
        {
            parse_gyro_data(raw_packet, packet_type);
            packet_index = 0;

            if (packet_type == 0x53)
            {
                print_gyro_data();
                return true; // 成功处理0x53包
                // INS_UpdatePosition(&ins, &gyro_data);
                // INS_PrintData(&ins);
            }
            return false; // 解析到非角度包
        }
        else
        {
            // 校验失败，丢弃本包，从头同步
            packet_index = 0;
            return false; // 添加返回值
        }
    }

    return false; // 数据包未完成，继续接收
}

void parse_gyro_data(uint8 *data, uint8 type)
{
    switch (type)
    {
    case 0x51: // 加速度
    {
        int16 ax = (data[3] << 8) | data[2];
        int16 ay = (data[5] << 8) | data[4];
        int16 az = (data[7] << 8) | data[6];

        gyro_data.accel_x = (float)ax / 32768.0f * 16.0f;
        gyro_data.accel_y = (float)ay / 32768.0f * 16.0f;
        gyro_data.accel_z = (float)az / 32768.0f * 16.0f;
        break;
    }
    case 0x52: // 角速度
    {
        int16 wx = (data[3] << 8) | data[2];
        int16 wy = (data[5] << 8) | data[4];
        int16 wz = (data[7] << 8) | data[6];

        gyro_data.gyro_x = (float)wx / 32768.0f * 2000.0f;
        gyro_data.gyro_y = (float)wy / 32768.0f * 2000.0f;
        gyro_data.gyro_z = (float)wz / 32768.0f * 2000.0f;
        break;
    }
    case 0x53: // 欧拉角
    {
        int16 roll = (data[3] << 8) | data[2];
        int16 pitch = (data[5] << 8) | data[4];
        int16 yaw = (data[7] << 8) | data[6];

        gyro_data.roll = (float)roll / 32768.0f * 180.0f;
        gyro_data.pitch = (float)pitch / 32768.0f * 180.0f;
        gyro_data.yaw = (float)yaw / 32768.0f * 180.0f;
        break;
    }
    default:
        break;
    }
}

bool checksum(const uint8 *data, uint8 len, uint8 sum)
{
    uint8 calc_sum = 0;
    for (uint8 i = 0; i < len; i++)
    {
        calc_sum += data[i];
    }
    return (calc_sum == sum);
}

/* ===================== 运动控制函数 ===================== */
void set_motion(const MotorPattern *pattern, bool is_fast_gear)
{
    const uint32 pins[8] = {
        FRONT_LEFT_PIN1, FRONT_LEFT_PIN2,
        FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2,
        BACK_LEFT_PIN1, BACK_LEFT_PIN2,
        BACK_RIGHT_PIN1, BACK_RIGHT_PIN2};

    // 设置GPIO电平
    for (int i = 0; i < 8; i++)
    {
        gpio_set_level(pins[i], pattern->in[i] ? GPIO_HIGH : GPIO_LOW);
    }

    if (is_fast_gear)
    {
        // 快速状态
        if (memcmp(pattern, &PATTERN_FRONT, sizeof(MotorPattern)) == 0 ||
            memcmp(pattern, &PATTERN_BACK, sizeof(MotorPattern)) == 0)
        {
            uint32 pwm_value = FAST_GEAR_FRONT_BACK_PWM;
            pwm_set_duty(FRONT_LEFT_PWM, pwm_value);
            pwm_set_duty(FRONT_RIGHT_PWM, pwm_value);
            pwm_set_duty(BACK_LEFT_PWM, pwm_value);
            pwm_set_duty(BACK_RIGHT_PWM, pwm_value);
        }
        else
        {
            uint32 front_pwm = FAST_GEAR_LEFT_RIGHT_PWM;
            uint32 back_pwm = FAST_GEAR_FRONT_BACK_PWM;
            pwm_set_duty(FRONT_LEFT_PWM, front_pwm);
            pwm_set_duty(FRONT_RIGHT_PWM, front_pwm);
//            system_delay_ms(50);
            pwm_set_duty(BACK_LEFT_PWM, back_pwm);
            pwm_set_duty(BACK_RIGHT_PWM, back_pwm);
        }
    }
    else
    {
        // 慢速状态 - 检查八种模式并设置特定PWM值
        if (memcmp(pattern, &PATTERN_FRONT, sizeof(MotorPattern)) == 0)
        {
            pwm_set_duty(FRONT_LEFT_PWM, 2300);
            pwm_set_duty(FRONT_RIGHT_PWM, 2300);
            pwm_set_duty(BACK_LEFT_PWM, 2300);
            pwm_set_duty(BACK_RIGHT_PWM, 2300);
        }
        else if (memcmp(pattern, &PATTERN_BACK, sizeof(MotorPattern)) == 0)
        {
            pwm_set_duty(FRONT_LEFT_PWM, 2300);
            pwm_set_duty(FRONT_RIGHT_PWM, 2300);
            pwm_set_duty(BACK_LEFT_PWM, 2300);
            pwm_set_duty(BACK_RIGHT_PWM, 2300);
        }
        else if (memcmp(pattern, &PATTERN_RIGHT, sizeof(MotorPattern)) == 0)
        {
            pwm_set_duty(FRONT_LEFT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(FRONT_RIGHT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(BACK_LEFT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
            pwm_set_duty(BACK_RIGHT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
        }
        else if (memcmp(pattern, &PATTERN_LEFT, sizeof(MotorPattern)) == 0)
        {
            pwm_set_duty(FRONT_LEFT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(FRONT_RIGHT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(BACK_LEFT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
            pwm_set_duty(BACK_RIGHT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
        }
        else if (memcmp(pattern, &PATTERN_CCW, sizeof(MotorPattern)) == 0)
        {
            pwm_set_duty(FRONT_LEFT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(FRONT_RIGHT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(BACK_LEFT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
            pwm_set_duty(BACK_RIGHT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
        }
        else if (memcmp(pattern, &PATTERN_CW, sizeof(MotorPattern)) == 0)
        {
            pwm_set_duty(FRONT_LEFT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(FRONT_RIGHT_PWM, SLOW_GEAR_FRONT_WHEEL_PWM);
            pwm_set_duty(BACK_LEFT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
            pwm_set_duty(BACK_RIGHT_PWM, SLOW_GEAR_BACK_WHEEL_PWM);
        }
        else if (memcmp(pattern, &PATTERN_STOP, sizeof(MotorPattern)) == 0)
        {
            pwm_set_duty(FRONT_LEFT_PWM, 0);
            pwm_set_duty(FRONT_RIGHT_PWM, 0);
            pwm_set_duty(BACK_LEFT_PWM, 0);
            pwm_set_duty(BACK_RIGHT_PWM, 0);
        }
        else
        {
            pwm_set_duty(FRONT_LEFT_PWM, 0);
            pwm_set_duty(FRONT_RIGHT_PWM, 0);
            pwm_set_duty(BACK_LEFT_PWM, 0);
            pwm_set_duty(BACK_RIGHT_PWM, 0);
        }
    }
}

float get_imu_yaw()
{
    return gyro_data.yaw;
}

// void set_yaw(float target_yaw)
// {
//     // 允许误差范围
//     const float YAW_TOLERANCE = 1.0f; // 1度内认为到达
//     const int MAX_ITER = 200;         // 防止死循环
//     int iter = 0;
//     float current_yaw = gyro_data.yaw;

//     // 角度归一化到[-180,180]
//     while (target_yaw > 180.0f) target_yaw -= 360.0f;
//     while (target_yaw < -180.0f) target_yaw += 360.0f;

//     while (1)
//     {
//         current_yaw = gyro_data.yaw;
//         while (current_yaw > 180.0f) current_yaw -= 360.0f;
//         while (current_yaw < -180.0f) current_yaw += 360.0f;

//         float diff = target_yaw - current_yaw;
//         if (diff > 180.0f)  diff -= 360.0f;
//         if (diff < -180.0f) diff += 360.0f;

//         if (fabsf(diff) < YAW_TOLERANCE || iter++ > MAX_ITER)
//         {
//             set_motion(&PATTERN_STOP, false);
//             break;
//         }

//         if (diff > 0)
//         {
//             // 目标在当前左侧，逆时针转
//             set_motion(&PATTERN_CCW, SLOW_GEAR);
//         }
//         else
//         {
//             // 目标在当前右侧，顺时针转
//             set_motion(&PATTERN_CW, SLOW_GEAR);
//         }

//         system_delay_ms(20);
//     }
// }

/* ===================== 导航算法函数 ===================== */
void INS_UpdatePosition(INS_System *ins_sys, GyroData *data)
{
    // 更新时间间隔dt
    uint32 now = system_getval_ms();
    ins_sys->dt = (now - ins_sys->last_update_time) / 1000.0f;
    ins_sys->last_update_time = now;

    // 1. 本体加速度（g -> m/s^2）
    float ax = data->accel_x * 9.8f;
    float ay = data->accel_y * 9.8f;

    // 2. 偏航角（deg -> rad）
    float yaw_rad = data->yaw * M_PI / 180.0f;

    // 3. 坐标变换：本体 -> 世界（平面）
    float a_world[3];
    float c = cosf(yaw_rad);
    float s = sinf(yaw_rad);

    a_world[0] = ax * c - ay * s;
    a_world[1] = ax * s + ay * c;
    a_world[2] = 0.0f;

    // 4. 积分更新速度和位置（仅 xy）
    for (int i = 0; i < 2; i++)
    {
        ins_sys->velocity[i] += a_world[i] * ins_sys->dt;
        ins_sys->position[i] += ins_sys->velocity[i] * ins_sys->dt;
    }

    ins_sys->velocity[2] = 0.0f;
    ins_sys->position[2] = 0.0f;

    // 调试输出
    printf("[DEBUG] dt=%.6f\r\n", ins_sys->dt);
    printf("[DEBUG] a_world: X=%.6f Y=%.6f Z=%.6f\r\n", a_world[0], a_world[1], a_world[2]);
    printf("[DEBUG] yaw_rad = %.6f\r\n", yaw_rad);
    printf("[DEBUG] cos = %.6f sin = %.6f\r\n", c, s);
}

/* ===================== 调试输出函数 ===================== */
void print_gyro_data(void)
{
    printf("Accel: X=%.4f Y=%.4f Z=%.4f g\r\n",
           gyro_data.accel_x, gyro_data.accel_y, gyro_data.accel_z);
    printf("Gyro:  X=%.2f Y=%.2f Z=%.2f °/s\r\n",
           gyro_data.gyro_x, gyro_data.gyro_y, gyro_data.gyro_z);
    printf("Angle: Roll=%.2f Pitch=%.2f Yaw=%.2f °\r\n\r\n",
           gyro_data.roll, gyro_data.pitch, gyro_data.yaw);
}

void INS_PrintData(INS_System *ins_sys)
{
    printf("Position: X=%.2f Y=%.2f Z=%.2f m\r\n",
           ins_sys->position[0], ins_sys->position[1], ins_sys->position[2]);
    printf("Velocity: Vx=%.2f Vy=%.2f Vz=%.2f m/s\r\n\r\n",
           ins_sys->velocity[0], ins_sys->velocity[1], ins_sys->velocity[2]);
}

/* ===================== 中断处理函数 ===================== */
void uart_rx_interrupt_handler(void)
{
    // 仅从硬件 UART 取 1 字节并写入软件 FIFO
    uint8 b = uart_read_byte(READ_UART);
    fifo_write_buffer(&uart_data_fifo, &b, 1);
}

#pragma section all restore
