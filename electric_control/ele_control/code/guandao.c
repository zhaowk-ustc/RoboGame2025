#include "guandao.h"

#pragma section all "cpu0_dsram"

/* ===================== 全局变量定义 ===================== */
uint8 fifo_get_data[GYRO_DATA_BUFFER_SIZE];
uint8 raw_packet[GYRO_PACKET_SIZE];
uint8 packet_index = 0;
uint8 get_data = 0;
uint32 fifo_data_count = 0;

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
// 左移：
const MotorPattern PATTERN_LEFT = {{0, 1, 1, 0, 1, 0, 0, 1}};
// 右移：
const MotorPattern PATTERN_RIGHT = {{1, 0, 0, 1, 0, 1, 1, 0}};
// 停止
const MotorPattern PATTERN_STOP = {{0, 0, 0, 0, 0, 0, 0, 0}};

/* ===================== 初始化函数 ===================== */
void guandao_init(void)
{
    clock_init();
    debug_init();

    // 初始化FIFO
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, fifo_get_data, GYRO_DATA_BUFFER_SIZE);

    // 初始化导航系统
    INS_Init(&ins);
    control_init();

    // 初始化串口
    uart_init(WRITE_UART, UART_BAUDRATE, TC264_TX_PIN, TC264_RX_PIN);
    uart_init(READ_UART, UART_BAUDRATE, IMU_TX_PIN, IMU_RX_PIN);
    uart_rx_interrupt(READ_UART, 1);

    // 初始化IMU
    imu_init();

    uart_write_string(WRITE_UART, "Guandao System Initialized.\r\n");
}

void imu_init(void)
{
    uint8 tx_data[] = {0xFF, 0xAA, 0x67};
    uart_write_buffer(UART_3, tx_data, sizeof(tx_data));
}

void control_init(void)
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
    ins->quaternion[0] = 1.0f;
    ins->quaternion[1] = 0.0f;
    ins->quaternion[2] = 0.0f;
    ins->quaternion[3] = 0.0f;

    for (int i = 0; i < 3; i++)
    {
        ins->position[i] = 0.0f;
        ins->velocity[i] = 0.0f;
    }

    ins->last_update_time = system_getval_ms();
    ins->dt = 0.0f;
}

/* ===================== 数据处理函数 ===================== */
void unpack_and_analyze_imu_data(void)
{
    uint32 fifo_data_count = fifo_used(&uart_data_fifo);
    if (fifo_data_count > 0)
    {
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
        for (uint32 i = 0; i < fifo_data_count; i++)
        {
            if (fifo_get_data[i] == 0x55)
            {
                packet_index = 0;
            }
            raw_packet[packet_index++] = fifo_get_data[i];

            if (packet_index > GYRO_PACKET_SIZE)
            {
                packet_index = 0;
                continue;
            }

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
                        INS_UpdateAttitude(&ins, &gyro_data);
                        INS_UpdatePosition(&ins, &gyro_data);
                        INS_PrintData(&ins);
                    }
                }
                packet_index = 0;
            }
        }
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
        int16 temp = (data[9] << 8) | data[8];

        gyro_data.accel_x = (float)ax / 32768.0f * 16.0f;
        gyro_data.accel_y = (float)ay / 32768.0f * 16.0f;
        gyro_data.accel_z = (float)az / 32768.0f * 16.0f;
        gyro_data.temp = (float)temp / 32768.0f * 96.38f + 36.53f;
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
void set_motion(const MotorPattern *pattern, uint16 pwm_val)
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
    pwm_set_duty(BACK_LEFT_PWM, pwm_val);
    pwm_set_duty(BACK_RIGHT_PWM, pwm_val);
    pwm_set_duty(FRONT_RIGHT_PWM, pwm_val);
    pwm_set_duty(FRONT_LEFT_PWM, pwm_val);
}

/* ===================== 导航算法函数 ===================== */
void Quaternion_Normalize(float q[4])
{
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0.0f)
    {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

void Quaternion_Multiply(float result[4], const float q1[4], const float q2[4])
{
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

void Quaternion_ToEuler(float q[4], float euler[3])
{
    euler[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));

    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) >= 1.0f)
        euler[1] = copysignf(M_PI / 2.0f, sinp);
    else
        euler[1] = asinf(sinp);

    euler[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

void INS_UpdateAttitude(INS_System *ins, GyroData *data)
{
    uint32 current_time = system_getval_ms();
    ins->dt = (current_time - ins->last_update_time) / 1000.0f;
    ins->last_update_time = current_time;

    float wx = data->gyro_x * M_PI / 180.0f;
    float wy = data->gyro_y * M_PI / 180.0f;
    float wz = data->gyro_z * M_PI / 180.0f;

    float q_dot[4];
    q_dot[0] = 0.5f * (-ins->quaternion[1] * wx - ins->quaternion[2] * wy - ins->quaternion[3] * wz);
    q_dot[1] = 0.5f * (ins->quaternion[0] * wx + ins->quaternion[2] * wz - ins->quaternion[3] * wy);
    q_dot[2] = 0.5f * (ins->quaternion[0] * wy - ins->quaternion[1] * wz + ins->quaternion[3] * wx);
    q_dot[3] = 0.5f * (ins->quaternion[0] * wz + ins->quaternion[1] * wy - ins->quaternion[2] * wx);

    ins->quaternion[0] += q_dot[0] * ins->dt;
    ins->quaternion[1] += q_dot[1] * ins->dt;
    ins->quaternion[2] += q_dot[2] * ins->dt;
    ins->quaternion[3] += q_dot[3] * ins->dt;

    Quaternion_Normalize(ins->quaternion);
    Quaternion_ToEuler(ins->quaternion, ins->euler_angles);
}

void INS_UpdatePosition(INS_System *ins, GyroData *data)
{
    float q0 = ins->quaternion[0], q1 = ins->quaternion[1];
    float q2 = ins->quaternion[2], q3 = ins->quaternion[3];

    float R11 = 2.0f * (q0 * q0 + q1 * q1) - 1.0f;
    float R12 = 2.0f * (q1 * q2 - q0 * q3);
    float R13 = 2.0f * (q1 * q3 + q0 * q2);

    float R21 = 2.0f * (q1 * q2 + q0 * q3);
    float R22 = 2.0f * (q0 * q0 + q2 * q2) - 1.0f;
    float R23 = 2.0f * (q2 * q3 - q0 * q1);

    float accel_world[3];
    accel_world[0] = R11 * (data->accel_x * 9.8f) + R12 * (data->accel_y * 9.8f) + R13 * (data->accel_z - 1) * 9.8f;
    accel_world[1] = R21 * (data->accel_x * 9.8f) + R22 * (data->accel_y * 9.8f) + R23 * (data->accel_z - 1) * 9.8f;
    accel_world[2] = 0;

    for (int i = 0; i < 3; i++)
    {
        ins->velocity[i] += accel_world[i] * ins->dt;
        ins->position[i] += ins->velocity[i] * ins->dt;
    }
}

/* ===================== 调试输出函数 ===================== */
void print_gyro_data(void)
{
    char buffer[128];
    sprintf(buffer, "Accel: X=%.4f Y=%.4f Z=%.4f g\r\n", gyro_data.accel_x, gyro_data.accel_y, gyro_data.accel_z);
    uart_write_string(WRITE_UART, buffer);

    sprintf(buffer, "Temp: %.2f °C\r\n", gyro_data.temp);
    uart_write_string(WRITE_UART, buffer);

    sprintf(buffer, "Gyro: X=%.2f Y=%.2f Z=%.2f °/s\r\n", gyro_data.gyro_x, gyro_data.gyro_y, gyro_data.gyro_z);
    uart_write_string(WRITE_UART, buffer);

    sprintf(buffer, "Angle: Roll=%.2f Pitch=%.2f Yaw=%.2f °\r\n\r\n", gyro_data.roll, gyro_data.pitch, gyro_data.yaw);
    uart_write_string(WRITE_UART, buffer);
}

void INS_PrintData(INS_System *ins)
{
    char buffer[128];
    sprintf(buffer, "Attitude: Roll=%.2f Pitch=%.2f Yaw=%.2f °\r\n",
            ins->euler_angles[0] * 180.0f / M_PI,
            ins->euler_angles[1] * 180.0f / M_PI,
            ins->euler_angles[2] * 180.0f / M_PI);
    uart_write_string(WRITE_UART, buffer);

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
