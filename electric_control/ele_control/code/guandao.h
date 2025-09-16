#ifndef GUANDAO_H
#define GUANDAO_H

#include "zf_common_headfile.h"
#include <math.h>

/* ===================== 硬件引脚定义 ===================== */
#define READ_UART (UART_3)
// #define WRITE_UART (UART_0)

#define IMU_TX_PIN (UART3_TX_P15_6)
#define IMU_RX_PIN (UART3_RX_P15_7)

// #define TC264_TX_PIN (UART0_TX_P14_0)
// #define TC264_RX_PIN (UART0_RX_P14_1)

#define BACK_LEFT_PIN1 (P11_10)
#define BACK_LEFT_PIN2 (P11_11)
#define BACK_RIGHT_PIN1 (P10_3)
#define BACK_RIGHT_PIN2 (P11_12)
#define FRONT_RIGHT_PIN1 (P02_1)
#define FRONT_RIGHT_PIN2 (P00_2)
#define FRONT_LEFT_PIN1 (P02_2)
#define FRONT_LEFT_PIN2 (P10_1)

#define BACK_LEFT_PWM (ATOM1_CH3_P02_3)
#define BACK_RIGHT_PWM (ATOM1_CH4_P02_4)
#define FRONT_RIGHT_PWM (ATOM1_CH5_P02_5)
#define FRONT_LEFT_PWM (ATOM1_CH6_P02_6)

// 档位宏定义
#define FAST_GEAR 1
#define SLOW_GEAR 0

#define FAST_GEAR_FRONT_BACK_PWM 4000
#define FAST_GEAR_FRONT_WHEEL_PWM 5250
#define FAST_GEAR_BACK_WHEEL_PWM 3400

#define SLOW_GEAR_FRONT_BACK_PWM 3000
#define SLOW_GEAR_FRONT_WHEEL_PWM 3210
#define SLOW_GEAR_BACK_WHEEL_PWM 1500

#define UART_BAUDRATE (115200)
#define GYRO_DATA_BUFFER_SIZE 128
#define GYRO_PACKET_SIZE 11

#ifndef M_PI
#define M_PI 3.1416f
#endif

/* ===================== 类型定义 ===================== */
// 陀螺仪数据结构体
typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float yaw; // 顺时针转，yaw角减小
} GyroData;

// 惯性导航系统结构体
typedef struct
{
    GyroData sensor_data;
    float position[3];
    float velocity[3];
    uint32 last_update_time;
    float dt;
} INS_System;

// 电机控制模式结构体
typedef struct
{
    uint8 in[8];
} MotorPattern;

/* ===================== 外部常量声明 ===================== */
extern const MotorPattern PATTERN_FRONT;
extern const MotorPattern PATTERN_BACK;
extern const MotorPattern PATTERN_LEFT;
extern const MotorPattern PATTERN_RIGHT;
extern const MotorPattern PATTERN_CW;
extern const MotorPattern PATTERN_CCW;
extern const MotorPattern PATTERN_STOP;

/* ===================== 外部变量声明 ===================== */
extern GyroData gyro_data;
extern INS_System ins;

/* ===================== 函数声明 ===================== */
// 初始化函数
void guandao_init(void);
void imu_init(void);
void motion_init(void);
void INS_Init(INS_System *ins);
void gyro_data_init(void);

// 数据处理函数
void unpack_and_analyze_imu_data(void);
void parse_gyro_data(uint8 *data, uint8 type);
uint8 checksum(uint8 *data, uint8 len, uint8 sum);

// 运动控制函数
void set_motion(const MotorPattern *pattern, bool is_fast_gear);

// 导航算法函数
void INS_UpdatePosition(INS_System *ins, GyroData *data);

// 调试输出函数
void print_gyro_data(void);
void INS_PrintData(INS_System *ins);

// 中断处理函数
void uart_rx_interrupt_handler(void);

#endif // GUANDAO_H
