## 电控子模块（ele_control）技术文档

该文档描述 `electric_control/ele_control` 下的代码结构、主要模块、硬件 IO 映射、以及与 PC 的串口通信协议（frame + TLV）。

> 项目基于`libraries/`（SeekFree/厂商库）开发，这些库的内部实现不在本文档中详述，仅说明如何使用本目录下的接口。

---

## 目录

- [电控子模块（ele\_control）技术文档](#电控子模块ele_control技术文档)
- [目录](#目录)
- [概述](#概述)
- [硬件模块](#硬件模块)
  - [主板](#主板)
  - [电机驱动](#电机驱动)
  - [舵机转接板](#舵机转接板)
  - [PCB设计](#pcb设计)
    - [板层结构](#板层结构)
    - [布局规划](#布局规划)
    - [布线规范](#布线规范)
    - [电源监控和管理](#电源监控和管理)
    - [功率分配](#功率分配)
  - [元件选型](#元件选型)
    - [核心器件](#核心器件)
    - [被动元件](#被动元件)
    - [连接器](#连接器)
- [代码结构](#代码结构)
- [执行器](#执行器)
  - [底盘控制 (guandao)](#底盘控制-guandao)
  - [推送装置 (push)](#推送装置-push)
  - [机械臂控制 (roboarm\_motion)](#机械臂控制-roboarm_motion)
  - [发射控制 (shot)](#发射控制-shot)
- [串口通信及通信协议](#串口通信及通信协议)
  - [协议架构](#协议架构)
  - [Frame层](#frame层)
  - [Data/TLV层](#datatlv层)
  - [变量定义](#变量定义)
  - [典型交互流程](#典型交互流程)
- [`user/cpu0_main.c` 执行流程说明](#usercpu0_mainc-执行流程说明)
  - [初始化阶段](#初始化阶段)
  - [主循环](#主循环)
- [附录：硬件 IO 映射](#附录硬件-io-映射)
  - [IMU（串口）](#imu串口)
  - [底盘电机控制](#底盘电机控制)
  - [推送模块](#推送模块)
  - [舵机控制](#舵机控制)
  - [无刷电机控制](#无刷电机控制)
  - [电压反馈](#电压反馈)

---

## 概述

`ele_control` 是机器人电控（MCU 端）代码的用户层实现，基于项目内提供的硬件抽象与第三方库（libraries）。本目录负责：

- 车辆底盘与电机驱动（`guandao.*`）
- 弹仓推进装置（`push.*`）
- 机械臂（`roboarm_motion.*`）
- 发射装置（`shot.*`）
- 与上位机（PC）基于自定义 frame + TLV 协议的收发（`communicate/*`）

本技术文档聚焦代码结构、接口与协议，便于移植、调试与二次开发。

## 硬件模块

### 主板

**主电源输入**
- 输入接口：DC-005电源插座
- 输入电压：12V DC ±10%
- 过压保护：TVS管 P6SMBJ18CA
- 反接保护：SS34肖特基二极管

**DC-DC转换电路**
- 5V转换：MP2451 buck转换器
  - 输出电流：3A最大
  - 效率：>92%
  - 外围器件：10μH电感，22μF输出电容
- 3.3V转换：AMS1117-3.3 LDO
  - 输出电流：1A
  - 输入范围：4.5V-12V
  - 滤波：10μF陶瓷电容

### 电机驱动
**H桥驱动模块**
- 驱动芯片：DRV8833双H桥
- 逻辑电压：3.3V兼容
- 续流二极管：内置MOSFET体二极管
- 电流采样：0.1Ω采样电阻

**无刷电机驱动**
- 驱动芯片：三相桥式驱动器
- 预驱电路：自举电容设计
- 死区时间：100ns硬件设置

### 舵机转接板
**电源接口**
- 电平转换：74LVC4245电平转换器
- 滤波电路：RC低通滤波器，fc=1MHz
- ESD保护：ESD9L5.0 TVS阵列

**PWM输出**
- 缓冲器：74HC245总线驱动器
- 限流电阻：220Ω串联保护

### PCB设计

#### 板层结构
- 层数：4层板设计
- 叠层：信号层-地层-电源层-信号层
- 板厚：1.6mm FR-4材料
- 铜厚：外层1oz，内层0.5oz

#### 布局规划
**电源区域**
- 位置：板卡右侧
- 特征：大面积铺铜，多路电源分区
- 散热：电源芯片带散热过孔

**电机驱动区域**
- 位置：板卡四周边缘
- 布线：大电流路径加宽至40mil
- 隔离：数字地与功率地单点连接

#### 布线规范
**电源布线**
- 主电源：60mil线宽，承载3A电流
- 电机电源：80mil线宽，承载6A电流
- 地平面：完整地平面，减少环路面积

#### 电源监控和管理
**电压监测点**
- 主输入：12V分压监测
- 电池电压：通过ADC1_CH5_A21监测
- 3.3V供电：直接ADC采样

**电流监测**
- 采样电阻：5mΩ精密电阻
- 放大器：INA282差分放大器
- 增益：50V/V固定增益

#### 功率分配
**总功率预算**
- 主板功耗：1.2W（3.3V@360mA）
- 舵机功耗：最大36W（6V@6A）
- 电机功耗：最大120W（12V@10A）
- 总输入功率：150W设计余量

**保护电路**
- 保险丝：自恢复保险丝 PPTC
- 过流保护：电流监控芯片 INA199
- 温度监控：NTC热敏电阻


### 元件选型

#### 核心器件
**HPM6750微控制器**
- 封装：LQFP176
- 温度范围：-40℃ to +85℃
- 供电电压：3.3V核心，1.2V内核

**DRV8833电机驱动**
- 封装：HTSSOP16
- Rds(on)：280mΩ典型值
- 待机电流：<1μA

#### 被动元件
**电容选型**
- 去耦电容：100nF 0402封装
- 滤波电容：10μF 0805陶瓷电容
- 电解电容：220μF 耐压25V

**电阻选型**
- 信号电阻：1%精度 0402封装
- 功率电阻：1%精度 1206封装
- 采样电阻：0.5%精度 2512封装

#### 连接器
**电机接口**
- 类型：PH2.0-4P连接器
- 电流：每针5A额定
- 锁扣：带机械锁扣设计

**传感器接口**
- 类型：DF13-6P连接器
- 间距：1.25mm间距
- 线序：防误插设计


## 代码结构

```
ele_control/
├── libraries/          # 开发中使用的底层库
├── code/
│   ├── communicate/    
│   │   ├── protocol_c  # 协议栈（frame, data, protocol_defs）
│   │   ├── parser.*    # 接收解析器
│   │   └── sender.*    # 数据发送器
│   ├── guandao.*       # IMU接收、运动模式控制、导航辅助函数
│   ├── push.*          # 送管/推弹子模块控制逻辑
│   ├── shot.*          # 开火/触发子模块
│   ├── roboarm_motion.*# 机械臂动作序列
│   └── 本文件夹作用.txt
└── user/
    ├── cpu0_main.c     # CPU0主函数（初始化流程与主循环）
    ├── cpu1_main.c     # 第二个核心程序（未使用）
    ├── cpu0_main.h
    └── isr.*           # 启动/中断相关
```

## 执行器

### 底盘控制 (guandao)

**文件**: `guandao.c / guandao.h`

**功能概述**：
IMU 串口接收解析（FIFO）、惯导（INS）基础结构、底盘 motor pattern、PWM/GPIO 控制、IMU 数据解析。

**主要接口**：
- `guandao_init()` - 底盘系统初始化
- `imu_reset()` - IMU 复位
- `motion_init()` - 运动控制初始化
- `set_motion()` - 设置运动状态
- `get_imu_yaw()` - 获取 IMU 偏航角
- `INS_UpdatePosition()` - 更新位置信息

**硬件依赖**：
- IMU 串口 (UART3)
- 4路电机 PWM 输出
- 8路电机 GPIO 控制

### 推送装置 (push)

**文件**: `push.c / push.h`

**功能概述**：
推管/送弹逻辑（编码器读数判断极限、PWM 控制、方向 GPIO），实现精确的推送控制。

**主要接口**：
- `push_init()` - 推送装置初始化
- `push_update()` - 状态更新与监控
- `push_forward_and_back()` - 前进后退控制
- `push_set_speed()` - 设置推送速度
- `push_get_position()` - 获取当前位置

**硬件依赖**：
- 编码器接口 (TIM2)
- PWM 输出通道
- 方向控制 GPIO

### 机械臂控制 (roboarm_motion)

**文件**: `roboarm_motion.c / roboarm_motion.h`

**功能概述**：
机械臂动作序列控制，包括抓取、准备、回位等预设动作，通过调用底层舵机接口实现精确运动控制。

**主要接口**：
- `roboarm_init()` - 机械臂初始化
- `roboarm_grab()` - 执行抓取动作
- `roboarm_prepare()` - 准备姿态
- `roboarm_reset()` - 回位动作
- `roboarm_set_angle()` - 设置关节角度

**硬件依赖**：
- 多路舵机 PWM 输出
- 限位开关检测

### 发射控制 (shot)

**文件**: `shot.c / shot.h`

**功能概述**：
开火/触发控制模块，管理摩擦轮速度控制、单发/连发模式、发射时序等。

**主要接口**：
- `shot_init()` - 发射系统初始化
- `shot_fire_once()` - 单发射击
- `shot_fire_continuous()` - 连发射击
- `shot_stop()` - 停止射击
- `shot_set_speed()` - 设置摩擦轮速度

**硬件依赖**：
- 摩擦轮 PWM 控制
- 触发机构 GPIO
- 弹丸检测传感器

---

## 串口通信及通信协议

### 协议架构

协议分三层：
1. **Frame 层** - 数据帧边界与校验
2. **Data 层** - TLV 编解码
3. **Variable 层** - 变量定义与语义

### Frame层

**帧结构**：
```
0xAA | LEN | VER | SEQ | CHK | DATA[LEN-3]... | 0x55
```

- **LEN** = 3 + N（N 是 DATA 字节数），DATA 最大长度 = 252 字节
- **头尾字节**：`PCMCU_FRAME_HEAD = 0xAA`，`PCMCU_FRAME_TAIL = 0x55`

**主要 API**：
- `pcmcu_build_frame()` - 构建帧
- `pcmcu_parse_frame_data()` / `pcmcu_stream_feed()` - 解析数据流

### Data/TLV层

**TLV 格式**：
- **T** (1 byte) - 类型
- **L** (1 byte) - 长度（0-255）
- **V** (L bytes) - 数据值

**编码 API**：
- `data_put_u8/u16le/u32le/f32le()` - 基础类型编码
- `data_put_tlv()` - TLV 编码
- `data_put_var()` - 变量编码（校验固定宽度）
- `data_encode()` - 高层数据包构建

### 变量定义

变量定义在 `protocol_c/protocol_defs.h` 中，部分示例如下：

| 变量名 | ID | 大小 | 说明 |
|--------|----|------|------|
| `VAR_FRICTION_WHEEL_SPEED` | 0x01 | 4字节 | 摩擦轮速度/功率 |
| `VAR_FIRE_ONCE` | 0x41 | 1字节 | 触发射击 |
| `VAR_GET_IMU_YAW` | 0xA3 | 0字节 | 请求 IMU yaw |
| `VAR_VOLTAGE` | 0xBB | 4字节 | 电压反馈 |
| `VAR_HEARTBEAT` | 0xD1 | 1字节 | 心跳 |

完整变量表见：`electric_control/ele_control/protocol_vars.csv`

### 典型交互流程

1. **上位机 → MCU**：
   - 构建 DATA(TLV)（如 T=VAR_FIRE_ONCE, L=1, V=0x01）
   - 使用 `data_encode()` 编码 DATA
   - 使用 `pcmcu_build_frame()` 包装帧
   - 通过串口发送

2. **MCU 处理**：
   - 解析器接收数据流
   - 在 `parser.c:on_tlv_callback` 中根据 VAR 执行相应动作
   - 将结果写入响应缓冲
   - 调用 `sender_send_multiple_tlv()` 发送回包

---

## `user/cpu0_main.c` 执行流程说明

### 初始化阶段

1. **基础初始化**：
   - `clock_init()` - 时钟初始化
   - `debug_init()` - 调试串口初始化

2. **协议与解析器初始化**：
   - `sender_init(sender_callback, NULL)` - 发送器初始化
   - `parser_init()` - 解析器初始化

3. **模块初始化**：
   - `guandao_init()` - 底盘初始化
   - `push_init()` - 推送模块初始化
   - `roboarm_init()` - 机械臂初始化
   - `shot_init()` - 射击模块初始化

4. **等待其他核就绪**：
   - `cpu_wait_event_ready()`

### 主循环

```c
while (1) {
    // 从串口环形缓冲读取数据
    debug_read_ring_buffer(read_data, 32);
    
    // 数据解析与处理
    if (有数据) {
        parser_feed_stream(read_data, fifo_data_size);
    }
    
    // 可选：周期性发送心跳
    // sender_send_u8(VAR_HEARTBEAT, 0x00);
}
```

**设计要点**：将 IO/外设初始化与协议栈初始化放在主函数前期，主循环仅负责把串口收数据交给解析器。

---

## 附录：硬件 IO 映射

### IMU（串口）
- **RX/TX**: `IMU_RX_PIN = UART3_RX_P15_7`, `IMU_TX_PIN = UART3_TX_P15_6`
- **波特率**: 115200

### 底盘电机控制
**GPIO 控制**：
- `FRONT_LEFT_PIN1 = P02_2`, `FRONT_LEFT_PIN2 = P10_1`
- `FRONT_RIGHT_PIN1 = P02_1`, `FRONT_RIGHT_PIN2 = P00_2`
- `BACK_LEFT_PIN1 = P11_10`, `BACK_LEFT_PIN2 = P11_11`
- `BACK_RIGHT_PIN1 = P10_3`, `BACK_RIGHT_PIN2 = P11_12`

**PWM 通道**：
- `FRONT_LEFT_PWM = ATOM1_CH6_P02_6`
- `FRONT_RIGHT_PWM = ATOM1_CH5_P02_5`
- `BACK_LEFT_PWM = ATOM1_CH3_P02_3`
- `BACK_RIGHT_PWM = ATOM1_CH4_P02_4`

### 推送模块
- `DIRECTION_PIN = P13_2`
- `SPEED_PWM = ATOM3_CH0_P13_3`
- `ENCODER_A_PIN = TIM2_ENCODER_CH1_P33_7`
- `ENCODER_B_PIN = TIM2_ENCODER_CH2_P33_6`
- 编码器：`TIM2_ENCODER`

### 舵机控制
**PWM 通道**：
- `DI_PWM = ATOM0_CH2_P00_3`
- `DABI_PWM = ATOM0_CH3_P00_4`
- `ZHONGBI_PWM = ATOM0_CH4_P00_5`
- `XIAOBI_PWM = ATOM0_CH5_P00_6`
- `SHOUWAN_PWM = ATOM0_CH6_P00_7`
- `GRIPPER_PWM = ATOM0_CH7_P00_8`

### 无刷电机控制
**PWM 通道**：
- `SHOT_FRONT_LEFT = ATOM2_CH2_P11_3`
- `SHOT_FRONT_RIGHT = ATOM2_CH1_P11_2`
- `SHOT_BACK_LEFT = ATOM2_CH6_P13_1`
- `SHOT_BACK_RIGHT = ATOM2_CH5_P13_0`

### 电压反馈
**ADC 通道**：
- `ADC_CHANNEL = ADC1_CH5_A21`


---