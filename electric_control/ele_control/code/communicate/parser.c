
#include "parser.h"
#include "protocol_c/frame.h"
#include "protocol_c/data.h"
#include "protocol_c/protocol_defs.h"
#include "sender.h"
#include "push.h"
#include "shot.h"
#include "guandao.h"
#include "roboarm_motion.h"
#include <string.h>

static data_parser_t _parser = {
    .stream = {{0}},      // 初始化流式解析器
    .current_msg = {{0}}, // 清空当前消息
};

// ================= 内部静态变量和函数声明 =================

// 响应数据缓存结构
typedef struct
{
    tlv_item_t items[16]; // 最多16个响应项
    size_t item_count;    // 当前响应项数量
} response_buffer_t;

// 全局响应缓存
static response_buffer_t g_response_buffer = {{0}, 0};

// 每个响应项的内联存储（最长4字节）
static uint8_t g_resp_storage[16][4];

/**
 * @brief 添加响应数据到全局缓存（内部完成拷贝）
 * @param var_id 变量ID
 * @param data_ptr 输入数据指针（可为局部/临时变量）
 * @param data_len 数据长度（最大4）
 * @return 0成功，-1失败
 */
static int add_response_to_buffer(uint8_t var_id, const void *data_ptr, uint8_t data_len)
{
    if (!data_ptr || data_len == 0 || data_len > 4)
    {
        return -1; // 参数非法
    }
    if (g_response_buffer.item_count >= 16)
    {
        return -1; // 缓存已满
    }

    size_t idx = g_response_buffer.item_count;

    // 拷贝到内联存储，并让TLV条目指向该存储
    memcpy(g_resp_storage[idx], data_ptr, data_len);

    g_response_buffer.items[idx].var_id = var_id;
    g_response_buffer.items[idx].data = g_resp_storage[idx];
    g_response_buffer.items[idx].len = data_len;

    g_response_buffer.item_count++;
    return 0;
}

/** TLV解析回调函数 */
static int on_tlv_callback(uint8_t t, const uint8_t *v, uint8_t l, void *user)
{
    parsed_message_t *msg = (parsed_message_t *)user;

    if (!msg || msg->var_count >= 16)
    {
        return -1; // 参数错误或变量数组已满
    }

    // 将解析出的TLV添加到消息结构中
    msg->vars[msg->var_count].t = t;
    msg->vars[msg->var_count].l = l;
    msg->vars[msg->var_count].v = v;
    msg->var_count++;
    switch (t)
    {
    /* ===== A: ARM ===== */
    case VAR_ARM_RESET:
        // 复位/初始化
        arm_reset();
        break;
    case VAR_ARM_SHOT_TO_RESET:
        // 射击位 -> 初始化
        arm_shot_to_reset();
        break;

    case VAR_ARM_RESET_TO_STORE:
        // 初始化 -> 存储位
        // TODO: 补齐具体动作函数，例如 arm_reset_to_store()
        arm_reset_to_store();
        break;

    case VAR_ARM_RESET_TO_HIGH_PREPARE:
        // 初始化 -> 高位准备
        arm_reset_to_high_prepare();
        break;

    case VAR_ARM_RESET_TO_LOW_PREPARE:
        // 初始化 -> 低位准备
        arm_reset_to_prepare();
        break;

    case VAR_ARM_HIGH_GRIP_TO_SHOT:
        // 高位夹取 -> 射击位
        arm_high_grip_to_shot();
        break;

    case VAR_ARM_STORE_TO_SHOT:
        // 存储位 -> 射击位
        // TODO: 补齐具体动作函数，例如 arm_store_to_shot()
        arm_store_to_shot();
        break;

    case VAR_ARM_LOW_GRIP_TO_WAIT_SHOT:
        // 低位夹取 -> 等待射击
        // TODO: 补齐具体动作函数，例如 arm_grip_to_wait_shot()
        arm_grip_to_wait_shot();
        break;

    case VAR_ARM_LOW_PREPARE_TO_GRIP:
        // 低位准备 -> 低位夹取
        arm_prepare_to_grip();
        break;

    case VAR_ARM_HIGH_PREPARE_TO_GRIP:
        // 高位准备 -> 高位夹取
        arm_high_prepare_to_grip();
        break;

    case VAR_ARM_RELAX:
        // 夹爪松弛（掉电保护位/低力）
        arm_relax();
        break;

    case VAR_ARM_LOW_GRIP_TO_STORE:
        // 低位夹取 -> 存储位
        // TODO: 补齐具体动作函数，例如 arm_grip_to_store()
        arm_grip_to_store();
        break;

    case VAR_ARM_HIGH_GRIP_TO_WAIT_SHOT:
        // 高位夹取 -> 等待射击
        // TODO: 补齐具体动作函数，例如 arm_grip_to_wait_shot()
        arm_grip_to_wait_shot();
        break;

    case VAR_ARM_WAIT_SHOT_TO_SHOT:
        // 等待射击 -> 射击位
        // TODO: 补齐具体动作函数，例如 arm_wait_shot_to_shot()
        arm_wait_shot_to_shot();
        break;

    case VAR_ARM_HIGH_GRIP_TO_STORE:
        // 高位夹取 -> 存储位
        // TODO: 补齐具体动作函数，例如 arm_grip_to_store()
        arm_high_grip_to_store();
        break;

    case VAR_ARM_LOW_GRIP_TO_SHOT:
        // 低位夹取 -> 射击位
        arm_grip_to_shot();
        break;

    case VAR_ARM_STORE_TO_RESET:
        // 存储位 -> 初始化
        // TODO: 若有 arm_store_to_reset() 则调用
        arm_store_to_reset();
        break;

        /* ===== B: BASE ===== */
    case VAR_BASE_MOVE_BACKWARD_FAST:
        // 底盘后退（快速）
        set_motion(&PATTERN_BACK, FAST_GEAR);
        break;
    case VAR_BASE_MOVE_BACKWARD_FAST_EX:
        // 底盘后退（极快速）
        // TODO: 若有更快的模式，可替换此处
        set_motion(&PATTERN_BACK, FAST_GEAR);
        break;

    case VAR_BASE_MOVE_BACKWARD_SLOW:
        // 底盘后退（慢速）
        set_motion(&PATTERN_BACK, SLOW_GEAR);
        break;

    case VAR_BASE_MOVE_FORWARD_FAST:
        // 底盘前进（快速）
        set_motion(&PATTERN_FRONT, FAST_GEAR);
        break;
    case VAR_BASE_MOVE_FORWARD_FAST_EX:
        // 底盘前进（极快速）
        // TODO: 若有更快的模式，可替换此处
        set_motion(&PATTERN_FRONT, FAST_GEAR);
        break;

    case VAR_BASE_MOVE_FORWARD_SLOW:
        // 底盘前进（慢速）
        set_motion(&PATTERN_FRONT, SLOW_GEAR);
        break;

    case VAR_BASE_MOVE_LEFT_FAST:
        // 底盘左移（快速）
        set_motion(&PATTERN_LEFT, FAST_GEAR);
        break;

    case VAR_BASE_MOVE_LEFT_SLOW:
        // 底盘左移（慢速）
        set_motion(&PATTERN_LEFT, SLOW_GEAR);
        break;

    case VAR_BASE_MOVE_RIGHT_FAST:
        // 底盘右移（快速）
        set_motion(&PATTERN_RIGHT, FAST_GEAR);
        break;

    case VAR_BASE_MOVE_RIGHT_SLOW:
        // 底盘右移（慢速）
        set_motion(&PATTERN_RIGHT, SLOW_GEAR);
        break;

    case VAR_BASE_ROTATE_CCW_FAST:
        // 底盘逆时针旋转（快速）
        set_motion(&PATTERN_CCW, FAST_GEAR);
        break;

    case VAR_BASE_ROTATE_CCW_SLOW:
        // 底盘逆时针旋转（慢速）
        set_motion(&PATTERN_CCW, SLOW_GEAR);
        break;

    case VAR_BASE_ROTATE_CW_FAST:
        // 底盘顺时针旋转（快速）
        set_motion(&PATTERN_CW, FAST_GEAR);
        break;

    case VAR_BASE_ROTATE_CW_SLOW:
        // 底盘顺时针旋转（慢速）
        set_motion(&PATTERN_CW, SLOW_GEAR);
        break;

    case VAR_BASE_STOP:
        // 底盘停止
        set_motion(&PATTERN_STOP, FALSE);
        break;

        /* ===== D: DART / FEEDER ===== */
    case VAR_DART_PUSH_ONCE:
        // 送料/管道推弹一次
        // TODO: 若 guandao_push_once() 可用，则启用
        // guandao_push_once();
        break;

        /* ===== E: ERROR (reserved for echo back) ===== */
        // VAR_ERROR 仅用于回传，不做主动处理分支

        /* ===== F: FIRE / FRICTION ===== */
    case VAR_FIRE_ONCE:
        // 发射一次
        shot_fire_once();
        break;

    case VAR_FRICTION_WHEEL_SPEED:
        // 摩擦轮速度 (4字节，小端 float)
        {
            float value = 0.0f;
            if (data_read_f32le(v, 4, &value) == DATA_OK)
            {
                bldc_set_speed(value);
            }
        }
        break;

    case VAR_FRICTION_WHEEL_START:
        // 摩擦轮启动
        // TODO: 替换为你项目中的实际接口，如 shot_friction_start()
        // shot_friction_start();
        break;

    case VAR_FRICTION_WHEEL_STOP:
        // 摩擦轮停止
        // TODO: 替换为你项目中的实际接口，如 shot_friction_stop()
        // shot_friction_stop();
        break;

        /* ===== G: GET/IMU ===== */
    case VAR_GET_IMU_YAW:
    {
        // 获取IMU偏航角并回传
        while (1)
        {
            if (unpack_imu_data())
            {
                break;
            }
        }
        float yaw = get_imu_yaw();
        add_response_to_buffer(VAR_IMU_YAW, &yaw, 4);
    }
    break;

    /* ===== H: HEARTBEAT ===== */
    case VAR_HEARTBEAT:
        // 心跳回显
        add_response_to_buffer(VAR_HEARTBEAT, v, 1);
        break;

        /* ===== I: IMU ===== */
    case VAR_IMU_RESET:
        // IMU复位
        imu_reset();
        break;

        /* ===== T: TEST / TURRET ===== */
    case VAR_TEST_VAR_F32:
    {
        // 测试变量 float：+0.1 回传
        float value;
        if (data_read_f32le(v, 4, &value) == DATA_OK)
        {
            value += 0.1f;
            add_response_to_buffer(VAR_TEST_VAR_F32, &value, 4);
        }
    }
    break;

    case VAR_TEST_VAR_U16:
    {
        // 测试变量 u16：+10 回传（小端）
        uint16_t value = (uint16_t)(v[0] | ((uint16_t)v[1] << 8));
        value = (uint16_t)(value + 10);
        add_response_to_buffer(VAR_TEST_VAR_U16, &value, 2);
    }
    break;

    case VAR_TEST_VAR_U8:
    {
        // 测试变量 u8：+1 回传
        uint8_t value = (uint8_t)(v[0] + 1);
        add_response_to_buffer(VAR_TEST_VAR_U8, &value, 1);
    }
    break;

    case VAR_TURRET_ANGLE_YAW:
    {
        // 云台/炮塔偏航角 (4字节 float)
        float value = 0.0f;
        if (data_read_f32le(v, 4, &value) == DATA_OK)
        {
            set_launch_angle(value);
        }
    }
    break;

    /* ===== default ===== */
    default:
    {
        // 未知变量：回传错误码，值为未知的 t
        uint8_t value = t;
        add_response_to_buffer(VAR_ERROR, &value, 1);
    }
    break;
    }

    // 统一回发VAR_OK，值为接收到的变量ID
    add_response_to_buffer(VAR_OK, &t, 1);

    return 0; // 继续解析
}

static parser_status_t parse_complete_frame(const uint8_t *frame, size_t frame_len, parsed_message_t *msg)
{
    // 初始化消息结构
    memset(msg, 0, sizeof(parsed_message_t));

    // 重置全局响应缓存
    memset(&g_response_buffer, 0, sizeof(g_response_buffer));

    // 解析帧
    uint8_t frame_data[PCMCU_MAX_DATA_LEN] = {0};
    size_t frame_data_size = 0;
    pcmcu_parse_frame_data(frame, frame_len,
                           frame_data, PCMCU_MAX_DATA_LEN, &frame_data_size,
                           &msg->version, &msg->sequence);

    // 解析TLV变量
    uint8_t decoded_msg_type = 0;
    uint8_t decoded_version = 0;

    int decode_result = data_decode(frame_data, frame_data_size,
                                    &decoded_msg_type, &decoded_version,
                                    on_tlv_callback, msg);

    if (decode_result < 0)
    {
        return PARSER_ERROR_DATA_ERROR;
    }

    // 设置消息类型
    msg->msg_type = decoded_msg_type;

    // 验证版本一致性
    if (decoded_version != PROTOCOL_DATA_VER)
    {
        return PARSER_ERROR_DATA_ERROR;
    }

    // 一次性发送所有响应数据
    if (g_response_buffer.item_count > 0)
    {
        sender_send_multiple_tlv(g_response_buffer.items, g_response_buffer.item_count);
    }

    return PARSER_OK;
}

/** 帧回调函数，用于处理解析出的完整帧 */
static int on_frame_callback(const uint8_t *frame, size_t frame_len, void *user)
{
    data_parser_t *parser = (data_parser_t *)user;
    if (!parser || !frame)
        return -1;

    // 解析完整帧
    parser_status_t status = parse_complete_frame(frame, frame_len, &parser->current_msg);
    switch (status)
    {
    case PARSER_OK:
        // 解析成功，当前消息已更新
        break;
    default:
        // 其他错误
        break;
    }

    return 0; // 继续解析
}

// ================= 基础管理函数实现 =================

parser_status_t parser_init(void)
{
    // 初始化流式解析器
    pcmcu_stream_init(&_parser.stream);

    // 清空当前消息
    memset(&_parser.current_msg, 0, sizeof(_parser.current_msg));

    return PARSER_OK;
}

void parser_clear(void)
{
    pcmcu_stream_clear(&_parser.stream);
    memset(&_parser.current_msg, 0, sizeof(_parser.current_msg));
}

// ================= 字节流输入和解析实现 =================

parser_status_t parser_feed_byte(uint8_t byte)
{
    return parser_feed_stream(&byte, 1);
}

parser_status_t parser_feed_stream(const uint8_t *data, size_t len)
{
    if (!data && len > 0)
    {
        return PARSER_ERROR_INVALID_PARAM;
    }

    if (len == 0)
    {
        return PARSER_OK;
    }

    // 使用流式解析器处理数据
    int result = pcmcu_stream_feed(&_parser.stream, data, len, on_frame_callback, &_parser);

    return result;
}