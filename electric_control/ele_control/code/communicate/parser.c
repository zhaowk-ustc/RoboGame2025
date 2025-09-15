
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
    case VAR_FRICTION_WHEEL_SPEED: // 0x01
        // 摩擦轮速度 (4字节)
        {
            float value = 0;
            if (data_read_f32le(v, 4, &value) == DATA_OK)
            {
                // 设置摩擦轮速度为value
                bldc_set_speed(value); // value值在1000-2000
            }
        }
        break;
    case VAR_DART_BACKWARD: // 0x02
        // 飞镖后退 (1字节)
        {
            push_set_mode(MODE_RETURN);
        }
        break;
    case VAR_GRIPPER_RELEASE: // 0x04
        // 夹爪释放 (1字节)
        {
            pwm_set_duty(GRIPPER_PWM, GRIPPER_OPEN);
        }
        break;
    case VAR_GRIPPER_TAG_Y: // 0x15
        // 夹爪标签Y坐标 (4字节)
        break;
    case VAR_BASE_MOVE_FORWARD: // 0x34
        // 底盘前进 (4字节)
        {
            float value;
            data_read_f32le(v, 4, &value);
            set_motion(&PATTERN_FRONT, value);
            s
        }
        break;
    case VAR_TEST_VAR_F32: // 0x5D
                           // 测试变量浮点数 (4字节) - 把值加0.1再发回
    {
        float value;
        if (data_read_f32le(v, 4, &value) == DATA_OK)
        {
            value += 0.1f; // 加0.1
            add_response_to_buffer(VAR_TEST_VAR_F32, &value, 4);
        }
    }
    break;
    case VAR_TEST_VAR_U8: // 0x67
                          // 测试变量8位整数 (1字节) - 把值加1再发回
    {
        uint8_t value = v[0] + 1; // 加1
        add_response_to_buffer(VAR_TEST_VAR_U8, &value, 1);
    }
    break;
    case VAR_GRIPPER_TAG_X: // 0x69
        // 夹爪标签X坐标 (4字节)
        break;
    case VAR_DATA_ERROR: // 0x6A
        // 数据错误 (1字节)
        break;
    case VAR_BASE_STOP: // 0x7B
        // 底盘停止 (1字节)
        {
            float value;
            data_read_f32le(v, 4, &value);
            set_motion(&PATTERN_STOP, value);
        }
        break;
    case VAR_DART_LAUNCH: // 0x90
        // 飞镖发射 (1字节)
        {
            float value = 0;
            launch_servor_shot();
            push_set_mode(MODE_FORWARD);
            bldc_set_speed(value);
        }
        break;
    case VAR_FRICTION_WHEEL_STOP: // 0xA6
        // 摩擦轮停止 (1字节)
        {
            bldc_set_speed(1000);
        }
        break;
    case VAR_GRIPPER_TAG_Z: // 0xC4
        // 夹爪标签Z坐标 (4字节)
        break;
    case VAR_BASE_MOVE_LEFT: // 0xC6
        // 底盘左移 (4字节)
        {
            float value;
            data_read_f32le(v, 4, &value);
            set_motion(&PATTERN_LEFT, value);
        }
        break;
    case VAR_HEARTBEAT: // 0xD1
        // 心跳包 (1字节)
        break;
    case VAR_FRICTION_WHEEL_START: // 0xDE
        // 摩擦轮启动 (1字节)
        {
            bldc_init();
        }
        break;
    case VAR_TEST_VAR_U16: // 0xE6
                           // 测试变量16位整数 (2字节) - 把值加10再发回
    {
        uint16_t value = v[0] | (v[1] << 8); // 小端序读取
        value += 10;                         // 加10
        add_response_to_buffer(VAR_TEST_VAR_U16, &value, 2);
    }
    break;
    case VAR_BASE_ROTATE_YAW: // 0xE8
        // 底盘偏航旋转 (4字节)
        {
            float value;
            data_read_f32le(v, 4, &value);
            set_motion(&PATTERN_CW, value); // 顺时针转动
        }
        break;
    case VAR_GRIPPER_GRASP: // 0xEE
        // 夹爪抓取 (1字节)
        {
            pwm_set_duty(GRIPPER_PWM, GRIPPER_CLOSE);
        }
        break;
    default:
        // 未知变量类型 - 添加错误响应到缓存中
        {
            uint8_t value = t;
            add_response_to_buffer(VAR_DATA_ERROR, &value, 1);
        }
        break;
    }

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