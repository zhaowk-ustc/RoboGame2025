
#include "parser.h"
#include "protocol_c/frame.h"
#include "protocol_c/data.h"
#include "protocol_c/protocol_defs.h"
#include "sender.h"
#include <string.h>

static data_parser_t _parser = {
    .stream = {{0}},              // 初始化流式解析器
    .current_msg = {{0}},         // 清空当前消息
};

// ================= 内部静态变量和函数声明 =================

/** TLV解析回调函数 */
static int on_tlv_callback(uint8_t t, const uint8_t* v, uint8_t l, void* user)
{
    parsed_message_t* msg = (parsed_message_t*)user;
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
        case VAR_FRICTION_WHEEL_SPEED:
            // 摩擦轮速度 (4字节)
            break;
        case VAR_DART_BACKWARD:
            // 飞镖后退 (1字节)
            break;
        case VAR_GRIPPER_RELEASE:
            // 夹爪释放 (1字节)
            break;
        case VAR_GRIPPER_TAG_Y:
            // 夹爪标签Y坐标 (4字节)
            break;
        case VAR_BASE_MOVE_FORWARD:
            // 底盘前进 (4字节)
            break;
        case VAR_TEST_VAR_F32:
            // 测试变量浮点数 (4字节) - 把值加1再发回
            if (l == 4)
            {
                float value;
                if (data_read_f32le(v, l, &value) == DATA_OK)
                {
                    sender_send_var_f32(VAR_TEST_VAR_F32, value + 0.1f);
                }
            }
            break;
        case VAR_TEST_VAR_U8:
            // 测试变量8位整数 (1字节) - 把值加1再发回
            if (l == 1)
            {
                uint8_t value = v[0];
                sender_send_var_u8(VAR_TEST_VAR_U8, value + 1);
            }
            break;
        case VAR_GRIPPER_TAG_X:
            // 夹爪标签X坐标 (4字节)
            break;
        case VAR_DART_LAUNCH:
            // 飞镖发射 (1字节)
            break;
        case VAR_FRICTION_WHEEL_STOP:
            // 摩擦轮停止 (1字节)
            break;
        case VAR_GRIPPER_TAG_Z:
            // 夹爪标签Z坐标 (4字节)
            break;
        case VAR_BASE_MOVE_LEFT:
            // 底盘左移 (4字节)
            break;
        case VAR_FRICTION_WHEEL_START:
            // 摩擦轮启动 (1字节)
            break;
        case VAR_TEST_VAR_U16:
            // 测试变量16位整数 (2字节) - 把值加1再发回
            if (l == 2)
            {
                uint16_t value = v[0] | (v[1] << 8); // 小端序读取
                sender_send_var_u16(VAR_TEST_VAR_U16, value + 10);
            }
            break;
        case VAR_BASE_ROTATE_YAW:
            // 底盘偏航旋转 (4字节)
            break;
        case VAR_GRIPPER_GRASP:
            // 夹爪抓取 (1字节)
            break;
        default:
            // 未知变量类型
            sender_send_var_u8(VAR_DATA_ERROR, t);
            break;
    }

    return 0; // 继续解析
}

static parser_status_t parse_complete_frame(const uint8_t* frame, size_t frame_len, parsed_message_t* msg)
{
    // 初始化消息结构
    memset(msg, 0, sizeof(parsed_message_t));

    // 解析帧
    uint8_t frame_data[PCMCU_MAX_DATA_LEN] = { 0 };
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
    if (decoded_version != msg->version)
    {
        return PARSER_ERROR_DATA_ERROR;
    }

    return PARSER_OK;
}

/** 帧回调函数，用于处理解析出的完整帧 */
static int on_frame_callback(const uint8_t* frame, size_t frame_len, void* user)
{
    data_parser_t* parser = (data_parser_t*)user;
    if (!parser || !frame) return -1;

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

parser_status_t parser_feed_stream(const uint8_t* data, size_t len)
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