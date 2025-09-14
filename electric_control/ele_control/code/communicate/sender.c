#include "sender.h"
#include "protocol_c/frame.h"
#include "protocol_c/data.h"
#include "protocol_c/protocol_defs.h"
#include <string.h>

// ================= 单例实例 =================

static data_sender_t _sender = {
    .send_buffer = {0},           // 初始化发送缓冲区
    .sequence = 0,                // 初始化序列号
    .send_callback = NULL,        // 初始化回调函数
    .user_data = NULL,            // 初始化用户数据
};

// ================= 发送器初始化和管理实现 =================

sender_status_t sender_init(sender_callback_t callback, void* user_data)
{
    if (!callback) {
        return SENDER_ERROR_INVALID_PARAM;
    }

    // 清空发送缓冲区
    memset(_sender.send_buffer, 0, sizeof(_sender.send_buffer));
    
    // 初始化序列号
    _sender.sequence = 0;
    
    // 设置回调函数
    _sender.send_callback = callback;
    _sender.user_data = user_data;

    return SENDER_OK;
}

void sender_clear(void)
{
    memset(_sender.send_buffer, 0, sizeof(_sender.send_buffer));
    _sender.sequence = 0;
    _sender.send_callback = NULL;
    _sender.user_data = NULL;
}

// ================= 内部辅助函数 =================

/**
 * @brief 发送数据的内部实现函数
 */
static sender_status_t send_data_internal(const kv_t* kvs, size_t kv_count)
{
    if (!_sender.send_callback || !kvs) {
        return SENDER_ERROR_INVALID_PARAM;
    }

    // 编码数据，使用MSG_MCU_TO_PC作为消息类型
    size_t encoded_len = 0;
    int encode_result = data_kv_encode(MSG_MCU_TO_PC, PROTOCOL_DATA_VER, kvs, kv_count,
                                     _sender.send_buffer, sizeof(_sender.send_buffer), &encoded_len);

    if (encode_result < 0) {
        return SENDER_ERROR_ENCODE_ERROR;
    }

    // 创建帧
    uint8_t frame_buffer[PCMCU_MAX_DATA_LEN];
    size_t frame_len = 0;
    
    int frame_result = pcmcu_build_frame(_sender.sequence,
                                       _sender.send_buffer, encoded_len,
                                       PROTOCOL_DATA_VER,
                                       frame_buffer, sizeof(frame_buffer), &frame_len);

    if (frame_result < 0) {
        return SENDER_ERROR_FRAME_ERROR;
    }

    // 增加序列号
    _sender.sequence++;

    // 通过回调发送数据
    return _sender.send_callback(frame_buffer, frame_len, _sender.user_data);
}

// ================= 工具函数实现 =================

uint8_t sender_get_sequence(void)
{
    return _sender.sequence;
}

void sender_set_sequence(uint8_t sequence)
{
    _sender.sequence = sequence;
}

bool sender_is_valid_var_id(uint8_t var_id)
{
    // 直接读取变量长度表来判断变量ID是否有效
    // 如果变量长度不为0，说明该变量ID已定义
    return VAR_SIZE_TABLE[var_id] != 0;
}

// ================= 快速TLV发送函数实现 =================

sender_status_t sender_send_u8(uint8_t var_id, uint8_t value)
{
    kv_t kv = {
        .t = var_id,
        .v = (const uint8_t*)&value,
        .l = 1
    };
    
    return send_data_internal(&kv, 1);
}

sender_status_t sender_send_u16(uint8_t var_id, uint16_t value)
{
    // 转换为小端序
    uint8_t data[2];
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    
    kv_t kv = {
        .t = var_id,
        .v = data,
        .l = 2
    };
    
    return send_data_internal(&kv, 1);
}

sender_status_t sender_send_u32(uint8_t var_id, uint32_t value)
{
    // 转换为小端序
    uint8_t data[4];
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 24) & 0xFF;
    
    kv_t kv = {
        .t = var_id,
        .v = data,
        .l = 4
    };
    
    return send_data_internal(&kv, 1);
}

sender_status_t sender_send_f32(uint8_t var_id, float value)
{
    // 使用联合体进行浮点数到字节的转换
    union {
        float f;
        uint8_t bytes[4];
    } converter;
    
    converter.f = value;
    
    kv_t kv = {
        .t = var_id,
        .v = converter.bytes,
        .l = 4
    };
    
    return send_data_internal(&kv, 1);
}

sender_status_t sender_send_tlv(uint8_t var_id, const void* data, uint8_t len)
{
    if (!data && len > 0) {
        return SENDER_ERROR_INVALID_PARAM;
    }
    
    kv_t kv = {
        .t = var_id,
        .v = (const uint8_t*)data,
        .l = len
    };
    
    return send_data_internal(&kv, 1);
}

// ================= 批量TLV发送函数实现 =================

sender_status_t sender_send_multiple_tlv(const tlv_item_t* items, size_t item_count)
{
    if (!items || item_count == 0) {
        return SENDER_ERROR_INVALID_PARAM;
    }
    
    // 转换为kv_t数组
    kv_t kvs[16]; // 限制最大16个变量
    if (item_count > 16) {
        return SENDER_ERROR_INVALID_PARAM;
    }
    
    for (size_t i = 0; i < item_count; i++) {
        kvs[i].t = items[i].var_id;
        kvs[i].v = (const uint8_t*)items[i].data;
        kvs[i].l = items[i].len;
    }
    
    return send_data_internal(kvs, item_count);
}

// ================= 便捷发送函数（使用变量ID验证）实现 =================

sender_status_t sender_send_var_u8(uint8_t var_id, uint8_t value)
{
    // 验证变量长度
    if (VAR_SIZE_TABLE[var_id] != 1) {
        return SENDER_ERROR_INVALID_PARAM;
    }
    
    return sender_send_u8(var_id, value);
}

sender_status_t sender_send_var_u16(uint8_t var_id, uint16_t value)
{
    // 验证变量长度
    if (VAR_SIZE_TABLE[var_id] != 2) {
        return SENDER_ERROR_INVALID_PARAM;
    }
    
    return sender_send_u16(var_id, value);
}

sender_status_t sender_send_var_u32(uint8_t var_id, uint32_t value)
{
    // 验证变量长度
    if (VAR_SIZE_TABLE[var_id] != 4) {
        return SENDER_ERROR_INVALID_PARAM;
    }
    
    return sender_send_u32(var_id, value);
}

sender_status_t sender_send_var_f32(uint8_t var_id, float value)
{
    // 验证变量长度
    if (VAR_SIZE_TABLE[var_id] != 4) {
        return SENDER_ERROR_INVALID_PARAM;
    }
    
    return sender_send_f32(var_id, value);
}
