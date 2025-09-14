#ifndef SENDER_H
#define SENDER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol_c/frame.h"
#include "protocol_c/data.h"
#include "protocol_c/protocol_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================= 发送器状态和结构体定义 =================

/** 发送状态枚举 */
typedef enum {
    SENDER_OK = 0,                  /**< 发送成功 */
    SENDER_ERROR_INVALID_PARAM,     /**< 无效参数 */
    SENDER_ERROR_BUFFER_FULL,       /**< 缓冲区满 */
    SENDER_ERROR_ENCODE_ERROR,      /**< 编码错误 */
    SENDER_ERROR_FRAME_ERROR,       /**< 帧错误 */
    SENDER_ERROR_SEND_FAILED,       /**< 发送失败 */
} sender_status_t;

/** 发送回调函数类型定义 */
typedef sender_status_t (*sender_callback_t)(const uint8_t* data, size_t len, void* user_data);

/** 发送器结构体 */
typedef struct {
    uint8_t send_buffer[PCMCU_MAX_DATA_LEN];  /**< 发送缓冲区 */
    uint8_t sequence;                          /**< 当前序列号 */
    sender_callback_t send_callback;           /**< 发送回调函数 */
    void* user_data;                           /**< 用户数据指针 */
} data_sender_t;

// ================= 发送器初始化和管理 =================

/**
 * @brief 初始化数据发送器（单例模式）
 * @param callback 发送回调函数
 * @param user_data 用户数据指针
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_init(sender_callback_t callback, void* user_data);

/**
 * @brief 清空发送器状态
 */
void sender_clear(void);

// ================= 快速TLV发送函数 =================

/**
 * @brief 发送单个uint8类型的TLV变量
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_u8(uint8_t var_id, uint8_t value);

/**
 * @brief 发送单个uint16类型的TLV变量
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_u16(uint8_t var_id, uint16_t value);

/**
 * @brief 发送单个uint32类型的TLV变量
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_u32(uint8_t var_id, uint32_t value);

/**
 * @brief 发送单个float32类型的TLV变量
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_f32(uint8_t var_id, float value);

/**
 * @brief 发送自定义长度的TLV变量
 * @param var_id 变量ID
 * @param data 数据指针
 * @param len 数据长度
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_tlv(uint8_t var_id, const void* data, uint8_t len);

// ================= 批量TLV发送函数 =================

/**
 * @brief TLV数据结构，用于批量发送
 */
typedef struct {
    uint8_t var_id;         /**< 变量ID */
    const void* data;       /**< 数据指针 */
    uint8_t len;            /**< 数据长度 */
} tlv_item_t;

/**
 * @brief 批量发送多个TLV变量
 * @param items TLV项数组
 * @param item_count TLV项数量
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_multiple_tlv(const tlv_item_t* items, size_t item_count);

// ================= 便捷发送函数（使用变量ID验证） =================

/**
 * @brief 基于变量定义发送uint8变量（会验证变量长度）
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_var_u8(uint8_t var_id, uint8_t value);

/**
 * @brief 基于变量定义发送uint16变量（会验证变量长度）
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_var_u16(uint8_t var_id, uint16_t value);

/**
 * @brief 基于变量定义发送uint32变量（会验证变量长度）
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_var_u32(uint8_t var_id, uint32_t value);

/**
 * @brief 基于变量定义发送float32变量（会验证变量长度）
 * @param var_id 变量ID
 * @param value 变量值
 * @return SENDER_OK 成功，其他值表示错误
 */
sender_status_t sender_send_var_f32(uint8_t var_id, float value);

// ================= 工具函数 =================

/**
 * @brief 获取当前序列号
 * @return 当前序列号
 */
uint8_t sender_get_sequence(void);

/**
 * @brief 设置序列号
 * @param sequence 序列号
 */
void sender_set_sequence(uint8_t sequence);

/**
 * @brief 验证变量ID是否有效
 * @param var_id 变量ID
 * @return true 有效，false 无效
 */
bool sender_is_valid_var_id(uint8_t var_id);

#ifdef __cplusplus
}
#endif

#endif /* SENDER_H */
