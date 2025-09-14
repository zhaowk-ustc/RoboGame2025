
#ifndef PARSER_H
#define PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol_c/frame.h"
#include "protocol_c/data.h"
#include "protocol_c/protocol_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

    // ================= 解析器状态和结构体定义 =================

    /** 解析状态枚举 */
    typedef enum
    {
        PARSER_OK = 0,                  /**< 解析成功 */
        PARSER_ERROR_INVALID_PARAM,     /**< 无效参数 */
        PARSER_ERROR_BUFFER_FULL,       /**< 缓冲区满 */
        PARSER_ERROR_FRAME_ERROR,       /**< 帧错误 */
        PARSER_ERROR_DATA_ERROR,        /**< 数据错误 */
        PARSER_ERROR_CHECKSUM,          /**< 校验错误 */
    } parser_status_t;

    /** 解析出的变量数据结构 */
    typedef struct
    {
        uint8_t t;
        uint8_t l;
        const uint8_t* v;
    } tlv_t;

    /** 解析出的完整消息结构 */
    typedef struct
    {
        uint8_t msg_type;               /**< 消息类型 (MSG_PC_TO_MCU/MSG_MCU_TO_PC) */
        uint8_t version;                /**< 协议版本 */
        uint8_t sequence;               /**< 序列号 */
        tlv_t vars[16];                 /**< 解析出的变量数组 */
        uint8_t var_count;              /**< 变量数量 */
    } parsed_message_t;

    /** 流式解析器结构体 */
    typedef struct
    {
        pcmcu_stream_t stream;          /**< 底层流式解析器 */
        parsed_message_t current_msg;   /**< 当前解析的消息 */
    } data_parser_t;

    // ================= 解析器初始化和管理 =================

    /**
     * @brief 初始化数据解析器
     * @return PARSER_OK 成功，其他值表示错误
     */
    parser_status_t parser_init(void);

    /**
     * @brief 清空解析器状态
     */
    void parser_clear(void);

    // ================= 字节流输入和解析 =================

    /**
     * @brief 输入单个字节进行解析
     * @param parser 解析器指针
     * @param byte 输入字节
     * @return PARSER_OK 成功，其他值表示错误
     */
    parser_status_t parser_feed_byte(uint8_t byte);

    /**
     * @brief 输入字节流进行解析
     * @param parser 解析器指针
     * @param data 输入数据指针
     * @param len 数据长度
     * @return PARSER_OK 成功，其他值表示错误
     */
    parser_status_t parser_feed_stream(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* PARSER_H */
