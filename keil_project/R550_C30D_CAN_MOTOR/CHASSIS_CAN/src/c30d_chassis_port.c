/*
 * 本文件是控制核心与 WHEELTEC C30D 原工程之间唯一的硬件接口层。
 * 它会调用原工程已经验证过的编码器、PWM、CAN、电压和 Flash 函数。
 *
 * 将本文件加入 Keil 工程时，include path 中需要已有：
 * BALANCE、HARDWARE、SYSTEM、FreeRTOS/Source/include。
 */
#include "c30d_chassis_port.h"

#include "balance.h"
#include "can.h"
#include "encoder.h"
#include "motor.h"
#include "stmflash.h"
#include "system.h"

#include <string.h>

uint32_t c30d_port_get_time_ms(void)
{
    /* FreeRTOS tick 在该资料工程中通常为 1 ms；转换宏兼容其他 tick 频率。 */
    const TickType_t ticks = xTaskGetTickCount();
    return (uint32_t)(ticks * portTICK_PERIOD_MS);
}

int16_t c30d_port_read_left_encoder(void)
{
    int count = Read_Encoder(C30D_LEFT_ENCODER_TIMER);
    count *= C30D_LEFT_ENCODER_SIGN;
    return (int16_t)count;
}

int16_t c30d_port_read_right_encoder(void)
{
    int count = Read_Encoder(C30D_RIGHT_ENCODER_TIMER);
    count *= C30D_RIGHT_ENCODER_SIGN;
    return (int16_t)count;
}

uint16_t c30d_port_read_battery_mv(void)
{
    float millivolts = Voltage * 1000.0f;
    if (millivolts <= 0.0f)
    {
        return 0U;
    }
    if (millivolts >= 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)millivolts;
}

uint8_t c30d_port_hardware_is_enabled(void)
{
    return (uint8_t)(EN != 0U);
}

void c30d_port_set_output(int16_t left_pwm,
                          int16_t right_pwm,
                          uint16_t servo_pwm)
{
    int left = (int)left_pwm * C30D_LEFT_PWM_SIGN;
    int right = (int)right_pwm * C30D_RIGHT_PWM_SIGN;

    /* R550 使用 A、B 两路电机；C、D 保持为零。 */
    Set_Pwm(left, right, 0, 0, (int)servo_pwm);
}

int c30d_port_can_receive(chassis_can_frame_t *frame)
{
    u32 id = 0U;
    u8 ide = 0U;
    u8 rtr = 0U;
    u8 length = 0U;

    if ((frame == 0) || (CAN1_Msg_Pend(0U) == 0U))
    {
        return 0;
    }

    CAN1_Rx_Msg(0U, &id, &ide, &rtr, &length, frame->data);
    if ((ide != 0U) || (rtr != 0U) || (length != CHASSIS_CAN_DLC))
    {
        return 0;
    }
    frame->id = id;
    frame->length = length;
    return 1;
}

int c30d_port_can_send(const chassis_can_frame_t *frame)
{
    u8 mailbox;
    uint32_t start_ms;

    if ((frame == 0) || (frame->length > CHASSIS_CAN_DLC))
    {
        return 0;
    }

    mailbox = CAN1_Tx_Msg(frame->id, 0U, 0U, frame->length,
                          (u8 *)frame->data);
    if (mailbox == 0xFFU)
    {
        return 0;
    }

    /* 等待当前邮箱完成，确保连续发送遥测帧时不会耗尽三个邮箱。 */
    start_ms = c30d_port_get_time_ms();
    while (CAN1_Tx_Staus(mailbox) == 0U)
    {
        if ((c30d_port_get_time_ms() - start_ms) > 3U)
        {
            return 0;
        }
        taskYIELD();
    }
    return (CAN1_Tx_Staus(mailbox) == 0x07U) ? 1 : 0;
}

int c30d_port_load_parameters(chassis_params_t *params)
{
    uint32_t word;
    uint32_t first_word;
    uint16_t index;
    const uint16_t word_count = (uint16_t)(sizeof(*params) / sizeof(uint32_t));

    if ((params == 0) || ((sizeof(*params) % sizeof(uint32_t)) != 0U))
    {
        return 0;
    }

    for (index = 0U; index < word_count; ++index)
    {
        word = (uint32_t)Read_Flash(index);
        memcpy(&((uint8_t *)params)[(uint32_t)index * sizeof(uint32_t)],
               &word,
               sizeof(word));
    }

    memcpy(&first_word, params, sizeof(first_word));
    if ((first_word == 0xFFFFFFFFUL) ||
        (first_word != CHASSIS_PARAMETER_MAGIC))
    {
        /* Flash 为空或仍是原厂四参数格式，都按“尚未保存新参数”处理。 */
        return 0;
    }
    if (chassis_params_validate(params) && chassis_params_crc_is_valid(params))
    {
        return 1;
    }
    return -1; /* Flash 有内容，但格式、范围或 CRC 不正确。 */
}

int c30d_port_save_parameters(const chassis_params_t *params)
{
    chassis_params_t copy;
    uint32_t words[sizeof(chassis_params_t) / sizeof(uint32_t)];
    uint8_t flash_result;
    const uint16_t word_count = (uint16_t)(sizeof(copy) / sizeof(uint32_t));

    if ((params == 0) || ((sizeof(copy) % sizeof(uint32_t)) != 0U))
    {
        return 0;
    }

    copy = *params;
    chassis_params_update_crc(&copy);
    if (!chassis_params_validate(&copy))
    {
        return 0;
    }
    memcpy(words, &copy, sizeof(copy));

    /* 擦写内部 Flash 时先关闭电机，并禁止任务切换，避免读取半写入的数据。 */
    c30d_port_set_output(0, 0, (uint16_t)(copy.servo_center_pwm + 0.5f));
    taskENTER_CRITICAL();
    flash_result = Write_Flash((u32 *)words, word_count);
    taskEXIT_CRITICAL();
    return (flash_result == 0U) ? 1 : 0;
}
