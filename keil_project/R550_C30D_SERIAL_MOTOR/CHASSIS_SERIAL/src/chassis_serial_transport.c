#include "chassis_serial_transport.h"

#include "stm32f4xx_usart.h"
#include "usartx.h"

#include <string.h>

/*
 * 中断函数只完成两件事：逐字节组帧、把完整帧放入小队列。
 * 协议解析和电机控制都留在 FreeRTOS 任务中，避免中断执行时间过长。
 */
static chassis_can_frame_t g_receive_queue[CHASSIS_SERIAL_QUEUE_LENGTH];
static volatile uint8_t g_queue_write = 0U;
static volatile uint8_t g_queue_read = 0U;
static volatile uint16_t g_queue_overrun = 0U;

static uint8_t g_parser_buffer[CHASSIS_SERIAL_FRAME_LENGTH];
static uint8_t g_parser_count = 0U;

static void parser_reset_with_possible_header(uint8_t byte)
{
    if (byte == CHASSIS_SERIAL_SOF0)
    {
        g_parser_buffer[0] = byte;
        g_parser_count = 1U;
    }
    else
    {
        g_parser_count = 0U;
    }
}

static void queue_complete_frame(void)
{
    uint8_t next_write;
    chassis_can_frame_t *destination;

    next_write = (uint8_t)((g_queue_write + 1U) % CHASSIS_SERIAL_QUEUE_LENGTH);
    if (next_write == g_queue_read)
    {
        /* 队列满时丢弃最新帧。已有命令仍会按顺序被任务处理。 */
        if (g_queue_overrun < 65535U)
        {
            g_queue_overrun++;
        }
        return;
    }

    destination = &g_receive_queue[g_queue_write];
    destination->id = (uint32_t)g_parser_buffer[3] |
                      ((uint32_t)g_parser_buffer[4] << 8U);
    destination->length = CHASSIS_CAN_DLC;
    memcpy(destination->data, &g_parser_buffer[6], CHASSIS_CAN_DLC);

    /* 数据写完后再移动写指针，任务不会读到只写了一半的帧。 */
    __DMB();
    g_queue_write = next_write;
}

static void parser_accept_byte(uint8_t byte)
{
    uint8_t calculated_crc;

    if (g_parser_count == 0U)
    {
        parser_reset_with_possible_header(byte);
        return;
    }

    if (g_parser_count == 1U)
    {
        if (byte == CHASSIS_SERIAL_SOF1)
        {
            g_parser_buffer[1] = byte;
            g_parser_count = 2U;
        }
        else
        {
            parser_reset_with_possible_header(byte);
        }
        return;
    }

    g_parser_buffer[g_parser_count] = byte;
    g_parser_count++;

    /* 尽早拒绝错误的版本或长度，随后重新寻找帧头。 */
    if ((g_parser_count == 3U) &&
        (g_parser_buffer[2] != CHASSIS_SERIAL_VERSION))
    {
        parser_reset_with_possible_header(byte);
        return;
    }
    if ((g_parser_count == 6U) &&
        (g_parser_buffer[5] != CHASSIS_CAN_DLC))
    {
        parser_reset_with_possible_header(byte);
        return;
    }

    if (g_parser_count == CHASSIS_SERIAL_FRAME_LENGTH)
    {
        calculated_crc = chassis_crc8(&g_parser_buffer[2], 12U);
        if (calculated_crc == g_parser_buffer[14])
        {
            queue_complete_frame();
        }
        parser_reset_with_possible_header(byte);
    }
}

void CHASSIS_USART3_IRQHandler(void)
{
    uint8_t received_byte;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        /* 读取 DR 会清除 RXNE 标志。 */
        received_byte = (uint8_t)USART_ReceiveData(USART3);
        parser_accept_byte(received_byte);
    }

    /* 发生溢出时必须读 SR/DR 清标志，否则中断可能反复进入。 */
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        volatile uint16_t status_value = USART3->SR;
        volatile uint16_t data_value = USART3->DR;
        (void)status_value;
        (void)data_value;
    }
}

int chassis_serial_receive(chassis_can_frame_t *frame)
{
    if ((frame == 0) || (g_queue_read == g_queue_write))
    {
        return 0;
    }

    *frame = g_receive_queue[g_queue_read];
    __DMB();
    g_queue_read = (uint8_t)((g_queue_read + 1U) % CHASSIS_SERIAL_QUEUE_LENGTH);
    return 1;
}

int chassis_serial_send(const chassis_can_frame_t *frame)
{
    uint8_t output[CHASSIS_SERIAL_FRAME_LENGTH];
    uint8_t index;

    if ((frame == 0) || (frame->length != CHASSIS_CAN_DLC) ||
        (frame->id > 0x7FFU))
    {
        return 0;
    }

    output[0] = CHASSIS_SERIAL_SOF0;
    output[1] = CHASSIS_SERIAL_SOF1;
    output[2] = CHASSIS_SERIAL_VERSION;
    output[3] = (uint8_t)(frame->id & 0xFFU);
    output[4] = (uint8_t)((frame->id >> 8U) & 0xFFU);
    output[5] = CHASSIS_CAN_DLC;
    memcpy(&output[6], frame->data, CHASSIS_CAN_DLC);
    output[14] = chassis_crc8(&output[2], 12U);

    for (index = 0U; index < CHASSIS_SERIAL_FRAME_LENGTH; ++index)
    {
        usart3_send(output[index]);
    }
    return 1;
}

uint16_t chassis_serial_get_overrun_count(void)
{
    return g_queue_overrun;
}
