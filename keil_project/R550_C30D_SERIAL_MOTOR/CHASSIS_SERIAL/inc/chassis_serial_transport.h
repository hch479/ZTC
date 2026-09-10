#ifndef CHASSIS_SERIAL_TRANSPORT_H
#define CHASSIS_SERIAL_TRANSPORT_H

#include <stdint.h>

#include "chassis_can_protocol.h"

/*
 * C30D 与 RDK X5 之间的串口外层帧。
 *
 * 0      0xA5              帧头 1
 * 1      0x5A              帧头 2
 * 2      0x01              串口协议版本
 * 3..4   logical_id        原 CAN 标准帧 ID，小端序
 * 5      0x08              数据长度
 * 6..13  data[8]           原协议的 8 字节数据
 * 14     crc8              对字节 2..13 做 CRC-8/ATM
 *
 * 保留 logical_id 和 data[8] 的布局，可以让电机控制、参数、遥测协议
 * 同时用于串口版和 CAN 版；只有最外层的传输方式不同。
 */
#define CHASSIS_SERIAL_SOF0          0xA5U
#define CHASSIS_SERIAL_SOF1          0x5AU
#define CHASSIS_SERIAL_VERSION       0x01U
#define CHASSIS_SERIAL_FRAME_LENGTH  15U
#define CHASSIS_SERIAL_QUEUE_LENGTH  8U

/* USART3 接收中断入口。启动文件的向量表会直接调用它。 */
void CHASSIS_USART3_IRQHandler(void);

/* 非阻塞读取一帧。收到合法帧返回 1，没有完整帧返回 0。 */
int chassis_serial_receive(chassis_can_frame_t *frame);

/* 发送一帧；输入不合法返回 0，发送成功返回 1。 */
int chassis_serial_send(const chassis_can_frame_t *frame);

/* 接收速度过快导致队列满时的累计丢帧数，供诊断使用。 */
uint16_t chassis_serial_get_overrun_count(void);

#endif
