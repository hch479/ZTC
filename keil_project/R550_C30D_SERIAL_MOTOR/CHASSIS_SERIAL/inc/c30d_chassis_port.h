#ifndef C30D_CHASSIS_PORT_H
#define C30D_CHASSIS_PORT_H

#include <stdint.h>

#include "chassis_can_protocol.h"
#include "chassis_params.h"

/*
 * C30D 原工程中，R550/差速/阿克曼模式的前两个编码器使用 TIM2、TIM3，
 * 右侧编码器在原代码中需要取反。若你的实车方向不同，只改这里即可。
 */
#define C30D_LEFT_ENCODER_TIMER  2U
#define C30D_RIGHT_ENCODER_TIMER 3U
#define C30D_LEFT_ENCODER_SIGN   1
#define C30D_RIGHT_ENCODER_SIGN  (-1)

/* PWM 符号也集中配置，禁止在控制算法内部到处加负号。 */
#define C30D_LEFT_PWM_SIGN  1
#define C30D_RIGHT_PWM_SIGN 1

uint32_t c30d_port_get_time_ms(void);
int16_t c30d_port_read_left_encoder(void);
int16_t c30d_port_read_right_encoder(void);
uint16_t c30d_port_read_battery_mv(void);
uint8_t c30d_port_hardware_is_enabled(void);

void c30d_port_set_output(int16_t left_pwm,
                          int16_t right_pwm,
                          uint16_t servo_pwm);

/*
 * 传输接口。串口版从 USART3 接收队列读帧，并通过 USART3 发送。
 * 上层仍使用统一的 logical_id + 8 字节数据结构，因此控制算法无需关心物理总线。
 */
int c30d_port_transport_receive(chassis_can_frame_t *frame);
int c30d_port_transport_send(const chassis_can_frame_t *frame);

/* 返回 1=有效，0=从未保存，-1=已有内容但校验失败。 */
int c30d_port_load_parameters(chassis_params_t *params);
int c30d_port_save_parameters(const chassis_params_t *params);

#endif
