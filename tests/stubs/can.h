#ifndef TEST_STUB_CAN_H
#define TEST_STUB_CAN_H
#include "system.h"
u8 CAN1_Tx_Msg(u32 id, u8 ide, u8 rtr, u8 length, u8 *data);
u8 CAN1_Tx_Staus(u8 mailbox);
u8 CAN1_Msg_Pend(u8 fifo);
void CAN1_Rx_Msg(u8 fifo, u32 *id, u8 *ide, u8 *rtr, u8 *length, u8 *data);
#endif

