#ifndef TEST_STUB_STMFLASH_H
#define TEST_STUB_STMFLASH_H
#include "system.h"
uint8_t Write_Flash(uint32_t *data, uint16_t length);
int Read_Flash(uint16_t index);
#endif

