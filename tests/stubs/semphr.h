#ifndef TEST_STUB_SEMPHR_H
#define TEST_STUB_SEMPHR_H
#include "system.h"
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore);
#endif

