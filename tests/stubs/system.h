#ifndef TEST_STUB_SYSTEM_H
#define TEST_STUB_SYSTEM_H

#include <stdint.h>
#include <stddef.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef void *SemaphoreHandle_t;

#define pdTRUE 1
#define pdPASS 1
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(value) ((TickType_t)(value))
#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL() do { } while (0)
#define taskYIELD() do { } while (0)

extern float Voltage;
extern int SysVal;

TickType_t xTaskGetTickCount(void);
BaseType_t xTaskCreate(void (*task)(void *), const char *name,
                       uint16_t stack, void *argument,
                       unsigned int priority, void *handle);
void vTaskDelayUntil(TickType_t *last, TickType_t ticks);

#endif
