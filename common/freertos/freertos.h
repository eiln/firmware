#ifndef __COMMON_FREERTOS_H__
#define __COMMON_FREERTOS_H__

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "atomic.h"
#include "list.h"
#include "message_buffer.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"
#include "timers.h"

#include <stdint.h>

typedef struct {
    void (*taskFunction)();
    uint32_t delay;
    osThreadAttr_t attrs;
    osThreadId_t handle;
} ThreadWrapper;

void rtosWrapper(void *);

// cursed macro
// TODO calculate stack size
#define threadWrapperName(NAME) (threadWrapper_##NAME)
#define __defineThread(TASK, DELAY, PRIORITY, STACK)\
    ThreadWrapper threadWrapperName(TASK) = { \
        .taskFunction = &(TASK),     \
        .delay = (DELAY),            \
        .attrs = {                   \
            .priority = (PRIORITY),  \
            .stack_size = (STACK),   \
            .name = "\""#TASK"\"",   \
        }                            \
    };                               \

#define defineThread(T, D, P) __defineThread(T, D, P, 1024)
#define defineThreadStack(T, D, P, S) __defineThread(T, D, P, S)

#define createThread(NAME, DELAY, PRIORITY)\
    threadWrapperName(NAME).handle = osThreadNew(rtosWrapper, &(threadWrapperName(NAME)), &(threadWrapperName(NAME)).attrs);

#define createThreadStack(NAME, DELAY, PRIORITY, STACK) createThread(NAME, DELAY, PRIORITY)
#define createThreadV2(NAME) createThread(NAME, NULL, NULL)

#define defineStaticQueue(NAME, ITEM, COUNT)\
    QueueHandle_t NAME;\
    static StaticQueue_t xStaticQueue_##NAME;\
    uint8_t ucQueueStorageArea_##NAME[sizeof(ITEM) * (COUNT)];

#define createStaticQueue(NAME, ITEM, COUNT)\
    xQueueCreateStatic((COUNT),\
                       sizeof(ITEM),\
                       ucQueueStorageArea_##NAME,\
                       &xStaticQueue_##NAME);

#define getTaskHandle(NAME) threadWrapperName(NAME).handle

#define getTick() xTaskGetTickCount()
#define getTickms() pdMS_TO_TICKS(getTick())

#define mDelay(ms) (osDelay(pdMS_TO_TICKS((ms))))

#endif // __COMMON_FREERTOS_H__

