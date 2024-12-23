#include "cmsis_os2.h"
#include <stdint.h>

typedef struct {
    void (*taskFunction)();
    uint32_t delay;
    osThreadAttr_t attrs;
} ThreadWrapper;

void rtosWrapper(void *);

// cursed macro
// TODO calculate stack size
#define defineThread(TASK, DELAY, PRIORITY)\
    ThreadWrapper wrapper_##TASK = { \
        .taskFunction = &(TASK),     \
        .delay = (DELAY),            \
        .attrs = {                   \
            .priority = (PRIORITY),  \
            .stack_size = 1024,      \
            .name = "\""#TASK"\"",   \
        }                            \
    };

#define createThread(NAME, DELAY, PRIORITY)\
    osThreadId_t handle_##NAME = osThreadNew(rtosWrapper, &(wrapper_##NAME), &(wrapper_##NAME).attrs);
