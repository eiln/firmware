#include "freertos.h"

void rtosWrapper(void *thread)
{
    ThreadWrapper *wrapper = (ThreadWrapper *) thread;

    while(1)
    {
        wrapper->taskFunction();
        osDelay(wrapper->delay);
    }
}

osThreadId_t createThread(void (*task)(), uint32_t delay, osPriority_t priority, char* name)
{
    ThreadWrapper thread = {.taskFunction = task, .delay = delay};
    osThreadAttr_t threadAttrs = {.priority = priority, .name = name};

    return osThreadNew(rtosWrapper, &thread, &threadAttrs);
}
