#include "cmsis_os2.h"
#include <stdint.h>

typedef struct {
    void (*taskFunction)();
    uint32_t delay;
} ThreadWrapper;

void rtosWrapper(void *);
osThreadId_t createThread(void (*)(), uint32_t, osPriority_t, char*);
