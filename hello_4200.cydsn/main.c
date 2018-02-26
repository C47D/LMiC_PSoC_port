// Port of IBM LMiC for PSoC
// hello example project
#include "project.h"

#include "lmic.h"
#include "debug.h"

// LMIC application callbacks not used in his example
void os_getDevEui(u1_t *buf)
{
    (void)buf;
}

void os_getDevKey(u1_t *buf)
{
    (void)buf;
}

void os_getArtEui(u1_t *buf)
{
    (void)buf;
}

void onEvent(ev_t ev)
{
    (void)ev;
}

static void init_func(osjob_t *job);

int main(void)
{
    osjob_t initjob;
    
    debug_init();
    os_init();
    
    // setup initial job
    os_setCallback(&initjob, init_func);
    
    // execute scheduled jobs and events
    os_runloop();

    while (1) {
    }
}

static void init_func(osjob_t *job)
{
    static int cnt = 0;
    
    debug_str("Hello World\r\n");
    debug_val("cnt = ", cnt++);
    
    LED_Write(~LED_Read());
    
    // reschedule job every second
    os_setTimedCallback(job, os_getTime() + sec2osticks(1), init_func);
}

/* [] END OF FILE */
