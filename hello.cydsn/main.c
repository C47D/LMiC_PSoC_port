// Port of IBM LMiC for PSoC
// hello example project

#include "project.h"

#include "lmic.h"
#include "debug.h"

// LMIC application callbacks not used in his example
void os_getDevEui(u1_t* buf)
{
    (void)buf;
}

void os_getDevKey(u1_t* buf)
{
    (void)buf;
}

void os_getArtEui(u1_t* buf)
{
    (void)buf;
}

void onEvent(ev_t ev)
{
    (void)ev;
}

static int cnt = 0;

static void initfunc(osjob_t* job);

int main(void)
{
   
    osjob_t initjob;
    
    // initialize debug library
    debug_init();
    
    // initialize run-time env
    os_init();
    
    // setup initial job
    os_setCallback(&initjob, initfunc);
    
    // execute scheduled jobs and events
    os_runloop();

    // not reached
    while(1);
}

static void initfunc(osjob_t* job)
{
    // say hello
    debug_str("Hello World\r\n");
    // log counter
    debug_val("cnt = ", cnt++);
    // toggle LED
    debug_led( ~LED_Read() );
    // reschedule job every second
    os_setTimedCallback(job, os_getTime() + sec2osticks( 1 ), initfunc );
}

/* [] END OF FILE */
