/*
** HAL targeted to PSoC5LP and SX1276 IC
*/

#include "project.h"

#include "hal.h"
#include "lmic.h"
#include "debug.h"

static struct {
    int irq_level;
    u4_t ticks;
} HAL;

enum {
    PIN_DIO_0 = 0x01,
    PIN_DIO_1 = 0x02,
    PIN_DIO_2 = 0x04,
};

extern void radio_irq_handler(u1_t dio);

// interrupt triggered on the rising edge of
// DIO0, DIO1, DIO2 input lines, and the corresponding
// interrupt handlers must invoke the function
// radio_irq_handler() passing the line whichh generated
// the interrupt as argument (0, 1, 2)
CY_ISR(DIO_Handler)
{
    switch(DIO_ClearInterrupt())
    {
    case PIN_DIO_0:
        radio_irq_handler(0);
        break;
    case PIN_DIO_1:
        radio_irq_handler(1);
        break;
    case PIN_DIO_2:
        radio_irq_handler(2);
        break;
    }

}

// Update the system clock tick on roll-over of the counter
// It is sufficient that the CPU waked from sleep and the tun-time
// environment of LMiC can check for pending actions.
CY_ISR(Timer_Handler)
{    
    TP_Write(~TP_Read());

    HAL.ticks++;
    
    // Read the Timer STATUS reg to clear the interrupt
     Timer_ReadStatusRegister();
}

void hal_init(void)
{   
    memset(&HAL, 0x00, sizeof(HAL));
    
    hal_disableIRQs();
    
    // Configure radio I/O and interrupt handler
    // GPIO configuration is done on the schematic
    isr_DIO_StartEx(DIO_Handler);
    isr_Timer_StartEx(Timer_Handler);
    
    // Configure radio SPI
    SPI_Start();
    Timer_Start();
    
    hal_enableIRQs();
}

// set radio NSS pin to given value
void hal_pin_nss(u1_t val)
{
    SS_Write(val);
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx(u1_t val)
{    
    if(1 == val) {
        //RX_Write(0);
        //TX_Write(1);
    } else {
        //RX_Write(1);
        //TX_Write(0);
    }
}

// set radio RST pin to given value
void hal_pin_rst(u1_t val)
{
    switch (val) {
    case 0:
        RST_Write(0);
        break;
    case 1:
    case 2:
        RST_Write(1);
        break;
    default:
        hal_failed();
        break;
    }
}

// perform SPI transaction with radio
u1_t hal_spi(u1_t outval)
{
    SPI_WriteTxData(outval);
    while(!(SPI_ReadTxStatus() & SPI_STS_BYTE_COMPLETE));
    return SPI_ReadRxData();
}

// See cy_boot page 53
void hal_disableIRQs( void )
{
    CyGlobalIntDisable;    
}

// See cy_boot page 53
void hal_enableIRQs( void )
{
    CyGlobalIntEnable;
}

// Sleep until interrupt occurs, Preferably system components
// can be put in low-power mode before sleep, and be re-initialized
// after sleep.
void hal_sleep( void )
{
    asm volatile("WFI");
}

u4_t hal_ticks( void )
{
    hal_disableIRQs();
    
    u4_t t = HAL.ticks;
    u2_t cnt = Timer_ReadCounter();
    
    hal_enableIRQs();
    
    return (t << 16) | cnt;
}

static u2_t deltaticks (u4_t time)
{
    u4_t t = hal_ticks();
    s4_t d = time - t;
    
    // in the past
    if (d <= 0) {
        return 0;
    }
    
    // far ahead
    if ((d >> 16) != 0) {
        return 0xFFFF;
    }
    
    return (u2_t)d;
}

// busy wait until timestamp is reached
void hal_waitUntil(u4_t time)
{
    while(deltaticks(time) != 0);
}

u1_t hal_checkTimer(u4_t targettime)
{
    u2_t dt;
    // clear any pending interrupts
    Timer_ReadStatusRegister();

    if((dt = deltaticks(targettime)) < 5) {
        return 1;
    } else { // Rewind timer
        return 0;
    }
}

void hal_failed( void )
{
    debug_str("HAL failed, halting...\r\n");
    hal_disableIRQs();

    while(1);
}
