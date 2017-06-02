/*
** HAL targeted to PSoC5LP and SX1272 IC
*/

#include "project.h"

#include "hal.h"
#include "lmic.h"
#include "debug.h"

static struct {
    int irq_level;
    u4_t ticks;
} HAL;

CY_ISR( DIO_Handler )
{
    // interrupt triggered on the rising edge of
    // DIO0, DIO1, DIO2 input lines, and the corresponding
    // interrupt handlers must invoke the function
    // radio_irq_handler() passing the line whichh generated
    // the interrupt as argument ( 0, 1, 2)
    
    // TODO: Fix it!!!
    volatile uint8_t mask;
    
    mask = DIO_ClearInterrupt();
    
    switch(mask)
    {
    case 0x01: // DIO0
        radio_irq_handler( 0 );
        break;
    case 0x02: // DIO1
        radio_irq_handler( 1 );
        break;
    case 0x04: // DIO2
        radio_irq_handler( 2 );
        break;
    }

}

CY_ISR( Timer_Handler )
{    
    // Update the system clock tick on roll-over of the counter
    // It is sufficient that the CPU waked from sleep and the tun-time
    // environment of LMiC can check for pending actions.
#if 0
    if(TIM9->SR & TIM_SR_UIF) { // overflow
        HAL.ticks++;
    }
    if((TIM9->SR & TIM_SR_CC2IF) && (TIM9->DIER & TIM_DIER_CC2IE)) { // expired
        // do nothing, only wake up cpu
    }
    TIM9->SR = 0; // clear IRQ flags
#endif

// overflow
    HAL.ticks++;
    

    // Read the Timer STATUS reg to clear the interrupt
     (void)Timer_ReadStatusRegister();
}

void hal_init( void )
{
    //hal_time_init();
    
    // Most of the HAL configuration is done on the schematic
    // so not so much to be done here.
    
    memset( &HAL, 0x00, sizeof( HAL ) );
    hal_disableIRQs();
    
    // Configure radio I/O and interrupt handler
    isr_DIO_StartEx( DIO_Handler );
    
    // Configure radio SPI
    SPI_Start();
    
    // Configure timer and interrupt handler
    //Timer_Start();
    // isr_Timer_StartEx( Timer_Handler );
    
    hal_enableIRQs();
    
    // Make sure that SPI communication with the radio module works
    // by reading the "version" register 0x42 of the radio module.
    hal_pin_nss(0);
    u1_t val = hal_spi(0x42 & 0x7F);
    hal_pin_nss(1);
    
    if( 0 == val )
    {
        debug_str("HAL: There is an issue with the SPI communication to the radio module.\r\n");
        hal_failed();
    }
    else if( 0x12 == val)
    {
        debug_str("HAL: Detected the SX1276 radio module.\r\n");
    }
    
}

// set radio NSS pin to given value
void hal_pin_nss(u1_t val)
{
    SS_Write( (uint8_t)val );
#if 0
    hw_set_pin(GPIOx(NSS_PORT), NSS_PIN, val);
#endif
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx(u1_t val)
{
    ASSERT(val == 1 || val == 0);
    
    if( 1 == val )
    {
        RX_Write( 0 );
        TX_Write( 1 );
    } else  {
        RX_Write( 1 );
        TX_Write( 0 );
    }
#if 0
    ASSERT(val == 1 || val == 0);
#ifndef CFG_sx1276mb1_board
    hw_set_pin(GPIOx(RX_PORT), RX_PIN, ~val);
#endif
    hw_set_pin(GPIOx(TX_PORT), TX_PIN, val);
#endif
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst(u1_t val)
{
    RST_Write( (uint8_t)val );
#if 0
    if(val == 0 || val == 1) { // drive pin
        hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
        hw_set_pin(GPIOx(RST_PORT), RST_PIN, val);
    } else { // keep pin floating
        hw_cfg_pin(GPIOx(RST_PORT), RST_PIN, GPIOCFG_MODE_INP | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_OPEN);
    }
#endif
}

// perform SPI transaction with radio
u1_t hal_spi(u1_t outval) {
    SPI_WriteTxData( outval );
    while( 0 == ( SPI_ReadRxStatus() & SPI_STS_SPI_DONE ) );
    return SPI_ReadRxData();
    
#if 0
    SPI2->DR = out;
    while( (SPI2->SR & SPI_SR_RXNE ) == 0);
    return SPI2->DR; // in
#endif
}

void hal_disableIRQs( void )
{
    // TODO: Check if it's the right way to do it
    CyGlobalIntDisable;    
}

void hal_enableIRQs( void )
{
    // TODO: Check if it's the right way to do it
    CyGlobalIntEnable;
}

void hal_sleep( void )
{
    // __WFI();
    // Sleep until interrupt occurs, Preferably system components
    // can be put in low-power mode before sleep, and be re-initialized
    // after sleep.
}

u4_t hal_ticks( void )
{
#if 0
    u4_t hal_ticks () {
        hal_disableIRQs();
        u4_t t = HAL.ticks;
        u2_t cnt = TIM9->CNT;
        if( (TIM9->SR & TIM_SR_UIF) ) {
            // Overflow before we read CNT?
            // Include overflow in evaluation but
            // leave update of state to ISR once interrupts enabled again
            cnt = TIM9->CNT;
            t++;
        }
        hal_enableIRQs();
        return (t<<16)|cnt;
    }
#endif
    hal_disableIRQs();

    u4_t t = HAL.ticks;
    
    hal_enableIRQs();
    return 0;
}

void hal_waitUntil(u4_t time)
{
    (void)time;
}

u1_t hal_checkTimer(u4_t targettime)
{
#if 0
    u2_t dt;
    TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
        TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
        return 1;
    } else { // rewind timer (fully or to exact time))
        TIM9->CCR2 = TIM9->CNT + dt;   // set comparator
        TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
        TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
        return 0;
    }
#endif
    u2_t dt;
// clear any pending interrupts

    if( dt )
    {
        return 1;
    }
    else // Rewind timer
    {
        return 0;
    }

    (void)targettime;
}

void hal_failed( void )
{
    debug_str("HAL failed, halting...\r\n");
    hal_disableIRQs();
    while(1);
}

extern void radio_irq_handler(u1_t dio);

#if 0
void EXTI_IRQHandler () {
    // DIO 0
    if((EXTI->PR & (1<<DIO0_PIN)) != 0) { // pending
        EXTI->PR = (1<<DIO0_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(0);
    }
    // DIO 1
    if((EXTI->PR & (1<<DIO1_PIN)) != 0) { // pending
        EXTI->PR = (1<<DIO1_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(1);
    }
    // DIO 2
    if((EXTI->PR & (1<<DIO2_PIN)) != 0) { // pending
        EXTI->PR = (1<<DIO2_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(2);
    }
       
#ifdef CFG_EXTI_IRQ_HANDLER
    // invoke user-defined interrupt handler
    {
        extern void CFG_EXTI_IRQ_HANDLER(void);
        CFG_EXTI_IRQ_HANDLER();
    }
#endif // CFG_EXTI_IRQ_HANDLER
}
#endif

#if CFG_lmic_clib
void EXTI0_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI1_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI2_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI3_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI4_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI15_10_IRQHandler () {
    EXTI_IRQHandler();
}
#endif // CFG_lmic_clib

// -----------------------------------------------------------------------------
// SPI

// for sx1272 and 1276

#if 0
#define SCK_PORT   0 // SCK:  PA5
#define SCK_PIN    5
#define MISO_PORT  0 // MISO: PA6
#define MISO_PIN   6
#define MOSI_PORT  0 // MOSI: PA7
#define MOSI_PIN   7

#define GPIO_AF_SPI1        0x05

#endif

#ifdef CFG_lmic_clib

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
#ifndef CFG_clock_HSE
    PWR->CR |= PWR_CR_DBP; // disable write protect
    RCC->CSR |= RCC_CSR_LSEON; // switch on low-speed oscillator @32.768kHz
    while( (RCC->CSR & RCC_CSR_LSERDY) == 0 ); // wait for it...
#endif
    
    RCC->APB2ENR   |= RCC_APB2ENR_TIM9EN;     // enable clock to TIM9 peripheral 
    RCC->APB2LPENR |= RCC_APB2LPENR_TIM9LPEN; // enable clock to TIM9 peripheral also in low power mode
    RCC->APB2RSTR  |= RCC_APB2RSTR_TIM9RST;   // reset TIM9 interface
    RCC->APB2RSTR  &= ~RCC_APB2RSTR_TIM9RST;  // reset TIM9 interface

#if CFG_clock_HSE
    TIM9->PSC  = (640 - 1); // HSE_CLOCK_HWTIMER_PSC-1);  XXX: define HSE_CLOCK_HWTIMER_PSC somewhere
#else
    TIM9->SMCR = TIM_SMCR_ECE; // external clock enable (source clock mode 2) with no prescaler and no filter
#endif
    
    NVIC->IP[TIM9_IRQn] = 0x70; // interrupt priority
    NVIC->ISER[TIM9_IRQn>>5] = 1<<(TIM9_IRQn&0x1F);  // set enable IRQ

    // enable update (overflow) interrupt
    TIM9->DIER |= TIM_DIER_UIE;
    
    // Enable timer counting
    TIM9->CR1 = TIM_CR1_CEN;
}

u4_t hal_ticks () {
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    u2_t cnt = TIM9->CNT;
    if( (TIM9->SR & TIM_SR_UIF) ) {
        // Overflow before we read CNT?
        // Include overflow in evaluation but
        // leave update of state to ISR once interrupts enabled again
        cnt = TIM9->CNT;
        t++;
    }
    hal_enableIRQs();
    return (t<<16)|cnt;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time) {
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    u2_t dt;
    TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
        TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
        return 1;
    } else { // rewind timer (fully or to exact time))
        TIM9->CCR2 = TIM9->CNT + dt;   // set comparator
        TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
        TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
        return 0;
    }
}
  
void TIM9_IRQHandler () {
    if(TIM9->SR & TIM_SR_UIF) { // overflow
        HAL.ticks++;
    }
    if((TIM9->SR & TIM_SR_CC2IF) && (TIM9->DIER & TIM_DIER_CC2IE)) { // expired
        // do nothing, only wake up cpu
    }
    TIM9->SR = 0; // clear IRQ flags
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
    __disable_irq();
    HAL.irqlevel++;
}

void hal_enableIRQs () {
    if(--HAL.irqlevel == 0) {
        __enable_irq();
    }
}

void hal_sleep () {
    // low power sleep mode
#ifndef CFG_no_low_power_sleep_mode
    PWR->CR |= PWR_CR_LPSDSR;
#endif
    // suspend execution until IRQ, regardless of the CPSR I-bit
    __WFI();
}

// -----------------------------------------------------------------------------

void hal_init () {
    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

    hal_enableIRQs();
}

void hal_failed () {
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

#endif // CFG_lmic_clib
