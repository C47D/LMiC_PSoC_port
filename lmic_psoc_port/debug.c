/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "project.h"

#include "debug.h"

#include "lmic.h"

#define LED_OFF 0
#define LED_ON  1

#if 0
#define LED_PORT        GPIOA // use GPIO PA8 (LED4 on IMST, P11/PPS/EXT1_10/GPS6 on Blipper)
#define LED_PIN         8
#define USART_TX_PORT   GPIOA
#define USART_TX_PIN    9
#define GPIO_AF_USART1  0x07
#endif
    
void debug_init () {

    LED_Write( LED_OFF );

#if 0
    // configure LED pin as output
    hw_cfg_pin(LED_PORT, LED_PIN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
    debug_led(0);

    // configure USART1 (115200/8N1, tx-only)
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    hw_cfg_pin(USART_TX_PORT, USART_TX_PIN, GPIOCFG_MODE_ALT|GPIOCFG_OSPEED_40MHz|GPIOCFG_OTYPE_PUPD|GPIOCFG_PUPD_PUP|GPIO_AF_USART1);
    USART1->BRR = 277; // 115200
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE; // usart+transmitter enable
#endif
    // print banner
    debug_str("\r\n============== DEBUG STARTED ==============\r\n");
}

void debug_led(int val)
{
    LED_Write( (uint8_t)val );
#if 0
    hw_set_pin(LED_PORT, LED_PIN, val);
#endif
}

void debug_char(char c)
{
    UART_PutChar( c );
#if 0
    while( !(USART1->SR & USART_SR_TXE) );    
    USART1->DR = c;
#endif
}

void debug_hex(u1_t b)
{
    debug_char("0123456789ABCDEF"[b>>4]);
    debug_char("0123456789ABCDEF"[b&0xF]);
}

void debug_buf(const u1_t* buf, int len)
{
    while(len--) {
        debug_hex(*buf++);
        debug_char(' ');
    }
#if 0
    debug_char('\r');
    debug_char('\n');
#endif
    UART_PutCRLF();
}

void debug_uint(u4_t v)
{
    UART_PutHexInt( v );
#if 0
    for(s1_t n=24; n>=0; n-=8) {
        debug_hex(v>>n);
    }
#endif
}

void debug_int(s4_t v)
{
    char buf[10], *p = buf;
    int n = debug_fmt(buf, sizeof(buf), v, 10, 0, 0);
    while(n--)
        debug_char(*p++);
}

void debug_str(const char* str)
{
    UART_PutString( str );
#if 0    
    while(*str) {
        debug_char(*str++);
    }
#endif
}

void debug_val(const char* label, u4_t val)
{
    debug_str(label);
    debug_uint(val);
#if 0
    debug_char('\r');
    debug_char('\n');
#endif
    UART_PutCRLF();
}

void debug_valdec(const char* label, s4_t val)
{
    debug_str(label);
    debug_int(val);
#if 0
    debug_char('\r');
    debug_char('\n');
#endif
    UART_PutCRLF();
}

int debug_fmt(char* buf, int max, s4_t val, int base, int width, char pad)
{
    char num[33], *p = num, *b = buf;
    u4_t m, v;
    // special handling of negative decimals
    v = (base == 10 && val < 0) ? -val : val;
    // generate digits backwards
    do {
        *p++ = ((m=v%base) <= 9) ? m+'0' : m+'A'-10;
    } while( v /= base );
    // prefix negative decimals with '-'
    if(base == 10 && val < 0) {
        *p++ = '-';
    }
    // add leading zeroes or spaces
    while( b-buf < max-1 && b-buf < width-(p-num) ) {
        *b++ = pad;
    }
    // copy digits and sign forwards
    do *b++ = *--p;
    while( b-buf < max && p > num );
    // return number of characters written
    return b - buf;
}

void debug_event(int ev)
{
    static const char* evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_RFU1]           = "RFU1",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
        [EV_SCAN_FOUND]     = "SCAN_FOUND",
        [EV_TXSTART]        = "EV_TXSTART",
    };
    
    debug_str((ev < sizeof(evnames)/sizeof(evnames[0])) ? evnames[ev] : "EV_UNKNOWN" );
#if 0
    debug_char('\r');
    debug_char('\n');
#endif
    UART_PutCRLF();
}
