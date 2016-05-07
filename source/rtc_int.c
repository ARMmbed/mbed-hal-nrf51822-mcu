/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stddef.h>
#include <stdbool.h>
#include "cmsis.h"
#include "PeripheralNames.h"
#include "nrf_delay.h"
#include "rtc_int.h"

/** 
 * Defines whether or not the rtc1 
 * is initialized already 
 */
static bool rtc1_init =         false;
/**
 * The number of times the 24-bit 
 * RTC counter has overflowed. 
 */
volatile uint32_t        overflowCount;                   
/**
 * The handler to be executed on the
 * rtc1 overflow event
 */
static RTC1EventHandler ofHandler;                   
/**
 * The handler to be executed on the
 * rtc1 capture compare reg0 event
 */
static RTC1EventHandler cc0Handler;                   
/**
 * The handler to be executed on the
 * rtc1 capture compare reg1 event
 */
static RTC1EventHandler cc1Handler;                   
/**
 * The handler to be executed on the
 * rtc1 capture compare reg2 event
 */
static RTC1EventHandler cc2Handler;

void rtc1_enableCaptureCompareEventReg0(void)
{
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE0_Msk;
    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;     
}

void rtc1_disableCaptureCompareEventReg0(void)
{
    NRF_RTC1->INTENCLR = RTC_INTENSET_COMPARE0_Msk;
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE0_Msk;  
}

void rtc1_clearCaptureCompareEventReg0(void)
{
    NRF_RTC1->EVENTS_COMPARE[0] = 0;
}

void rtc1_enableCaptureCompareEventReg1(void)
{
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE1_Msk;
    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE1_Msk;
}

void rtc1_disableCaptureCompareEventReg1(void)
{
    NRF_RTC1->INTENCLR = RTC_INTENSET_COMPARE1_Msk;
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE1_Msk;  
}

void rtc1_clearCaptureCompareEventReg1(void)
{
    NRF_RTC1->EVENTS_COMPARE[1] = 0;
}

void rtc1_enableCaptureCompareEventReg2(void)
{
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE2_Msk;
    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE2_Msk;
}

void rtc1_disableCaptureCompareEventReg2(void)
{
    NRF_RTC1->INTENCLR = RTC_INTENSET_COMPARE2_Msk;
    NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE2_Msk;  
}

void rtc1_clearCaptureCompareEventReg2(void)
{
    NRF_RTC1->EVENTS_COMPARE[2] = 0;
}

void rtc1_setCaptureCompareValueReg0(uint32_t value)
{
    NRF_RTC1->CC[0] = value;
}

void rtc1_setCaptureCompareValueReg1(uint32_t value)
{
    NRF_RTC1->CC[1] = value;
}

void rtc1_setCaptureCompareValueReg2(uint32_t value)
{
    NRF_RTC1->CC[2] = value;
}

uint32_t rtc1_getCaptureCompareValueReg0(void)
{
    return NRF_RTC1->CC[0];
}

uint32_t rtc1_getCaptureCompareValueReg1(void)
{
    return NRF_RTC1->CC[1];
}

uint32_t rtc1_getCaptureCompareValueReg2(void)
{
    return NRF_RTC1->CC[2];
}

void rtc1_enableOverflowEvent(void)
{
    NRF_RTC1->EVTENCLR = RTC_EVTEN_OVRFLW_Msk;
    NRF_RTC1->INTENSET = RTC_INTENSET_OVRFLW_Msk;
}

void rtc1_disableOverflowEvent(void)
{
    NRF_RTC1->INTENCLR = RTC_INTENSET_OVRFLW_Msk;
    NRF_RTC1->EVTENCLR = RTC_EVTEN_OVRFLW_Msk;
}

void rtc1_setOverflowEventHandler( RTC1EventHandler handler )
{
    ofHandler = handler;
}

void rtc1_setCaptureCompareEventHandlerReg0( RTC1EventHandler handler )
{
    cc0Handler = handler;    
}

void rtc1_setCaptureCompareEventHandlerReg1( RTC1EventHandler handler )
{
    cc1Handler = handler;    
}

void rtc1_setCaptureCompareEventHandlerReg2( RTC1EventHandler handler )
{
    cc2Handler = handler;    
}

static inline void invokeHandler(const RTC1EventHandler handler)
{
    if( handler != NULL ) { handler(); }
}

void rtc1_start(void)
{
    if( rtc1_init ) return;
    
    NRF_RTC1->PRESCALER = RTC1_PRESCALER; /* for no pre-scaling. */

    rtc1_enableOverflowEvent();

    NVIC_SetPriority(RTC1_IRQn, RTC1_IRQ_PRI);
    NVIC_ClearPendingIRQ(RTC1_IRQn);
    NVIC_EnableIRQ(RTC1_IRQn);

    NRF_RTC1->TASKS_START = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);
    
    rtc1_init = true;
}

/**
 * @brief Function for stopping the RTC1 timer. We don't expect to call this.
 */
void rtc1_stop(void)
{
    if( !rtc1_init ) return;
    
    NVIC_DisableIRQ(RTC1_IRQn);
    rtc1_disableCaptureCompareEventReg0();
    rtc1_disableCaptureCompareEventReg1();
    rtc1_disableCaptureCompareEventReg2();
    rtc1_disableOverflowEvent();

    NRF_RTC1->TASKS_STOP = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

    NRF_RTC1->TASKS_CLEAR = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);
    
    rtc1_init = false;
}

/**
 * @brief Function for returning the current value of the RTC1 counter.
 *
 * @return Current RTC1 counter as a 64-bit value with 56-bit precision (even
 *         though the underlying counter is 24-bit)
 */
uint64_t rtc1_getCounter64(void)
{
    if (NRF_RTC1->EVENTS_OVRFLW) {
        overflowCount++;
        NRF_RTC1->EVENTS_OVRFLW = 0;
        NRF_RTC1->EVTENCLR      = RTC_EVTEN_OVRFLW_Msk;
    }
    return ((uint64_t)overflowCount << 24) | NRF_RTC1->COUNTER;
}

/**
 * @brief Function for returning the current value of the RTC1 counter.
 *
 * @return Current RTC1 counter as a 32-bit value (even though the underlying counter is 24-bit)
 */
uint32_t rtc1_getCounter(void)
{
    return rtc1_getCounter64();
}

uint32_t rtc1_getOverflowsCounter(void)
{
    return overflowCount;
}

/**
 * @brief Function for handling the RTC1 interrupt.
 *
 * @details Checks for timeouts, and executes timeout handlers for expired timers.
 */
void RTC1_IRQHandler(void)
{
    if (NRF_RTC1->EVENTS_OVRFLW) {
        overflowCount++;
        NRF_RTC1->EVENTS_OVRFLW = 0;
        NRF_RTC1->EVTENCLR      = RTC_EVTEN_OVRFLW_Msk;
        invokeHandler(ofHandler);
        //TODO: Execute an overflow callback
    }
    if (NRF_RTC1->EVENTS_COMPARE[0]) {
        NRF_RTC1->EVENTS_COMPARE[0] = 0;
        NRF_RTC1->EVTENCLR          = RTC_EVTEN_COMPARE0_Msk;
        invokeHandler(cc0Handler);
        //invokeCallback();
        // TODO: Execute an cc reg1 callback
    }
    if (NRF_RTC1->EVENTS_COMPARE[1]) {
        // Compare[1] used by lp ticker
        NRF_RTC1->EVENTS_COMPARE[1] = 0;
        NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE1_Msk;
        invokeHandler(cc1Handler);
    }
    if (NRF_RTC1->EVENTS_COMPARE[2]) {
        // Compare[2] used by anyone ...
        NRF_RTC1->EVENTS_COMPARE[2] = 0;
        NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE2_Msk;
        invokeHandler(cc2Handler);
    }
}