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

// The lp ticker is using RTC which is shared with us ticker
//#include "us_ticker_api.h"
#include "lp_ticker_api.h"
#include "rtc_int.h"
#include "sleep_api.h"

void lp_ticker_init(void)
{
    rtc1_start();
}

uint32_t lp_ticker_read(void)
{
    return rtc1_getCounter();
}

void lp_ticker_set_interrupt(uint32_t now, uint32_t time)
{
    (void)now;    
    rtc1_setCaptureCompareValueReg1(time);
    rtc1_enableCaptureCompareEventReg1();
}

uint32_t lp_ticker_get_overflows_counter(void)
{
    return rtc1_getOverflowsCounter() >> 8;
}

uint32_t lp_ticker_get_compare_match(void)
{
    return rtc1_getCaptureCompareValueReg1();
}

void lp_ticker_sleep_until(uint32_t now, uint32_t time)
{
    lp_ticker_set_interrupt(now, time);
    sleep_t sleep_obj;
    mbed_enter_sleep(&sleep_obj);
    mbed_exit_sleep(&sleep_obj);
}
