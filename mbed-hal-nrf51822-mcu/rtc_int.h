/* mbed Microcontroller Library
 * Copyright (c) 2013 Nordic Semiconductor
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
#ifndef MBED_RTCINT_H
#define MBED_RTCINT_H

#define MAX_RTC_COUNTER_VAL     0x00FFFFFF               /**< Maximum value of the RTC counter. */
#define RTC_CLOCK_FREQ          (uint32_t)(32768)
#define RTC1_PRESCALER          (0)
#define RTC1_IRQ_PRI            3                        /**< Priority of the RTC1 interrupt (used
                                                          *  for checking for timeouts and executing
                                                          *  timeout handlers). This must be the same
                                                          *  as APP_IRQ_PRIORITY_LOW; taken from the
                                                          *  Nordic SDK. */
#define MAX_RTC_TASKS_DELAY     47                       /**< Maximum delay until an RTC task is executed. */

#define FUZZY_RTC_TICKS          2  /* RTC COMPARE occurs when a CC register is N and the RTC
                                     * COUNTER value transitions from N-1 to N. If we're trying to
                                     * setup a callback for a time which will arrive very shortly,
                                     * there are limits to how short the callback interval may be for us
                                     * to rely upon the RTC Compare trigger. If the COUNTER is N,
                                     * writing N+2 to a CC register is guaranteed to trigger a COMPARE
                                     * event at N+2. */

#define RTC_UNITS_TO_MICROSECONDS(RTC_UNITS) (((RTC_UNITS) * (uint64_t)1000000) / RTC_CLOCK_FREQ)
#define MICROSECONDS_TO_RTC_UNITS(MICROS)    ((((uint64_t)(MICROS) * RTC_CLOCK_FREQ) + 999999) / 1000000)

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*RTC1EventHandler)(void);

#ifdef __cplusplus
}
#endif

void rtc1_enableCaptureCompareEventReg0(void);
void rtc1_disableCaptureCompareEventReg0(void);
void rtc1_clearCaptureCompareEventReg0(void);

void rtc1_enableCaptureCompareEventReg1(void);
void rtc1_disableCaptureCompareEventReg1(void);
void rtc1_clearCaptureCompareEventReg1(void);

void rtc1_enableCaptureCompareEventReg2(void);
void rtc1_disableCaptureCompareEventReg2(void);
void rtc1_clearCaptureCompareEventReg2(void);

void rtc1_setCaptureCompareValueReg0(uint32_t value);
void rtc1_setCaptureCompareValueReg1(uint32_t value);
void rtc1_setCaptureCompareValueReg2(uint32_t value);

uint32_t rtc1_getCaptureCompareValueReg0(void);
uint32_t rtc1_getCaptureCompareValueReg1(void);
uint32_t rtc1_getCaptureCompareValueReg2(void);

void rtc1_enableOverflowEvent(void);
void rtc1_disableOverflowEvent(void);

void rtc1_setOverflowEventHandler( RTC1EventHandler handler );

void rtc1_setCaptureCompareEventHandlerReg0( RTC1EventHandler handler );
void rtc1_setCaptureCompareEventHandlerReg1( RTC1EventHandler handler );
void rtc1_setCaptureCompareEventHandlerReg2( RTC1EventHandler handler );


void rtc1_start(void);

/**
 * @brief Function for stopping the RTC1 timer. We don't expect to call this.
 */
void rtc1_stop(void);

/**
 * @brief Function for returning the current value of the RTC1 counter.
 *
 * @return Current RTC1 counter as a 64-bit value with 56-bit precision (even
 *         though the underlying counter is 24-bit)
 */
uint64_t rtc1_getCounter64(void);

/**
 * @brief Function for returning the current value of the RTC1 counter.
 *
 * @return Current RTC1 counter as a 32-bit value (even though the underlying counter is 24-bit)
 */
uint32_t rtc1_getCounter(void);

uint32_t rtc1_getOverflowsCounter(void);

#endif