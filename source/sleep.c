/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
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
#include "sleep_api.h"
#include "cmsis.h"
#include "mbed-drivers/mbed_interface.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"

void mbed_enter_sleep(sleep_t *obj)
{
    (void)obj;

    // ensure debug is disconnected if semihost is enabled....
    NRF_POWER->TASKS_LOWPWR = 1;

    SCB->SCR |= SCB_SCR_SEVONPEND_Msk; /* send an event when an interrupt is pending.
                                        * This helps with the wakeup from the following app_evt_wait(). */

    uint8_t sd_enabled;

    // look if exceptions are enabled or not, if they are, it is possible to make an SVC call
    // and check if the soft device is running
    if ((__get_PRIMASK() == 0) && (sd_softdevice_is_enabled(&sd_enabled) == NRF_SUCCESS) && (sd_enabled == 1)) {
        // soft device is enabled, use the primitives from the soft device to go to sleep
        sd_app_evt_wait();
    } else {
        // impossible to use soft device primitive, just wait for events
        __WFE();
    }
}

void mbed_exit_sleep(sleep_t *obj)
{
    (void)obj;
}
