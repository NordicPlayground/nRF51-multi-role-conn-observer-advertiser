/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __EVT_DISPATCHER_H__
#define __EVT_DISPATCHER_H__

#include "btle.h"
#include "nrf_soc.h"

#include <stdbool.h>
#include <stdint.h>

/**
* Initialize the event dispatcher.
* Parameter irq must be a software interrupt (i.e. SWI0_IRQn - SWI9_IRQn)
* The user will get an interrupt on APP LOW level whenever there's a pending 
* event.
*/
uint32_t nrf_report_disp_init(IRQn_Type irq);

/**
* Put an event in the dispatcher queue. Returns true
* if dispatch is successful
*/
uint32_t nrf_report_disp_dispatch(nrf_report_t* evt);

/**
* Get event from dispatcher queue. Parameter evt
* cannot be NULL. Returns true if fetch is successful.
*/
uint32_t nrf_report_disp_get(nrf_report_t* evt);

/**
* Returns whether there are any events in the dispatcher queue.
*/
uint32_t nrf_report_disp_pending(void);

#endif /* __EVT_DISPATCHER_H__ */
