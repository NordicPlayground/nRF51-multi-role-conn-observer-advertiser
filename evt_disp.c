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

#include "evt_disp.h"

#include "evt_fifo.h"
#include "nrf_soc.h"

/*****************************************************************************
* Local definitions
*****************************************************************************/

/* The event fifo buffer size. Must be at least 3 elements
 * to accommodate for 3 scan requests per advertising event.
 * Must also be power of two.
 */
#define EVENT_DISPATCHER_FIFO_SIZE (8)

/*****************************************************************************
* Static Globals
*****************************************************************************/

static evt_fifo_t evt_fifo;
static btle_event_t evt_buffer[EVENT_DISPATCHER_FIFO_SIZE];

static IRQn_Type irq_type;

/*****************************************************************************
* Interface Functions
*****************************************************************************/

void evt_disp_init(IRQn_Type irq)
{
	if (irq >= SWI0_IRQn && irq <= SWI5_IRQn)
	{
		irq_type = irq;
		NVIC_SetPriority(irq, 3); /* APP LOW interrupt level */
		NVIC_EnableIRQ(irq);
		evt_fifo_init(&evt_fifo, &evt_buffer[0], EVENT_DISPATCHER_FIFO_SIZE);
	}
}

bool evt_disp_dispatch(btle_event_t* evt)
{
	if (NRF_SUCCESS == evt_fifo_put(&evt_fifo, evt))
	{
		NVIC_SetPendingIRQ(irq_type); /* Set interrupt */
		return true;
	}
	else 
	{
		return false;
	}
}

bool evt_disp_get(btle_event_t* evt)
{
	if (NULL == evt)
	{
		return false;
	}
	
	return (NRF_SUCCESS == evt_fifo_get(&evt_fifo, evt));
}

bool evt_disp_pending(void)
{
	return (NRF_SUCCESS == evt_fifo_pending(&evt_fifo));
}
