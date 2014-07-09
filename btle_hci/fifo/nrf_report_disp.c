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

#include "nrf_report_disp.h"

#include "btle.h"
#include "nrf_error.h"
#include "nrf_report_fifo.h"
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

static nrf_report_fifo_t report_fifo;
static nrf_report_t evt_buffer[EVENT_DISPATCHER_FIFO_SIZE];

static IRQn_Type irq_type;

/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t nrf_report_disp_init(IRQn_Type irq)
{
  if (irq >= SWI0_IRQn && irq <= SWI5_IRQn)
  {
    irq_type = irq;
    NVIC_SetPriority(irq, 3); /* APP LOW interrupt level */
    NVIC_EnableIRQ(irq);
    nrf_report_fifo_init(&report_fifo, &evt_buffer[0], EVENT_DISPATCHER_FIFO_SIZE);
  }

  return NRF_SUCCESS;
}

uint32_t nrf_report_disp_dispatch(nrf_report_t* report)
{
  uint32_t err_code;

  err_code = nrf_report_fifo_put (&report_fifo, report);

  if (err_code == NRF_SUCCESS)
  {
    NVIC_SetPendingIRQ(irq_type); /* Set interrupt */
  }

  return err_code;
}

uint32_t nrf_report_disp_get(nrf_report_t* report)
{
  uint32_t err_code;

  if (report == NULL)
  {
    return NRF_ERROR_NULL;
  }

  err_code = nrf_report_fifo_get(&report_fifo, report);
 
  return err_code;
}

uint32_t nrf_report_disp_pending(void)
{
  return nrf_report_fifo_pending(&report_fifo);
}
