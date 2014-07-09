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

#include "nrf_report_fifo.h"

#include "btle.h"
#include "app_common/app_util.h"

#include <string.h>

/*****************************************************************************
* Local definitions
*****************************************************************************/

/* Macro for calculating the FIFO length. */
#define FIFO_LENGTH (p_fifo->write_pos - p_fifo->read_pos)

/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t nrf_report_fifo_init (nrf_report_fifo_t * p_fifo, nrf_report_t* p_buf, uint16_t buf_size)
{
    /* Check buffer for null pointer. */
    if (p_buf == NULL)
    {
        return NRF_ERROR_NULL;
    }

    /* Check that the buffer size is a power of two. */
    if (!IS_POWER_OF_TWO(buf_size))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    p_fifo->p_buf         = p_buf;
    p_fifo->buf_size_mask = buf_size - 1;
    p_fifo->read_pos      = 0;
    p_fifo->write_pos     = 0;

    return NRF_SUCCESS;
}

uint32_t nrf_report_fifo_put (nrf_report_fifo_t * p_fifo, nrf_report_t* p_elem)
{
    if (FIFO_LENGTH <= p_fifo->buf_size_mask)
    {
      memcpy((void*) &p_fifo->p_buf[p_fifo->write_pos & p_fifo->buf_size_mask], (void*) p_elem, sizeof(nrf_report_t));
      p_fifo->write_pos++;
      return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}

uint32_t nrf_report_fifo_get (nrf_report_fifo_t * p_fifo, nrf_report_t* p_elem)
{
    if (FIFO_LENGTH != 0)
    {
        memcpy((void*) p_elem, (void*) &p_fifo->p_buf[p_fifo->read_pos & p_fifo->buf_size_mask], sizeof(nrf_report_t));
        p_fifo->read_pos++;
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NOT_FOUND;
}

uint32_t nrf_report_fifo_peek (nrf_report_fifo_t * p_fifo, nrf_report_t* p_elem)
{
    if (FIFO_LENGTH != 0)
    {
        memcpy((void*) p_elem, (void*) &p_fifo->p_buf[p_fifo->read_pos & p_fifo->buf_size_mask], sizeof(nrf_report_t));

        return NRF_SUCCESS;
    }

    return NRF_ERROR_NOT_FOUND;
}

uint32_t nrf_report_fifo_pending (nrf_report_fifo_t * p_fifo)
{
    if (FIFO_LENGTH != 0)
    {
        return NRF_SUCCESS;
    }
    else 
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t nrf_report_fifo_flush (nrf_report_fifo_t * p_fifo)
{
    p_fifo->read_pos = p_fifo->write_pos;
    return NRF_SUCCESS;
}
