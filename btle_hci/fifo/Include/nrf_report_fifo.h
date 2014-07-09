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

#ifndef __NRF_REPORT_FIFO_H__
#define __NRF_REPORT_FIFO_H__

#include "btle.h"
#include "nrf_error.h"

#include <stdint.h>
#include <stdlib.h>

/**@brief A FIFO instance structure. Keeps track of which events to read and write next.
 *        Also it keeps the information about which memory is allocated for the buffer
 *        and its size. This needs to be initialized by nrf_report_fifo_init() before use.
 */
typedef struct
{
    nrf_report_t*      p_buf;           /**< Pointer to FIFO buffer memory.                      */
    uint16_t           buf_size_mask;   /**< Read/write index mask. Also used for size checking. */
    volatile uint32_t  read_pos;        /**< Next read position in the FIFO buffer.              */
    volatile uint32_t  write_pos;       /**< Next write position in the FIFO buffer.             */
} nrf_report_fifo_t;

/**@brief Function for initializing the FIFO.
 *
 * @param[out] p_fifo   FIFO object.
 * @param[in]  p_buf    FIFO buffer for storing data. The buffer size has to be a power of two.
 * @param[in]  buf_size Size of the FIFO buffer provided, has to be a power of 2.
 *
 * @retval     NRF_SUCCESS              If initialization was successful.
 * @retval     NRF_ERROR_NULL           If a NULL pointer is provided as buffer.
 * @retval     NRF_ERROR_INVALID_LENGTH If size of buffer provided is not a power of two.
 */
uint32_t nrf_report_fifo_init(nrf_report_fifo_t * p_fifo, nrf_report_t* p_buf, uint16_t buf_size);

/**@brief Function for adding an element to the FIFO.
 *
 * @param[in]  p_fifo   Pointer to the FIFO.
 * @param[in]  elem     Data pointer to add to the FIFO.
 *
 * @retval     NRF_SUCCESS              If an element has been successfully added to the FIFO.
 * @retval     NRF_ERROR_NO_MEM         If the FIFO is full.
 */
uint32_t nrf_report_fifo_put(nrf_report_fifo_t * p_fifo, nrf_report_t* elem);

/**@brief Function for getting the next element from the FIFO.
 *
 * @param[in]  p_fifo   Pointer to the FIFO.
 * @param[out] p_elem   Pointer fetched from the FIFO.
 *
 * @retval     NRF_SUCCESS              If an element was returned.
 * @retval     NRF_ERROR_NOT_FOUND      If there is no more elements in the queue.
 */
uint32_t nrf_report_fifo_get(nrf_report_fifo_t * p_fifo, nrf_report_t* p_elem);

uint32_t nrf_report_fifo_peek(nrf_report_fifo_t * p_fifo, nrf_report_t* p_elem);

uint32_t nrf_report_fifo_pending(nrf_report_fifo_t * p_fifo);

/**@brief Function for flushing the FIFO.
 *
 * @param[in]  p_fifo   Pointer to the FIFO.
 *
 * @retval     NRF_SUCCESS              If the FIFO flushed successfully.
 */
uint32_t nrf_report_fifo_flush(nrf_report_fifo_t * p_fifo);

#endif /* __EVT_FIFO_H__ */

/** @} */
