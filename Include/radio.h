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

#ifndef __RADIO_H__
#define __RADIO_H__

#include "nrf_soc.h"

#include <stdbool.h>
#include <stdint.h>

#define RADIO_RSSI_INVALID 255

void radio_event_cb (void);
void radio_timeout_cb (void);

/* Initializes the radio. Assumes that the HF crystal is started */
void radio_init(uint8_t channel);

/* Return to disabled state */
void radio_disable (void);

/* Set radio buffer */
void radio_buffer_configure (uint8_t * const buff);

/* Enable capture of RSSI. Will automatically disable after capture. */
void radio_rssi_enable (void);

/* Get captured RSSI value. A value of RADIO_RSSI_INVALID indicates an
 * invalid RSSI value.
 */
uint8_t radio_rssi_get (void);

/* Configures radio to receive */
void radio_rx_prepare (bool start_immediately);

/* Initialize packet receive timeout */
void radio_rx_timeout_init (void);

/* Enable packet receive timeout */
void radio_rx_timeout_enable (void);

/* Disable packet receive timeout */
void radio_rx_timeout_disable (void);

/* Enable SCAN_RSP receive timeout */
void radio_timeout_enable (void);

/* Change to TX mode on packet receipt */
void radio_tx_mode_on_receipt (void);

/* Configure radio to transmit */
void radio_tx_prepare (void);

#endif /* __RADIO_H__ */
