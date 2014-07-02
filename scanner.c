/* Copyright (c) 2014, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   * Neither the name of Nordic Semiconductor ASA nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "scanner.h"

#include "radio.h"

#include "nrf_gpio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*****************************************************************************
* Static Globals
*****************************************************************************/

static scanner_state_t m_scanner_state;
static uint8_t m_rx_buf[255];
static uint8_t m_tx_buf[] =
{
	0xC3,                               // BLE Header (PDU_TYPE: SCAN_REQ, TXadd: 1 (random address), RXadd: 1 (random address)
	0x0C,                               // Length of payload: 12
	0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
	0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, // InitAddr LSByte first
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // AdvAddr LSByte first
};

/*****************************************************************************
* Static Functions
*****************************************************************************/

uint8_t m_packet_type(uint8_t *packet);

uint8_t m_packet_type(uint8_t *packet)
{
	return packet[0] & 0x0F;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

void scanner_event_start(void)
{
	radio_init(39);
	radio_receive(m_rx_buf);
	
	m_scanner_state = SCANNER_STATE_WAITING;
}

void scanner_event_radio(void)
{
	uint8_t type;
	volatile uint8_t t;
	
	switch (m_scanner_state)
	{
		/* Packet received */
		case SCANNER_STATE_WAITING:
			/* Abort immediately if packet has invalid CRC */
			if (NRF_RADIO->CRCSTATUS == 0)
			{
				radio_transmit_abort();
				return;
			}
			
			type = m_packet_type(m_rx_buf);
			switch (type)
			{
				case 0:
					/* Fall-through */
				case 6:
					/* Received ADV_IND or ADV_SCAN_IND. Sending SCAN_REQ. */
					memcpy(&m_tx_buf[9], &m_rx_buf[3], 6);
					radio_transmit(m_tx_buf);
					m_scanner_state = SCANNER_STATE_RECEIVED;
					break;
				
				case 4:
					/* Received SCAN_RSP */
					t = 2;
					break;
				
				default:
					radio_transmit_abort();
			}
			break;
		
		case SCANNER_STATE_RECEIVED:
			radio_receive(m_rx_buf);
			m_scanner_state = SCANNER_STATE_WAITING;
			break;
	}
}

void scanner_event_timer(void)
{
}

void scanner_event_extend_succeed(void)
{
}

void scanner_event_extend_failed(void)
{
}
