/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ts_adv_packet.h"
#include "ts_peripheral.h"
#include "nrf_assert.h"



/*****************************************************************************
* Static Globals
*****************************************************************************/


/* The BLE-packet being transmitted */
static uint8_t *packet = NULL;

/* array of frequency bins related to ch 37, 38, 39 */
static const uint8_t freq_bins[] = {2, 26, 80};


/*****************************************************************************
* Static Functions
*****************************************************************************/

/** 
* Set up radio for BLE packets. Needs to be done at the beginning of every timeslot.
*/
static __INLINE void radio_init(void) 
{
  /* Set radio configuration parameters */
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->MODE 	 = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

  NRF_RADIO->FREQUENCY 	 = 2;					// Frequency bin 80, 2480MHz, channel 39.
  NRF_RADIO->DATAWHITEIV = 37;					// NOTE: This value needs to correspond to the frequency being used
	
	
	/* Configure Access Address to be the BLE standard */
  NRF_RADIO->PREFIX0	 = 0x8e;
  NRF_RADIO->BASE0 		 = 0x89bed600; 
  NRF_RADIO->TXADDRESS = 0x00;					// Use logical address 0 (prefix0 + base0) = 0x8E89BED6 when transmitting
	NRF_RADIO->RXADDRESSES = 0x01;				// Enable reception on logical address 0 (PREFIX0 + BASE0)
  
	/* PCNF-> Packet Configuration. Now we need to configure the sizes S0, S1 and length field to match the datapacket format of the advertisement packets. */
  NRF_RADIO->PCNF0 =  (
                          (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)    // length of S0 field in bytes 0-1.
                        | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)    // length of S1 field in bits 0-8.
                        | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)    // length of length field in bits 0-8.
                      );

	/* Packet configuration */
  NRF_RADIO->PCNF1 =  (
                          (((37UL)                      << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)   // maximum length of payload in bytes [0-255]
                        | (((0UL)                       << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk)	// expand the payload with N bytes in addition to LENGTH [0-255]
                        | (((3UL)                       << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)    // base address length in number of bytes.
                        | (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)   // endianess of the S0, LENGTH, S1 and PAYLOAD fields.
                        | (((1UL)                       << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)	// enable packet whitening
                      );

	/* CRC config */
  NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | 
                       (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); // Skip Address when computing crc     
  NRF_RADIO->CRCINIT = 0x555555;    // Initial value of CRC
  NRF_RADIO->CRCPOLY = 0x00065B;    // CRC polynomial function
	
	/* Lock interframe spacing, so that the radio won't send too soon / start RX too early */
	NRF_RADIO->TIFS = 145;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

bool periph_radio_setup(void)
{
	/* Reset all states in the radio peripheral */
	NRF_RADIO->POWER = 1;
	
	NRF_RADIO->EVENTS_DISABLED = 0; 
	
	/* Timer: 1us resolution, disable peripheral */
	NRF_TIMER0->PRESCALER = 4;
	NRF_TIMER0->TASKS_STOP = 1;
	
	/* Do BLE specific radio setup */
	radio_init();
	
	/* Recover packet pointer */
	NRF_RADIO->PACKETPTR = (uint32_t) &packet[0];
	
	/* Enable radio interrupt propagation */
	NVIC_EnableIRQ(RADIO_IRQn);
	
	return true;
}

__INLINE void periph_radio_tifs_set(uint8_t tifs)
{
	NRF_RADIO->TIFS = tifs;
}

__INLINE void periph_radio_ch_set(uint8_t ch)
{
	ASSERT(ch >= 37 && ch <= 39);
	
	NRF_RADIO->FREQUENCY = freq_bins[ch - 37]; 
	NRF_RADIO->DATAWHITEIV = ch;
	DEBUG_PIN_POKE(0);
}


void periph_radio_packet_ptr_set(uint8_t* p_packet)
{
	ASSERT(p_packet != NULL);
	NRF_RADIO->PACKETPTR = (uint32_t) &p_packet[0];
}

__INLINE void periph_radio_shorts_set(uint32_t shorts)
{
	NRF_RADIO->SHORTS = shorts;
}


__INLINE void periph_radio_rssi_read(uint8_t* rssi)
{
	NRF_RADIO->EVENTS_RSSIEND = 0;
	*rssi = NRF_RADIO->RSSISAMPLE;
}

__INLINE void periph_radio_intenset(uint32_t interrupt)
{
	NRF_RADIO->INTENSET = interrupt;
}

__INLINE void periph_radio_intenclr(uint32_t interrupt)
{
	NRF_RADIO->INTENCLR = interrupt;
}

__INLINE void periph_radio_evts_clear(void)
{
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_BCMATCH = 0;
	NRF_RADIO->EVENTS_DEVMATCH = 0;
	NRF_RADIO->EVENTS_DEVMISS = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_PAYLOAD = 0;
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_RSSIEND = 0;
}

__INLINE void periph_radio_channel_get(uint8_t* ch)
{
	*ch = (NRF_RADIO->DATAWHITEIV & 0x3F);
}



__INLINE void periph_ppi_set(uint8_t ppi_ch, volatile uint32_t* task, volatile uint32_t* event)
{
	ASSERT(ppi_ch < 16u);
	
	NRF_PPI->CH[ppi_ch].EEP = (uint32_t) (event);
	NRF_PPI->CH[ppi_ch].TEP = (uint32_t) (task);
	NRF_PPI->CHENSET 	    = (1 << ppi_ch);
}

__INLINE void periph_ppi_clear(uint8_t ppi_ch)
{
	ASSERT(ppi_ch < 16u);
	
	NRF_PPI->CHENCLR = (1 << ppi_ch);
}

__INLINE void periph_gpiote_config(uint8_t gpiote_slot, uint8_t gpio_pin,
																						uint8_t gpiote_config_mode, 
																						uint8_t gpiote_config_polarity,
																						uint8_t gpiote_config_outinit)
{
	ASSERT(gpiote_slot < 4u);
	
	NRF_GPIOTE->CONFIG[gpiote_slot] = 	gpiote_config_mode 			<< GPIOTE_CONFIG_MODE_Pos |
																			gpiote_config_polarity 	<< GPIOTE_CONFIG_POLARITY_Pos |
																			gpio_pin								<< GPIOTE_CONFIG_PSEL_Pos | 
																			gpiote_config_outinit		<< GPIOTE_CONFIG_OUTINIT_Pos;
}

__INLINE void periph_timer_start(uint8_t timer, uint16_t value, bool interrupt)
{	
	ASSERT(timer < 4);
	
	NRF_TIMER0->TASKS_START = 1;
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->EVENTS_COMPARE[timer] = 0; 
	
	/* set interrupt flag */
	if (interrupt)
	{
		NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
		NVIC_EnableIRQ(TIMER0_IRQn);
	}
	
	NRF_TIMER0->CC[timer] = value; /* timeout for RX abort */
}

__INLINE void periph_timer_abort(uint8_t timer)
{
	ASSERT(timer < 4);
	
	NRF_TIMER0->TASKS_STOP = 1;
	NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
}
