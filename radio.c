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
#include "radio.h"

#include "channel_resolver.h"
#include "ll_scanner.h"

#include "nrf_gpio.h"
#include "nrf_soc.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*****************************************************************************
* Local definitions
*****************************************************************************/

/*****************************************************************************
* Static Globals
*****************************************************************************/

/**@brief Global variables for timeslot requests and return values.
 */

/*****************************************************************************
* Static Functions
*****************************************************************************/

/* Configure COMPARE[1] to trigger START at 150us from previous END */
static void m_tifs_timer (void)
{
  /* Configure transmit enable timer to trigger 150us from now */
  NRF_TIMER0->EVENTS_COMPARE[1] = 0;
  NRF_TIMER0->CC[1] += 150;
  NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk;

  /* Set PPI to trigger transmission start on timer event */
  NRF_PPI->CH[4].EEP = (uint32_t) (&NRF_TIMER0->EVENTS_COMPARE[1]);
  NRF_PPI->CH[4].TEP = (uint32_t) (&NRF_RADIO->TASKS_START);
  NRF_PPI->CHENSET = PPI_CHENSET_CH4_Msk;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

void radio_init (uint8_t channel)
{
  /* Enable power to RADIO */
  NRF_RADIO->POWER = 1;

  /* Set radio transmit power to 0dBm */
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Set radio mode to 1Mbit/s Bluetooth Low Energy */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

  /* Set the requested channel */
  NRF_RADIO->FREQUENCY = channel_resolver_get_frequency(channel);

  /* This value needs to correspond to the channel being used */
  NRF_RADIO->DATAWHITEIV = channel;

  /* Configure Access Address according to the BLE standard */
  NRF_RADIO->PREFIX0 = 0x8e;
  NRF_RADIO->BASE0 = 0x89bed600;
  
  /* Use logical address 0 (prefix0 + base0) = 0x8E89BED6 when transmitting and receiving */
  NRF_RADIO->TXADDRESS = 0x00;
  NRF_RADIO->RXADDRESSES = 0x01;

  /* PCNF-> Packet Configuration. 
   * We now need to configure the sizes S0, S1 and length field to match the
   * datapacket format of the advertisement packets.
   */
  NRF_RADIO->PCNF0 = (
    (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) |  /* Length of S0 field in bytes 0-1.    */
    (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) |  /* Length of S1 field in bits 0-8.     */
    (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)    /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = (
    (((37UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk)   |                      /* Maximum length of payload in bytes [0-255] */
    (((0UL) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk)   |                      /* Expand the payload with N bytes in addition to LENGTH [0-255] */
    (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk)       |                      /* Base address length in number of bytes. */
    (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) |  /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
    (((1UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                         /* Enable packet whitening */
  );

  /* CRC config */
  NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                       (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); /* Skip Address when computing CRC */
  NRF_RADIO->CRCINIT = 0x555555;                                                  /* Initial value of CRC */
  NRF_RADIO->CRCPOLY = 0x00065B;                                                  /* CRC polynomial function */

  /* Clear events */
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_ADDRESS = 0;
  
  /* Enable interrupt on DISABLED event */
  NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
  
  /* Enable RADIO interrupts */
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);
}

void radio_receive_prepare_and_start (uint8_t *buff, bool prepare_tx)
{
  m_tifs_timer ();
  
  memset (buff, 0, 256);

  /* Set receive buffer */
  NRF_RADIO->PACKETPTR = (uint32_t) &buff[0];

  /* Clear events */
  NRF_RADIO->EVENTS_DISABLED = 0;

  /* Enable RX */
  NRF_RADIO->TASKS_RXEN = 1;

  /* Set shorts */
  NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;
  
  if (prepare_tx)
  {
    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
  }
}

void radio_transmit_prepare (uint8_t *buff)
{
  m_tifs_timer ();
  
  /* Set transmit buffer */
  NRF_RADIO->PACKETPTR = (uint32_t) &buff[0];

  /* Clear events */
  NRF_RADIO->EVENTS_DISABLED = 0;
  
  /* Enable RX */
  NRF_RADIO->TASKS_TXEN = 1;
  
  /* Set shorts */
  NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
}

void radio_disable (void)
{
  /* Clear events */
  NRF_RADIO->EVENTS_DISABLED = 0;

  /* Set shorts */
  NRF_RADIO->SHORTS = 0;

  /* Abort TX */
  NRF_RADIO->TASKS_DISABLE = 1;
}
