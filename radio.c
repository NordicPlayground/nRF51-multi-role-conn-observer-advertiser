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

#include "radio.h"

#include "channel_resolver.h"
#include "ll_scan.h"

#include "nrf_gpio.h"
#include "nrf_soc.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Possible scanner states
 */
static enum
{
  RADIO_DIR_NONE = 0,
  RADIO_DIR_RX,
  RADIO_DIR_TX
} m_radio_dir;

/*****************************************************************************
* Static Globals
*****************************************************************************/

/**@brief Global variables for timeslot requests and return values.
 */

/*****************************************************************************
* Static Functions
*****************************************************************************/

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
  
  /* Enable interrupt on events */
  NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk | RADIO_INTENSET_DISABLED_Msk;
  
  /* Enable RADIO interrupts */
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);
  
  m_radio_dir = RADIO_DIR_NONE;
}

void radio_disable (void)
{
  /* Clear events */
  NRF_RADIO->EVENTS_DISABLED = 0;

  /* Set shorts */
  NRF_RADIO->SHORTS = 0;

  /* Abort TX */
  NRF_RADIO->TASKS_DISABLE = 1;
  
  m_radio_dir = RADIO_DIR_NONE;
}

void radio_buffer_configure (uint8_t * const buff)
{
    NRF_RADIO->PACKETPTR = (uint32_t) buff;
}

void radio_rssi_enable (void)
{
  NRF_RADIO->EVENTS_RSSIEND = 0;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
}

uint8_t radio_rssi_get (void)
{
  uint8_t sample;

  /* First check if sample is available */
  if (NRF_RADIO->EVENTS_RSSIEND != 0)
  {
    sample = (NRF_RADIO->RSSISAMPLE & RADIO_RSSISAMPLE_RSSISAMPLE_Msk);
  }
  else
  {
    sample = RADIO_RSSI_INVALID;
  }
  
  /* Clear event */
  NRF_RADIO->EVENTS_RSSIEND = 0;

  return sample;
}
void radio_rx_prepare (bool start_immediately)
{
  /* Clear events */
  NRF_RADIO->EVENTS_DISABLED = 0;

  /* Enable RX */
  NRF_RADIO->TASKS_RXEN = 1;

  /* Set shorts */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  
  if (start_immediately)
  {
    NRF_RADIO->TIFS = 0;
  }
  else
  {
    NRF_RADIO->TIFS = 149;
  }
  
  m_radio_dir = RADIO_DIR_RX;
}

void radio_rx_timeout_init (void)
{
  /* Capture timer value when radio reaches DISABLED, so that we can
   * time out if we don't get a SCAN_RSP.
   */
  NRF_PPI->CH[4].EEP = (uint32_t) (&NRF_RADIO->EVENTS_DISABLED);
  NRF_PPI->CH[4].TEP = (uint32_t) (&NRF_TIMER0->TASKS_CAPTURE[1]);
  NRF_PPI->CHENSET = PPI_CHENSET_CH4_Msk;
}

/* Set timer to trigger if we don't receive a packet in time */
void radio_rx_timeout_enable (void)
{
  /* Trigger timeout at 200us after radio disabled event.
   */
  NRF_TIMER0->EVENTS_COMPARE[1] = 0;
  NRF_TIMER0->CC[1] += 200;
  NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
  
  /* Capture timer on radio address event.
   */
  NRF_PPI->CH[5].EEP = (uint32_t) (&NRF_RADIO->EVENTS_ADDRESS);
  NRF_PPI->CH[5].TEP = (uint32_t) (&NRF_TIMER0->TASKS_CAPTURE[1]);
  NRF_PPI->CHENSET = PPI_CHENSET_CH5_Msk;
  
  /* Disable radio on timeout.
  */
  NRF_PPI->CH[6].EEP = (uint32_t) (&NRF_TIMER0->EVENTS_COMPARE[1]);
  NRF_PPI->CH[6].TEP = (uint32_t) (&NRF_RADIO->TASKS_DISABLE);
  NRF_PPI->CHENSET = PPI_CHENSET_CH6_Msk;
}

void radio_rx_timeout_disable (void)
{
  NRF_TIMER0->CC[1] = 0;
  NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
  NRF_PPI->CHENCLR = PPI_CHENCLR_CH5_Msk;
  NRF_PPI->CHENCLR = PPI_CHENCLR_CH6_Msk;
}

void radio_tx_mode_on_receipt (void)
{
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
}

void radio_tx_prepare (void)
{
  /* Clear events */
  NRF_RADIO->EVENTS_DISABLED = 0;
  
  /* Enable RX */
  NRF_RADIO->TASKS_TXEN = 1;
  
  /* Set shorts */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
  
  NRF_RADIO->TIFS = 149;  
  
  m_radio_dir = RADIO_DIR_TX;
}

void radio_event_cb (void)
{
  bool crc_valid;
  
  if (NRF_RADIO->EVENTS_DISABLED != 0)
  {
    switch (m_radio_dir)
    {
      case RADIO_DIR_RX:
        /* Disable RSSISTART short so sample isn't overwritten */
        NRF_RADIO->SHORTS &= ~RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
      
        crc_valid = NRF_RADIO->CRCSTATUS != 0;
        ll_scan_rx_cb (crc_valid);
        break;
      case RADIO_DIR_TX:
        ll_scan_tx_cb ();
        break;
      default:
        break;
    }
    NRF_RADIO->EVENTS_DISABLED = 0;
  }
  
  if (NRF_RADIO->EVENTS_ADDRESS != 0)
  {
    if (m_radio_dir == RADIO_DIR_RX)
    {
      radio_rx_timeout_disable ();      
    }
    NRF_RADIO->EVENTS_ADDRESS = 0;
  }
}

void radio_timeout_cb (void)
{
  ll_scan_timeout_cb ();
}
