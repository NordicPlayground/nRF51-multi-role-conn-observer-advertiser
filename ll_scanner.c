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

#include "ll_scanner.h"

#include "btle.h"
#include "radio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Defining GPIO pins used for looking at timing.
 */

#define DBG_RADIO_END                        0
#define DBG_RADIO_READY                      1
#define DBG_RADIO_TIMER                      2

/**@brief Possible scanner states
 */
typedef enum
{
  SCANNER_STATE_NOT_INITIALIZED = 0,
  SCANNER_STATE_INITIALIZED,
  SCANNER_STATE_IDLE,
  SCANNER_STATE_RECEIVE_ADV,
  SCANNER_STATE_SEND_REQ,
  SCANNER_STATE_RECEIVE_SCAN_RSP
} m_state_t;

/**@brief Possible packet types
 */
typedef enum
{
  PACKET_TYPE_ADV_IND = 0x00,
  PACKET_TYPE_ADV_DIRECT_IND = 0x01,
  PACKET_TYPE_ADV_NONCONN_IND = 0x02,
  PACKET_TYPE_CONNECT_REQ = 0x03,
  PACKET_TYPE_SCAN_RSP = 0x04,
  PACKET_TYPE_SCAN_REQ = 0x05,
  PACKET_TYPE_ADV_SCAN_IND = 0x06
} m_packet_type_t;

typedef struct
{
  btle_scan_types_t scan_type;
  btle_address_type_t own_address_type;
  btle_scan_filter_policy_t scanning_filter_policy;
} scanner_params_t;

static struct
{
  scanner_params_t params;
  m_state_t state;
} m_scanner;
  

/*****************************************************************************
* Static Globals
*****************************************************************************/


static uint8_t m_rssi;
static bool m_rssi_valid;

static uint8_t m_rx_buf[255];
static uint8_t m_tx_buf[] =
{
  0xC3,                               // BLE Header (PDU_TYPE: SCAN_REQ, TXadd: 1 (random address), RXadd: 1 (random address)
  0x0C,                               // Length of payload: 12
  0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
  0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, // InitAddr LSByte first
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // AdvAddr LSByte first
};


/*****************************************************************************
* Static Function prototypes
*****************************************************************************/

/**@brief Generate report */
static void m_adv_report_generate (uint8_t * const pkt);

/**@brief Entry function for SCANNER_STATE_INIT */
static void m_state_init_entry (void);

/**@brief Exit function for SCANNER_STATE_INIT */
static void m_state_init_exit (void);

/**@brief Entry function for SCANNER_STATE_IDLE */
static void m_state_idle_entry (void);

/**@brief Exit function for SCANNER_STATE_IDLE */
static void m_state_idle_exit (void);

/**@brief Entry function for SCANNER_STATE_RECEIVE_ADV */
static void m_state_receive_adv_entry (void);

/**@brief Exit function for SCANNER_STATE_RECEIVE_ADV */
static void m_state_receive_adv_exit (void);

/**@brief Entry function for SCANNER_STATE_SEND_REQ */
static void m_state_send_scan_req_entry (void);

/**@brief Exit function for SCANNER_STATE_SEND_REQ */
static void m_state_send_scan_req_exit (void);

/**@brief Entry function for SCANNER_STATE_RECEIVE_SCAN_RSP */
static void m_state_receive_scan_rsp_entry (void);

/**@brief Exit function for SCANNER_STATE_RECEIVE_SCAN_RSP */
static void m_state_receive_scan_rsp_exit (void);

/*****************************************************************************
* Static Function definitions
*****************************************************************************/

static void m_adv_report_generate (uint8_t * const pkt)
{
  /* TODO */
  return;
}

static void m_state_init_entry (void)
{
  m_scanner.state = SCANNER_STATE_INITIALIZED;
}

static void m_state_init_exit (void)
{
  /* Nothing to do */
}

static void m_state_idle_entry (void)
{
  m_scanner.state = SCANNER_STATE_IDLE;
}

static void m_state_idle_exit (void)
{
  /* Nothing to do */
}

static void m_state_receive_adv_entry (void)
{
  radio_buffer_configure (&m_rx_buf[0]);
  radio_rx_prepare (true);
  radio_rssi_enable ();
 
  /* Only go directly to TX if we're doing active scanning */
  if (m_scanner.params.scan_type == BTLE_SCAN_TYPE_ACTIVE)
  {
    radio_tx_mode_on_receipt ();
  }
  
  m_scanner.state = SCANNER_STATE_RECEIVE_ADV;
}

static void m_state_receive_adv_exit (void)
{
  m_rssi_valid = radio_rssi_get (&m_rssi);
  radio_rssi_disable ();
}

static void m_state_send_scan_req_entry (void)
{
  memcpy(&m_tx_buf[9], &m_rx_buf[3], 6);
  radio_buffer_configure (&m_tx_buf[0]);
  radio_tx_prepare (false);
  
  m_scanner.state = SCANNER_STATE_SEND_REQ;
}

static void m_state_send_scan_req_exit (void)
{
  /* Nothing to do */
}

static void m_state_receive_scan_rsp_entry (void)
{
  radio_buffer_configure (&m_rx_buf[0]);
  radio_rx_prepare (false);
  radio_rssi_enable ();
  
  m_scanner.state = SCANNER_STATE_RECEIVE_SCAN_RSP;
}

static void m_state_receive_scan_rsp_exit (void)
{
  m_rssi_valid = radio_rssi_get (&m_rssi);
  radio_rssi_disable ();
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

void ll_scan_radio_cb (bool crc_valid)
{
  /* Received invalid packet */
  if (!crc_valid)
  {
    switch(m_scanner.state)
    {
      case SCANNER_STATE_RECEIVE_ADV:
        m_state_receive_adv_exit ();
        radio_disable ();
        m_state_receive_adv_entry ();
        break;

      case SCANNER_STATE_RECEIVE_SCAN_RSP:
        m_state_receive_scan_rsp_exit ();
        radio_disable ();
        m_state_receive_adv_entry ();
        break;

      default:
        break;
    }
  }
  
  switch (m_scanner.state)
  {
    /* Packet received */
    case SCANNER_STATE_RECEIVE_ADV:
      switch (m_rx_buf[0] & 0x0F)
      {
        /* If active scanning is enabled, these packets should be reponded to with
         * a SCAN_REQ, and we should wait for a SCAN_RSP.
         */
        case PACKET_TYPE_ADV_IND:
        case PACKET_TYPE_ADV_SCAN_IND:
          m_state_receive_adv_exit ();
        
          m_adv_report_generate (m_rx_buf);

          /* If we're doing active scanning, prepare to send SCAN REQ, otherwise
           * loop back around to receive a new advertisement.
           */
          m_state_receive_adv_exit ();

          if (m_scanner.params.scan_type == BTLE_SCAN_TYPE_ACTIVE)
          {
            m_state_send_scan_req_entry ();
          }
          else
          {
            m_state_receive_adv_entry ();
          }
          break;

        /* These packets do not require response.
         */
        case PACKET_TYPE_ADV_DIRECT_IND:
        case PACKET_TYPE_ADV_NONCONN_IND:
          m_state_receive_adv_exit ();
          radio_disable();
          m_adv_report_generate (m_rx_buf);
          m_state_receive_adv_entry ();
          break;

        /* This should not have happened */
        default:
          m_state_receive_adv_exit ();
          radio_disable();
          m_state_receive_adv_entry();
      }
      break;

    /* SCAN_REQ has been transmitted, and we must configure the radio to
     * listen for the incoming SCAN_RSP.
     */
    case SCANNER_STATE_SEND_REQ:
      m_state_send_scan_req_exit ();
      m_state_receive_scan_rsp_entry ();
      break;

    case SCANNER_STATE_RECEIVE_SCAN_RSP:
      m_state_receive_scan_rsp_exit ();
      m_adv_report_generate (m_rx_buf);
      m_state_receive_adv_entry ();
      break;

    default:
      break;
  }
}

void ll_scan_timer_cb (void)
{
  switch (m_scanner.state)
  {
    case SCANNER_STATE_RECEIVE_ADV:
      break;

    case SCANNER_STATE_SEND_REQ:
      break;

    case SCANNER_STATE_RECEIVE_SCAN_RSP:
      break;

    default:
      break;
  }
}

btle_status_codes_t ll_scan_init (void)
{
  m_scanner.state = SCANNER_STATE_NOT_INITIALIZED;
  
  m_state_init_entry ();
  ll_scan_reset ();
  
  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t ll_scan_reset (void)
{
  btle_status_codes_t status;
  
  if (m_scanner.state == SCANNER_STATE_IDLE || m_scanner.state == SCANNER_STATE_INITIALIZED)
  {
    status = BTLE_STATUS_CODE_SUCCESS;
  }
  else
  {
    status = BTLE_STATUS_CODE_COMMAND_DISALLOWED;
  }
  
  return status;
}

btle_status_codes_t ll_scan_config (btle_scan_types_t scan_type, btle_address_type_t address_type, btle_scan_filter_policy_t filter_policy)
{
  /* Scanner can only be configured when not running */
  if ((m_scanner.state != SCANNER_STATE_INITIALIZED) && (m_scanner.state != SCANNER_STATE_IDLE))
  {
    return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
  }
  
  m_scanner.params.scan_type = scan_type;
  m_scanner.params.own_address_type = address_type;
  m_scanner.params.scanning_filter_policy = filter_policy;
  
  if (m_scanner.state == SCANNER_STATE_INITIALIZED)
  {
    m_state_init_exit ();
    m_state_idle_entry ();
  }
  
  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t ll_scan_start (void)
{ 
  /* Capture timer value when radio reaches END, so that we can call
   * START after 150us.
   */
  NRF_PPI->CH[5].EEP = (uint32_t) (&NRF_RADIO->EVENTS_END);
  NRF_PPI->CH[5].TEP = (uint32_t) (&NRF_TIMER0->TASKS_CAPTURE[1]);
  NRF_PPI->CHENSET = PPI_CHENSET_CH5_Msk;

  /* Toggle pin when radio reaches END (RX or TX) */
  NRF_GPIOTE->CONFIG[DBG_RADIO_END] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                          GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                          DBG_RADIO_END << GPIOTE_CONFIG_PSEL_Pos |
                          GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;

  NRF_PPI->CH[DBG_RADIO_END].EEP = (uint32_t) (&NRF_RADIO->EVENTS_END);
  NRF_PPI->CH[DBG_RADIO_END].TEP = (uint32_t) (&NRF_GPIOTE->TASKS_OUT[DBG_RADIO_END]);
  NRF_PPI->CHENSET = (1 << DBG_RADIO_END);

  /* Toggle pin when radio reaches READY (RX or TX) */
  NRF_GPIOTE->CONFIG[DBG_RADIO_READY] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                          GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                          DBG_RADIO_READY << GPIOTE_CONFIG_PSEL_Pos |
                          GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;

  NRF_PPI->CH[DBG_RADIO_READY].EEP = (uint32_t) (&NRF_RADIO->EVENTS_READY);
  NRF_PPI->CH[DBG_RADIO_READY].TEP = (uint32_t) (&NRF_GPIOTE->TASKS_OUT[DBG_RADIO_READY]);
  NRF_PPI->CHENSET = (1 << DBG_RADIO_READY);

  /* Toggle pin when timer triggers radio START (TX) */
  NRF_GPIOTE->CONFIG[DBG_RADIO_TIMER] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                          GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                          DBG_RADIO_TIMER << GPIOTE_CONFIG_PSEL_Pos |
                          GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;

  NRF_PPI->CH[DBG_RADIO_TIMER].EEP = (uint32_t) (&(NRF_TIMER0->EVENTS_COMPARE[1]));
  NRF_PPI->CH[DBG_RADIO_TIMER].TEP = (uint32_t) (&NRF_GPIOTE->TASKS_OUT[DBG_RADIO_TIMER]);
  NRF_PPI->CHENSET = (1 << DBG_RADIO_TIMER);

  NVIC_EnableIRQ(TIMER0_IRQn);

  m_state_idle_exit ();
  m_state_receive_adv_entry ();

  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t ll_scan_stop (void)
{
  return BTLE_STATUS_CODE_SUCCESS;
}
