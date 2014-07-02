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

#include "btle.h"
#include "radio.h"

#include "nrf_soc.h"
#include "nrf_gpio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Defining GPIO pins used for looking at timing in the example.
 * For the PCA10001 board (Nordic Evaluation Kit) the pins matches
 * PO.00 - PO.07 and P0.12-P0.15 (PO.08-11 is the UART).
 * For the PCA10005 (Nordic Development Kit) the pins matches P1 1-8 and
 * PO.16-23 and PO.28-31 (PO.24-27 is the UART).
 */

#define DBG_RADIO_END                        0
#define DBG_RADIO_READY                      1
#define DBG_RADIO_TIMER                      2

/**@brief Defines for Timeslot parameters
 */
#define TIMESLOT_TIMEOUT_US 10000
#define TIMESLOT_RNG_LEN 10

/**@brief Defines for packet types
 */
#define PACKET_TYPE_ADV_IND           0x00
#define PACKET_TYPE_ADV_DIRECT_IND    0x08
#define PACKET_TYPE_ADV_SCAN_IND      0x06
#define PACKET_TYPE_ADV_NONCONN_IND   0x02
#define PACKET_TYPE_SCAN_RSP          0x04

/*****************************************************************************
* Static Globals
*****************************************************************************/

static btle_cmd_param_le_write_scan_parameters_t m_scan_param;
static scanner_state_t m_scanner_state;
static uint8_t m_rx_buf[255];
static uint8_t m_tx_buf[] =
{
  0xC3,                               // BLE Header (PDU_TYPE: SCAN_REQ, TXadd: 1 (random address), RXadd: 1 (random address)
  0x0C,                               // Length of payload: 12
  0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
  0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, // InitAddr LSByte first
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // AdvAddr LSByte first
};

static nrf_radio_request_t m_timeslot_req_earliest = {
  NRF_RADIO_REQ_TYPE_EARLIEST,
  .params.earliest = {
    NRF_RADIO_HFCLK_CFG_DEFAULT,
    NRF_RADIO_PRIORITY_NORMAL,
    0,
    TIMESLOT_TIMEOUT_US
  }
};

static nrf_radio_request_t m_timeslot_req_normal = {
  NRF_RADIO_REQ_TYPE_NORMAL,
  .params.normal = {
    NRF_RADIO_HFCLK_CFG_DEFAULT,
    NRF_RADIO_PRIORITY_NORMAL,
    0,
    0
  }
};

static btle_ev_param_le_advertising_report_t m_adv_report;

/**@brief Global variables for timeslot requests and return values.
 */

static nrf_radio_signal_callback_return_param_t m_signal_callback_return_param;
static uint8_t timeslot_rng_pool[TIMESLOT_RNG_LEN];
static uint8_t timeslot_rng_pool_index;

/*****************************************************************************
* Static Functions
*****************************************************************************/

static uint32_t m_btle_address_type_get (btle_address_type_t *type, uint8_t *packet);
static uint32_t m_btle_event_type_get (btle_report_event_type_t *type, uint8_t *packet);
static uint32_t m_btle_report_generate (btle_ev_param_le_advertising_report_t *report, uint8_t *packet);

static nrf_radio_signal_callback_return_param_t *scanner_event_cb (uint8_t sig);

static uint32_t m_btle_event_type_get (btle_report_event_type_t *type, uint8_t *packet)
{
  switch (packet[0] & 0x0F)
  {
    case PACKET_TYPE_ADV_IND:
      *type = BTLE_REPORT_TYPE_ADV_IND;
      break;
    case PACKET_TYPE_ADV_DIRECT_IND:
      *type = BTLE_REPORT_TYPE_ADV_IND;
      break;
    case PACKET_TYPE_ADV_SCAN_IND:
      *type = BTLE_REPORT_TYPE_ADV_IND;
      break;
    case PACKET_TYPE_ADV_NONCONN_IND:
      *type = BTLE_REPORT_TYPE_ADV_IND;
      break;
    case PACKET_TYPE_SCAN_RSP:
      *type = BTLE_REPORT_TYPE_ADV_IND;
      break;
    default:
      return NRF_ERROR_INVALID_PARAM;
  }

  return NRF_SUCCESS;
}

static uint32_t m_btle_address_type_get (btle_address_type_t *type, uint8_t *packet)
{
  *type = (btle_address_type_t) ((packet[0] & 0x40) >> 7) ? BTLE_ADDR_TYPE_RANDOM : BTLE_ADDR_TYPE_PUBLIC;
  
  return NRF_SUCCESS;
}

static uint32_t m_btle_report_generate (btle_ev_param_le_advertising_report_t *report, uint8_t *packet)
{
  report->num_reports = 1;
  m_btle_event_type_get (&report->event_type, packet);
  m_btle_address_type_get (&report->address_type, packet);
  memcpy(report->address, &packet[3], BTLE_DEVICE_ADDRESS__SIZE);
  report->length_data = packet[1];

  /* Only copy advertising data for the packet types that have this. */
  if (report->event_type == PACKET_TYPE_ADV_IND ||
      report->event_type == PACKET_TYPE_ADV_SCAN_IND ||
      report->event_type == PACKET_TYPE_ADV_NONCONN_IND ||
      report->event_type == PACKET_TYPE_SCAN_RSP)
  {
    memcpy(report->report_data, &packet[9], BTLE_ADVERTISING_DATA__SIZE);
  }

  report->rssi = 0;
  
  return NRF_SUCCESS;
}

static nrf_radio_signal_callback_return_param_t *scanner_event_cb (uint8_t sig)
{
  uint8_t rand_byte;

  switch (sig)
  {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
      m_scanner_state = SCANNER_STATE_WAIT_TO_SCAN_ADV;

      /* TIMER0 setup */
      NRF_TIMER0->TASKS_CLEAR = 1;
      NRF_TIMER0->EVENTS_COMPARE[0] = 0;
      NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
      NRF_TIMER0->CC[0] = m_scan_param.scan_window - 500;
    
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

      radio_init(39);
      radio_receive_prepare_and_start (m_rx_buf, true);
      
      NVIC_EnableIRQ(TIMER0_IRQn);
      
      m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
      break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
      if (NRF_RADIO->EVENTS_DISABLED != 0)
      {
          switch (m_scanner_state)
          {
            /* Packet received */
            case SCANNER_STATE_SCAN_ADV:
              /* Abort immediately if packet has invalid CRC */
              if (NRF_RADIO->CRCSTATUS == 0)
              {
                radio_transmit_abort();
                break;
              }

              switch (m_rx_buf[0] & 0x0F)
              {
                /* If active scanning is enabled, these packets should be reponded to with
                 * a SCAN_REQ, and we should wait for a SCAN_RSP.
                 */
                case PACKET_TYPE_ADV_IND:
                case PACKET_TYPE_ADV_SCAN_IND:
                  /* TODO: Send SCAN_REQ for ADV_IND and ADV_SCAN_IND only if this is enabled. */
                  /* Prepare to send SCAN_REQ. */
                  memcpy(&m_tx_buf[9], &m_rx_buf[3], 6);
                  radio_transmit_prepare (m_tx_buf);
                  m_scanner_state = SCANNER_STATE_WAIT_TO_SEND_REQ;
                  m_btle_report_generate (&m_adv_report, m_rx_buf);
                  NVIC_SetPendingIRQ (SWI0_IRQn);
                  break;

                /* These packets do not require response. All we do here is generate an
                 * advertisement report and signal the application.
                 */
                case PACKET_TYPE_ADV_DIRECT_IND:
                case PACKET_TYPE_ADV_NONCONN_IND:
                  radio_transmit_abort();
                  m_btle_report_generate (&m_adv_report, m_rx_buf);
                  NVIC_SetPendingIRQ (SWI0_IRQn);
                  break;

                /* SCAN_RSP has been received and we should simply resume scanning */
                case PACKET_TYPE_SCAN_RSP:
                  /* Received SCAN_RSP */
                  radio_receive_prepare_and_start (m_rx_buf, false);
                  m_btle_report_generate (&m_adv_report, m_rx_buf);
                  NVIC_SetPendingIRQ (SWI0_IRQn);
                  m_scanner_state = SCANNER_STATE_WAIT_TO_SCAN_ADV;

                  break;

                default:
                  radio_transmit_abort();
              }
              break;

            /* SCAN_REQ has been transmitted, and we must configure the radio to
             * listen for the incoming SCAN_RSP.
             */
            case SCANNER_STATE_SEND_REQ:
              radio_receive_prepare_and_start (m_rx_buf, false);
              m_btle_report_generate (&m_adv_report, m_rx_buf);
              NVIC_SetPendingIRQ (SWI0_IRQn);
              m_scanner_state = SCANNER_STATE_WAIT_TO_SCAN_RSP;
              break;

            default:
              break;
          }
        NRF_RADIO->EVENTS_DISABLED = 0;
      }
      break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
      /* Check the timeslot cleanup counter */
      if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
      {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
        NVIC_DisableIRQ(TIMER0_IRQn);
        
        rand_byte = timeslot_rng_pool[timeslot_rng_pool_index++];
        m_timeslot_req_normal.params.normal.distance_us += rand_byte;
        m_signal_callback_return_param.params.request.p_next = &m_timeslot_req_normal;
        m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
      }
      
      /* Check the T_IFS counter */
      if (NRF_TIMER0->EVENTS_COMPARE[1] != 0)
      {
        switch (m_scanner_state)
        {
          case SCANNER_STATE_WAIT_TO_SCAN_ADV:
            m_scanner_state = SCANNER_STATE_SCAN_ADV;
            NRF_PPI->CHENCLR = PPI_CHENCLR_CH4_Msk;
            NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
            break;
          
          case SCANNER_STATE_WAIT_TO_SEND_REQ:
            m_scanner_state = SCANNER_STATE_SEND_REQ;
            NRF_PPI->CHENCLR = PPI_CHENCLR_CH4_Msk;
            NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
            break;
          
          case SCANNER_STATE_WAIT_TO_SCAN_RSP:
            m_scanner_state = SCANNER_STATE_SCAN_RSP;
            NRF_PPI->CHENCLR = PPI_CHENCLR_CH4_Msk;
            NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
            break;

          default:
            break;
        }
      }
      break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
      break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
      break;
  }
  return &m_signal_callback_return_param;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

btle_status_codes_t btle_scan_ev_get (btle_event_t *p_ev)
{
  p_ev->event_code = BTLE_EVENT_LE_ADVERTISING_REPORT;
  p_ev->opcode = BTLE_CMD_LE_WRITE_SCAN_ENABLE;
  p_ev->params.le_advertising_report_event = m_adv_report;

  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t btle_scan_param_set (btle_cmd_param_le_write_scan_parameters_t param)
{
  uint32_t err_code;

  switch (m_scanner_state)
  {
    case SCANNER_STATE_IDLE:
      m_scanner_state = SCANNER_STATE_READY;

      err_code = sd_rand_application_vector_get(&timeslot_rng_pool[0], TIMESLOT_RNG_LEN);
      if (err_code != NRF_SUCCESS)
      {
        m_scanner_state = SCANNER_STATE_INIT_ERROR;
        return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }
      /* Fall-through */

    case SCANNER_STATE_READY:
      m_scan_param = param;

      m_timeslot_req_earliest.params.earliest.length_us = m_scan_param.scan_window;
      m_timeslot_req_normal.params.normal.length_us = m_scan_param.scan_window;
      m_timeslot_req_normal.params.normal.distance_us = m_scan_param.scan_interval;
      break;

    default:
      return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
  }

  return BTLE_STATUS_CODE_SUCCESS;
}

btle_status_codes_t btle_scan_enable_set (btle_cmd_param_le_write_scan_enable_t param)
{
  uint32_t err_code = NRF_SUCCESS;

  switch (param.scan_enable)
  {
    case BTLE_SCAN_MODE_ENABLE:
      err_code = sd_radio_session_open (scanner_event_cb);
      if (err_code != NRF_SUCCESS)
      {
        return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }

      err_code = sd_radio_request (&m_timeslot_req_earliest);
      if (err_code != NRF_SUCCESS)
      {
        return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }

      m_scanner_state = SCANNER_STATE_READY;
      break;

    case BTLE_SCAN_MODE_DISABLE:
      err_code = sd_radio_session_close ();
      if (err_code != NRF_SUCCESS)
      {
        return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }

      m_scanner_state = SCANNER_STATE_IDLE;
      break;
  }

  return BTLE_STATUS_CODE_SUCCESS;
}
