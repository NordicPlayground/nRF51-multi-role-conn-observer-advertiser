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

#include "scanner.h"

#include "btle.h"
#include "ll_scanner.h"
#include "radio.h"

#include "nrf_soc.h"
#include "nrf_gpio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Defines for Timeslot parameters
 */
#define TIMESLOT_TIMEOUT_US 10000
#define TIMESLOT_RNG_LEN 10

/*****************************************************************************
* Static Globals
*****************************************************************************/

static nrf_radio_signal_callback_return_param_t m_signal_callback_return_param;
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

/*****************************************************************************
* Static Functions
*****************************************************************************/

#if 0

static uint32_t m_btle_address_type_get (btle_address_type_t *type, uint8_t *packet);
static uint32_t m_btle_event_type_get (btle_report_event_type_t *type, uint8_t *packet);
static uint32_t m_btle_report_generate (btle_ev_param_le_advertising_report_t *report, uint8_t *packet);

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

#endif

nrf_radio_signal_callback_return_param_t *radio_cb (uint8_t sig)
{
  switch (sig)
  {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
      /* TIMER0 setup */
      NRF_TIMER0->TASKS_CLEAR = 1;
      NRF_TIMER0->EVENTS_COMPARE[0] = 0;
      NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
      NRF_TIMER0->CC[0] = m_timeslot_req_normal.params.normal.length_us - 500;  

      ll_scan_start ();

      m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
      break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
      radio_event_cb ();
      break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
      /* Check the timeslot cleanup counter */
      if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
      {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
        NVIC_DisableIRQ(TIMER0_IRQn);
        
        m_signal_callback_return_param.params.request.p_next = &m_timeslot_req_normal;
        m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
      }

      /* Check the T_IFS counter */
      if (NRF_TIMER0->EVENTS_COMPARE[1] != 0)
      {
        NRF_PPI->CHENCLR = PPI_CHENCLR_CH4_Msk;
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
        
        radio_tifs_cb ();
      }
      
      /* Check the timeout counter */
      if (NRF_TIMER0->EVENTS_COMPARE[2] != 0)
      {
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE2_Msk;
        
        radio_timeout_cb ();
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
  btle_status_codes_t status;
  
  m_timeslot_req_earliest.params.earliest.length_us = param.scan_window;
  m_timeslot_req_normal.params.normal.length_us = param.scan_window;
  m_timeslot_req_normal.params.normal.distance_us = param.scan_interval;
  
  ll_scan_init();
  status = ll_scan_config (param.scan_type, param.own_address_type, param.scanning_filter_policy);
  
  return status;
}

btle_status_codes_t btle_scan_enable_set (btle_cmd_param_le_write_scan_enable_t param)
{
  uint32_t err_code = NRF_SUCCESS;

  switch (param.scan_enable)
  {
    case BTLE_SCAN_MODE_ENABLE:
      err_code = sd_radio_session_open (radio_cb);
      if (err_code != NRF_SUCCESS)
      {
        return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }

      err_code = sd_radio_request (&m_timeslot_req_earliest);
      if (err_code != NRF_SUCCESS)
      {
        return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }
      break;

    case BTLE_SCAN_MODE_DISABLE:
      err_code = sd_radio_session_close ();
      if (err_code != NRF_SUCCESS)
      {
        return BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }
      break;
  }

  return BTLE_STATUS_CODE_SUCCESS;
}
