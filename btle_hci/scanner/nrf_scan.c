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

#include "nrf_scan.h"

#include "btle.h"
#include "ll_scan.h"
#include "radio.h"
#include "nrf_report_disp.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

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

#define softdevice (1)
#define nosoftdevice (0)

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

/*****************************************************************************
* Static Functions
*****************************************************************************/



//	 NVIC_EnableIRQ(TIMER0_IRQn);

//  m_state_idle_exit ();
//  
//  if(channel == 40)
//    channel = 37;
//   NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk | RADIO_INTENSET_DISABLED_Msk;
//  
//  /* Enable RADIO interrupts */
//  NVIC_ClearPendingIRQ(RADIO_IRQn);
//  NVIC_EnableIRQ(RADIO_IRQn);
//  
//  m_radio_dir = RADIO_DIR_NONE;
	
//  radio_init (channel++);
//  radio_rx_timeout_init ();
  
//	packetpointer set to receive
//	NRF_RADIO->TASKS_RXEN = 1;
//  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
//  if (start_immediately)
//  {
//    NRF_RADIO->TIFS = 0;
//  }
// 
//  m_radio_dir = RADIO_DIR_RX;
	
//  radio_rx_prepare (true);
//  radio_rssi_enable (); //NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
 
//  /* Only go directly to TX if we're doing active scanning */
//  if (m_scanner.params.scan_type == BTLE_SCAN_TYPE_ACTIVE)
//  {
//    radio_tx_mode_on_receipt (); //NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
//  }
//  
//  m_scanner.state = SCANNER_STATE_RECEIVE_ADV;
//	
//  m_state_receive_adv_entry ();

    //  NRF_TIMER0-> PRESCALER = 4;
		 
		 //NVIC_SetPriority(RADIO_IRQn, 0);
		 //NVIC_SetPriority(TIMER0_IRQn, 1);
	
#if nosoftdevice	
void ll_scan_start_1 (void)
{		 
		  NRF_TIMER0->TASKS_START = 1;
			NRF_TIMER0->TASKS_CLEAR = 1;
      NRF_TIMER0->EVENTS_COMPARE[0] = 0;
      NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
      NRF_TIMER0->CC[0] = 15000;//m_timeslot_req_normal.params.normal.length_us - 500;
			
      NVIC_EnableIRQ(TIMER0_IRQn);
			
			ll_scan_start ();					
}

void RADIO_IRQHandler(void)
{
   radio_event_cb ();
	
}

void TIMER0_IRQHandler (void)
{
			
   if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
      {
        ll_scan_stop ();
        
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
        NVIC_DisableIRQ(TIMER0_IRQn);
        NRF_TIMER0->TASKS_STOP = 1;
			
				NRF_RADIO->EVENTS_DISABLED = 0;
				/* Set shorts */
				NRF_RADIO->SHORTS = 0;

				/* Abort TX */
				NRF_RADIO->TASKS_DISABLE = 1;
				
				ll_scan_start_1();
				
      }
      
      /* Check the timeout counter */
      if (NRF_TIMER0->EVENTS_COMPARE[1] != 0)
      {
        NRF_TIMER0->EVENTS_COMPARE[1] = 0;
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
        
        radio_timeout_cb ();
			}
}

#endif

/****************************************************************************************/
#if softdevice

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
        ll_scan_stop ();
        
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
        NVIC_DisableIRQ(TIMER0_IRQn);
        
        m_signal_callback_return_param.params.request.p_next = &m_timeslot_req_normal;
        m_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
      }
      
      /* Check the timeout counter */
      if (NRF_TIMER0->EVENTS_COMPARE[1] != 0)
      {
        NRF_TIMER0->EVENTS_COMPARE[1] = 0;
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
        
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

#endif

/****************************************************************************************/

btle_status_codes_t btle_scan_init (IRQn_Type irq)
{
  uint32_t err_code = NRF_SUCCESS;
  btle_status_codes_t status = BTLE_STATUS_CODE_SUCCESS;

  nrf_report_disp_init (irq);
	
#if softdevice  
  err_code = sd_radio_session_open (radio_cb);
  if (err_code != NRF_SUCCESS)
  {
    status = BTLE_STATUS_CODE_COMMAND_DISALLOWED;
  }
#endif
  return status;
}


btle_status_codes_t btle_scan_ev_get (nrf_report_t *p_ev)
{
  btle_status_codes_t status = BTLE_STATUS_CODE_SUCCESS;

  if (nrf_report_disp_get (p_ev) != NRF_SUCCESS)
  {
    status = BTLE_STATUS_CODE_COMMAND_DISALLOWED;
  }
    
  return status;
}

btle_status_codes_t btle_scan_param_set (btle_cmd_param_le_write_scan_parameters_t param)
{
  btle_status_codes_t status;
  
#if softdevice
  m_timeslot_req_earliest.params.earliest.length_us = param.scan_window;
  m_timeslot_req_normal.params.normal.length_us = param.scan_window;
  m_timeslot_req_normal.params.normal.distance_us = param.scan_interval;
#endif
	
  ll_scan_init();
  status = ll_scan_config (param.scan_type, param.own_address_type, param.scanning_filter_policy);
  
  return status;
}

#if softdevice 

btle_status_codes_t btle_scan_enable_set (btle_cmd_param_le_write_scan_enable_t param)
{
  uint32_t err_code = NRF_SUCCESS;
  btle_status_codes_t status = BTLE_STATUS_CODE_SUCCESS;

  switch (param.scan_enable)
  {
    case BTLE_SCAN_MODE_ENABLE:
      err_code = sd_radio_request (&m_timeslot_req_earliest);
      if (err_code != NRF_SUCCESS)
      {
        status = BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }
      break;

    case BTLE_SCAN_MODE_DISABLE:
      err_code = sd_radio_session_close ();
      if (err_code != NRF_SUCCESS)
      {
        status = BTLE_STATUS_CODE_COMMAND_DISALLOWED;
      }
      break;
  }

  return status;
}

#endif