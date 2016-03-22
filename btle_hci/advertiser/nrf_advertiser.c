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

#include "nrf_advertiser.h"

#include "ts_controller.h"
#include "ts_whitelist.h"
#include "ts_peripheral.h"
#include "nrf_report_disp.h"

#include "boards.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf51.h"
#include "nrf.h"
#include "ble.h"
#include "nrf_assert.h"
#include "app_error.h"	

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/*****************************************************************************
* Static Functions
*****************************************************************************/

//#define softdevice (1)
//#define nosoftdevice (0)

/**
* Callback for timeslot signals. Is only called when in a timeslot.
* All radio events during timeslot is redirected here
*/

/*****************************************************************************************/
//#if softdevice
 #if defined(WITH_SOFTDEVICE)
static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{	
	 nrf_gpio_pin_toggle(17);
	
	g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;		
	
	
	
	// Send signal to the state machine, and let it decide how the event affects the flow 
	ctrl_signal_handler(sig);
	
	
	// indicating that the timeslot ended 
	if ((NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END 	
				== g_signal_callback_return_param.callback_action) || (NRF_RADIO_SIGNAL_CALLBACK_ACTION_END 	
				== g_signal_callback_return_param.callback_action)|| (NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND	
				== g_signal_callback_return_param.callback_action))
	{
		 nrf_gpio_pin_toggle(18);
	}
	
	DEBUG_PIN_CLEAR(4);
	
	return &g_signal_callback_return_param;
}

/**
* callback to handle timeslot specific events. Should never be called, 
* as the advertiser never is to go idle or shut down 
*/


void btle_hci_adv_sd_evt_handler(uint32_t event)
{
	DEBUG_PIN_SET(15);
	switch (event)
	{
		case NRF_EVT_RADIO_SESSION_IDLE:
			// Means the user stopped the advertiser. Do nothing. 
			DEBUG_PIN_POKE(14);	
     //  nrf_gpio_pin_toggle(24);		
			break;
		case NRF_EVT_RADIO_SESSION_CLOSED:
			// session close accepted, lets just sleep forever.
			// shouldn't really happen.
	//	 nrf_gpio_pin_toggle(24);
			ASSERT(false);
		
		case NRF_EVT_RADIO_BLOCKED:
			// The request was blocked by a softdevice event. 
			// Attempt to reschedule the timeslot for as soon as possible 
			DEBUG_PIN_POKE(0);
	//	nrf_gpio_pin_toggle(24);
		  new_timeslot_order();
			
			break;
		
		case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
			DEBUG_PIN_POKE(2);
			break;
		
		case NRF_EVT_RADIO_CANCELED:
			// The softdevice decided to cancel an ongoing timeslot. 
			//Attempt to reschedule the timeslot for as soon as possible 
			DEBUG_PIN_POKE(5);
			//ctrl_timeslot_order();
	//	nrf_gpio_pin_toggle(24);
		  new_timeslot_order();
		
			break;
		
		default:
			// Invalid event type 
			ASSERT(false);
			break;
	}
	DEBUG_PIN_CLEAR(15);
}

#endif
/*****************************************************************************************/

	
	
void btle_hci_adv_init(IRQn_Type btle_hci_adv_evt_irq)
{
	//#if softdevice	
	 #if defined(WITH_SOFTDEVICE)
	ASSERT(btle_hci_adv_evt_irq >= SWI0_IRQn && btle_hci_adv_evt_irq <= SWI5_IRQn);
	
	uint8_t error_code;	
	#endif
	
	/* init event dispatcher */
	nrf_report_disp_init(btle_hci_adv_evt_irq);
	
	/* init controller layer */
	ctrl_init();
	
//#if softdevice	
	 #if defined(WITH_SOFTDEVICE)
	error_code = sd_radio_session_open(&radio_signal_callback);
	APP_ERROR_CHECK(error_code);
#endif
}


bool btle_hci_adv_report_get(nrf_report_t* evt)
{
	return (NRF_SUCCESS == nrf_report_disp_get(evt));
}

void btle_hci_adv_params_set(btle_cmd_param_le_write_advertising_parameters_t* adv_params)
{
	ctrl_adv_param_set(adv_params);
}

void btle_hci_data_params_set(btle_data_channel_data_packet_parameters_t* data_params)
{
	ctrl_data_param_set(data_params);
}

void btle_hci_data_data_payload_set(btle_data_channel_data_packet_data_t* data_payload)
{
  ctrl_data_datapayload_set(data_payload);
}

void btle_hci_version_data_set(btle_control_packet_version_data_t* version_data)
{
	
ctrl_data_version_data_set(version_data);

}

void btle_hci_feature_rsp_data_set(btle_control_packet_feature_rsp_data_t* feature_rsp_data)

{
 
ctrl_data_feature_rsp_data_set(feature_rsp_data);

}

void btle_hci_disconnect_connection_event_set(uint16_t disconnect, bool enable)
{

	ctrl_disconnect_connection_event_set(disconnect, enable) ;

}
//****************************
//#if nosoftdevice
#if defined(NO_SOFTDEVICE)
void radio_1(void)
{
  radio_2();
}

void adv_1 (void)
	{
	  adv_2 ();
		
	}
#endif
	
//*************************************

//#if softdevice
 #if defined(WITH_SOFTDEVICE)	
void btle_hci_adv_enable(btle_adv_mode_t adv_enable)
{
	if (BTLE_ADV_ENABLE == adv_enable)
	{
		// send first timeslot request to get the session started: 
		ctrl_timeslot_order_first();
	}
	else if (BTLE_ADV_DISABLE == adv_enable)
	{
		// stop advertisement after next advertisement event 
		ctrl_timeslot_abort();
	}
	else
	{
		// invalid parameter 
		ASSERT(false);
	}
}
#endif


void btle_hci_adv_data_set(btle_cmd_param_le_write_advertising_data_t* adv_data)
{
	ctrl_adv_data_set(adv_data);
}

void btle_hci_adv_scan_rsp_data_set(btle_cmd_param_le_write_scan_response_data_t* scan_rsp)
{
	ctrl_scan_data_set(scan_rsp);
}




//#if softdevice 
#if defined(WITH_SOFTDEVICE)

void btle_hci_adv_whitelist_add(btle_cmd_param_le_add_device_to_whitelist_t* whitelist_device)
{
	wl_device_add(whitelist_device);
}

void btle_hci_adv_whitelist_remove(btle_cmd_param_le_remove_device_from_whitelist_t* whitelist_device)
{
	wl_device_remove(whitelist_device);
}

void btle_hci_adv_whitelist_flush(void)
{
	wl_flush();
}

#endif