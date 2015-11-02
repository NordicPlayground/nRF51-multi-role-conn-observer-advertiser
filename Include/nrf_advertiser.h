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

#ifndef _TS_ADVERTISER_H__
#define _TS_ADVERTISER_H__
#include <stdint.h>
#include <stdbool.h>
#include "nrf_soc.h"
#include "btle.h"
#include "app_error.h"

/**
* Initialize the timeslot advertiser. The IRQ parameter decides which 
* software interrupt to use for the generated events. For each event
* the advertiser generates, the selected IRQ flag will be set, and the user
* may get events from the queue using tsa_evt_get().
*/
void btle_hci_adv_init(IRQn_Type btle_hci_adv_evt_irq);
	
/**
* Function for getting pending tsa reports. It is recommended to repeat this 
* function until it returns false, as there may be more than one pending report
* in the queue.
*/
bool btle_hci_adv_report_get(nrf_report_t* evt);
															
/**
* Softdevice event handler for the timeslot advertiser. Takes any timeslot event generated
* by Softdevice function sd_evt_get(). 
*/
void btle_hci_adv_sd_evt_handler(uint32_t event);








															
/**
* Set the advertisement parameters for the timeslot advertiser. See btle.h for details about
* the struct parameter
*/
void btle_hci_adv_params_set(btle_cmd_param_le_write_advertising_parameters_t* adv_params);

/**
* Enable or disable advertisement
*/
void btle_hci_adv_enable(btle_adv_mode_t adv_enable);

/**
* Set the content of the timeslot advertiser advertisement packets. This does not include
* any headers or address, as this is controlled internally. To set the advertisement
* address, use the btle_hci_adv_params_set() function.
*/
void btle_hci_adv_data_set(btle_cmd_param_le_write_advertising_data_t* adv_data);

/**
* Set the content of the timeslot advertiser scan response packet. This is the packet the
* beacon will send to respond to external scan requests.
* This does not include any headers or address, as this is controlled internally. 
* To set the advertisement address, use the btle_hci_adv_params_set() function.
*/
void btle_hci_adv_scan_rsp_data_set(btle_cmd_param_le_write_scan_response_data_t* scan_rsp);

/**
* Add a scanner to the device whitelist. If the whitelist filter is enabled, only
* scanners on this whitelist will be accepted. 
* 
* The filter can be enabled with the btle_hci_adv_params_set() function above.
*/
void btle_hci_adv_whitelist_add(btle_cmd_param_le_add_device_to_whitelist_t* whitelist_device);

/**
* Remove a scanner from the device whitelist. If the whitelist filter is enabled, only
* scanners on this whitelist will be accepted. 
* 
* The filter can be enabled with the btle_hci_adv_params_set() function above.
*/
void btle_hci_adv_whitelist_remove(btle_cmd_param_le_remove_device_from_whitelist_t* whitelist_device);

/**
* Remove all scanners from the device whitelist. If the whitelist filter is enabled, only
* scanners on this whitelist will be accepted. 
* 
* The filter can be enabled or disabled with the btle_hci_adv_params_set() function above. 
* Note that this function does not disable the filter functionality.
*/ 
void btle_hci_adv_whitelist_flush(void);

#endif /* _TS_ADVERTISER_H__ */
