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

#ifndef _TS_PERIPHERAL_H__
#define _TS_PERIPHERAL_H__
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

/************************************************************
*************************************************************
* Thin wrappers for common hardware peripherals functions
*************************************************************
************************************************************/




/*****************************************************************************
* Local Definitions
*****************************************************************************/

/**
* Short macro for triggering a task
*/
#define PERIPHERAL_TASK_TRIGGER(x) x = 1

/**
* Short macro for clearing event flag
*/
#define PERIPHERAL_EVENT_CLR(x) x = 0


/**
* Various GPIO bindings for debugging with a logic analyzer
*/
#if DEBUG_NRF_USER

#define DEBUG_PIN_POKE(x) NRF_GPIO->OUTSET = (1 << (x)); \
													NRF_GPIO->OUTCLR = (1 << (x))

#define DEBUG_PIN_SET(x)  NRF_GPIO->OUTSET = (1 << (x));

#define DEBUG_PIN_CLEAR(x)  NRF_GPIO->OUTCLR = (1 << (x));

#else

#define DEBUG_PIN_POKE(x) 

#define DEBUG_PIN_SET(x)  

#define DEBUG_PIN_CLEAR(x)  

#endif


/*****************************************************************************
* Interface functions
*****************************************************************************/

/**
* Enable timer compare with given index in NRF_TIMER0. Resets the timer counter.
* Use the interrupt flag to set whether an interrupt for the compare should be 
* propagated to application space
*/
void periph_timer_start(uint8_t timer, uint16_t value, bool interrupt);

/**
* Abort timer with given index in NRF_TIMER0. Stops the timer counter
*/
void periph_timer_abort(uint8_t timer);

/**
* Do radio setup for BLE advertisement, 0dBm, 1MBit transfer rate, 
* BLE advertisement access address, BLE style header,
* BLE style payload config, 3 byte CRC and 145us TIFS
*/
bool periph_radio_setup(void);

/**
* Set the channel of the radio. Only advertisement channels (37, 38, 39) are accepted.
* Radio must be in state Disabled.
*/
void periph_radio_ch_set(uint8_t ch);

/**
* Set the radio packet pointer for both RX and TX. Buffer
* should be at least 256byte to fit all legally sized packets.
* If the packet is to be transmitted, packet type and length in the header
* must be configured according to BLE specification
*/
void periph_radio_packet_ptr_set(uint8_t* packet);

/**
* Set radio statemaching shortcuts. 
*/
void periph_radio_shorts_set(uint32_t shorts);

/**
* Fetch the previous rssi value recorded in the radio
*/
void periph_radio_rssi_read(uint8_t* rssi);
																						
/**
* Set interrupt mask for the radio peripheral
*/
void periph_radio_intenset(uint32_t interrupt);

/**
* Clear interrupt mask for the radio peripheral
*/
void periph_radio_intenclr(uint32_t interrupt);

/**
* Clear all radio event flags
*/
void periph_radio_evts_clear(void);

/**
* Set the radio's interframe spacing. 
*
* NOTE: While the Reference manual for nRF51 (v2.0) doesn't explicitly 
* state it, this TIFS register works for all interframe spacing (RX->TX,
* TX->RX, RX->RX and TX->TX).
*/
void periph_radio_tifs_set(uint8_t tifs);

/**
* Get the current channel of the radio
* 
* NOTE: Channels are represented as BLE spec channels, not frequency bins.
*/
void periph_radio_channel_get(uint8_t* ch);

/**
* Set up a PPI channel to trigger the given task when the given event is fired.
* This can be used to quickly trigger interperipheral actions without waking the 
* processing unit
*/
void periph_ppi_set(uint8_t ppi_ch, volatile uint32_t* task, volatile uint32_t* event);

/**
* Clear the given PPI channel, disabling the pipe.
*/
void periph_ppi_clear(uint8_t ppi_ch);

/**
* Configure a GPIOTE slot.
*/
void periph_gpiote_config(uint8_t gpiote_slot, uint8_t gpio_pin,
																						uint8_t gpiote_config_mode, 
																						uint8_t gpiote_config_polarity,
																						uint8_t gpiote_config_outinit);



#endif /* _TS_PERIPHERAL_H__ */
