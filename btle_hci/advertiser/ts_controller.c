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

#include "ts_controller.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ts_rng.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_assert.h"
#include "app_error.h"	

#include "app_uart.h"

#include "btle.h"
#include "ts_peripheral.h"
#include "nrf_advertiser.h"
#include "nrf_report_disp.h"
#include "ts_whitelist.h"

	
	
/*****************************************************************************
* Local Definitions
*****************************************************************************/


/* Disable this flag to disable listening for scan requests, and jump straight to 
* next advertisement 
*/
#define TS_SEND_SCAN_RSP (1)
//#define softdevice (1)
//#define nosoftdevice (0)

#define BLE_ADV_INTERVAL_100MS 0x00A0  //160=100/0.625
#define BLE_ADV_INTERVAL_150MS 0x00F0  //240=150/0.625


/* Quick macro to scale the interval to the timeslot scale */
#define ADV_INTERVAL_TRANSLATE(interval) (625 * (interval)) // in us

/* short macro to check whether the given event is triggered */
#define RADIO_EVENT(x) (NRF_RADIO->x != 0)

/*****************************************************************************
* Static Globals
*****************************************************************************/
void sm_enter_req(void);


/* Buffer for advertisement data */
static uint8_t ble_adv_data[BLE_ADDR_OFFSET + BLE_ADDR_LEN + BLE_PAYLOAD_MAXLEN];
static uint8_t ble_scan_rsp_data[BLE_ADDR_LEN + BLE_PAYLOAD_MAXLEN];

static uint8_t ble_linklayer_data[BLE_DATA_PAYLOAD_OFFSET + BLE_DATA_PAYLOAD_MAXLEN];
static uint8_t MD_bit;


static  uint8_t ble_acknow_data[] =
														{
															0x09,                               // LL Header (LLID :01 and length : 0  RFU: 0  MD :1)
															0x00,                               // Length of payload: 00
															0x00                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
														};
														
static uint8_t ble_terminate_data[] =
												{
													0x03,                               // LL Header (LLID :11 and length : 0  RFU: 0  MD :0) acknowledge bit set 
													0x02,                               // Length of payload: 02
													0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
													0x02,                               // opcode of termination 
													0x00                                // space for error code 
													
												};
												
		
 static  uint8_t ble_version_data[] =
														{
															0x03,                               // LL Header (LLID :01 and length : 0  RFU: 0  MD :1)
															0x06,                               // Length of payload: 00
															0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
															0x0C,                               // opcode of LL_VERSION_END
															0x00,                               //  LLVersNr
															0x00,                               //  CompId
															0x00,                               //  ComId
															0x00,                               // SubVersNr
															0x00                                // SubVersNr
														}; 		

												

static  uint8_t ble_feature_rsp_data[] =
														{
															0x03,                               // LL Header (LLID :01 and length : 0  RFU: 0  MD :1)
															0x09,                               // Length of payload: 00
															0x00,                               // Padding bits for S1 (REF: the  nRF51 reference manual 16.1.2)
															0x09,                               
															0x00,                               
															0x00,                              
															0x00,                               
															0x00,                               
															0x00,                                
															0x00,
															0x00,
															0x00
														}; 														
												
//#if TS_SEND_SCAN_RSP
/* Buffer for scan response data, only available in scan request/response mode,
* see define at top */


/* Buffer for any message receiving, only available in scan request/response mode,
* see define at top */
														
static uint16_t disconnect_event_counter;
static bool disconnect_activate  ;
														
static uint8_t ble_rx_buf[BLE_ADDR_LEN + BLE_PAYLOAD_MAXLEN]; //BLE_PAYLOAD_MAXLEN =31 byte 0248 bits BLE_ADDR_LEN = 6 

/* Packet counter */
static uint16_t packet_count_valid;

/* Faulty packets counter */
static uint16_t packet_count_invalid;
//#endif

/* Store the radio channel */
static uint8_t channel;

/* running flag for advertiser. Decision point: end of timeslot (adv event) */
static bool sm_adv_run;

/* Channel map for advertisement */
static btle_dd_channel_map_t channel_map;
	
/* min advertisement interval */
static btle_adv_interval_t adv_int_min = BLE_ADV_INTERVAL_100MS;

/* max advertisement interval */
static btle_adv_interval_t adv_int_max = BLE_ADV_INTERVAL_150MS;

/* statemachine state */
static ts_state_t sm_state;

/* advertisement param->type to spec type map */
static const uint8_t ble_adv_type_raw[] = {0, 1, 6, 2};

/* Pool of 255 (the range of the RNG-peripheral) psedorandomly generated values */
//static uint8_t rng_pool[100];

/* A pointer into our pool. Will wrap around upon overflow */


static const uint8_t data_freq_bins[] = {4, 6,8,10,12,14,16,18,20,22,24,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78};

//static uint8_t InitA[BLE_ADDR_LEN];
static uint8_t AdvA[BLE_ADDR_LEN];
static uint8_t AcessA[4];
static uint8_t CRCinit[3];

static uint8_t WinOffset[2];
static uint8_t ConnInterval[2];
static uint8_t ChMapp[5];
static char Hop[1];

static uint8_t missed_counter ;	
	
	
static uint32_t x1,a;
static uint16_t connection_event_counter ;
	

static uint32_t woffset ;
static uint32_t cointerval ;
static uint8_t num_used_channel=0;
static uint8_t used_channel_map[40];
static uint8_t chmapp [40]; 
static uint8_t hopping;

static uint32_t new_cointerval; 
static uint8_t new_ConnInterval[2];
static uint8_t new_WinOffset[2];
static uint32_t new_woffset ;

static uint8_t Instant[2] ;
static uint16_t instant ;

static uint8_t Channel_Map_Instant[2];
static uint16_t channel_map_instant ;
static uint8_t New_ChMapp[5];
static uint8_t new_num_used_channel=0;
static uint8_t new_used_channel_map[40];
static uint8_t new_chmapp [40]; 




static uint8_t last_unmapped_channel =0 ;
static uint8_t unmapped_channel  ;
static uint8_t data_channel  ;





static bool sm_exit_wait_for_idle(void);



static void set_channel_connection (void);
void sm_enter_conn_req(void);
void reconfigure_radio_for_connection (void);
void start_master_to_slave_receive (void);

/*****************************************************************************
* Globals
*****************************************************************************/

/* return param in signal handler */
nrf_radio_signal_callback_return_param_t g_signal_callback_return_param;


/* timeslot request EARLIEST. Used to send the first timeslot request */
static nrf_radio_request_t g_timeslot_req_earliest = 
			{NRF_RADIO_REQ_TYPE_EARLIEST, 
			.params.earliest = {
						HFCLK, 
						NRF_RADIO_PRIORITY_NORMAL, 
						5000, 		//4300 /**< The radio timeslot length (in the range 100 to 100,000] microseconds). */
						10000}    /**< Longest acceptable delay until the start of the requested timeslot*/
			};

/* timeslot request NORMAL. Used to request a periodic timeslot, i.e. advertisement events */
static nrf_radio_request_t g_timeslot_req_normal =
			{NRF_RADIO_REQ_TYPE_NORMAL, 
			.params.normal = {
						HFCLK, 
						NRF_RADIO_PRIORITY_HIGH, 
						TIMESLOT_INTERVAL_100MS, //Distance from the start of the previous radio timeslot		
						TIMESLOT_LENGTH} /**< 10000 The radio timeslot length (in the range [100..100,000] microseconds). */
			};


			
/*****************************************************************************
* Static Functions
*****************************************************************************/
			

/**
* Get next channel to use. Affected by the tsa adv channels.
*/		
			
/**********************************************************************/			
//#if softdevice
 #if defined(WITH_SOFTDEVICE)
/**
* Send initial time slot request to API. 
*/
static __INLINE void timeslot_req_initial(void)
{	
	DEBUG_PIN_POKE(7);
	uint8_t error_code = sd_radio_request(&g_timeslot_req_earliest); 	// send to sd: 
	APP_ERROR_CHECK(error_code);
}


/**
* Request a new timeslot for the next advertisement event
* If the advertiser has been told to stop, the function will not 
* schedule any more, but just end the timeslot.
*/
static __INLINE void next_timeslot_schedule(void)
{
	if (sm_adv_run)
	{ 
		
		g_timeslot_req_normal.params.normal.distance_us =adv_int_min*1000;// 120000;//ADV_INTERVAL_TRANSLATE(adv_int_min) 
																										//+ 1000 * ((rng_pool[pool_index++]) % (ADV_INTERVAL_TRANSLATE(adv_int_max - adv_int_min)));
		g_timeslot_req_normal.params.normal.length_us = 5000 ;
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;

	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}

//this timeslot request will be called when TIMER0 will be timed out because of no reception of any master to slave empty PDU data packet in connection state

static __INLINE void next_timeslot_schedule_connection_woffset(void)
{
	if (sm_adv_run)
	{ 
	
		a = (woffset + 1000-200 ) ;
  
		
		g_timeslot_req_normal.params.normal.distance_us = a ;
		g_timeslot_req_normal.params.normal.length_us = 12000 ;
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
	

	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}


static __INLINE void next_timeslot_schedule_connection(void)
{
	if (sm_adv_run)
	{ 
	
		//a = (cointerval-200+ woffset + 1000 ) ;
    a = (cointerval-300+x1 ) ;
		
		g_timeslot_req_normal.params.normal.distance_us = a ;
		g_timeslot_req_normal.params.normal.length_us = 12000 ;
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
	

	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}


static __INLINE void next_timeslot_schedule_connection_timer(void)
{
	if (sm_adv_run)
	{ 
		
		a = (cointerval-200 ) ;
	
		g_timeslot_req_normal.params.normal.distance_us = a ;
		g_timeslot_req_normal.params.normal.length_us = 12000 ;
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
		

	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}

//this timeslot request will be called after the successful send of empty PDU data from slave to master in connection state 
static __INLINE void next_timeslot_schedule_connection_2(void)
{
	if (sm_adv_run)
	{ 
		
		a = (cointerval-300+x1 ) ;
		g_timeslot_req_normal.params.normal.distance_us = a ;
		g_timeslot_req_normal.params.normal.length_us = 10000 ;
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
		
	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}



static __INLINE void next_timeslot_schedule_advertisement(void)
{
	if (sm_adv_run)
	{ 
		
		g_timeslot_req_normal.params.normal.distance_us = 20000 ;
		g_timeslot_req_normal.params.normal.length_us = 5000 ;
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		uint8_t error_code = sd_radio_request(&g_timeslot_req_normal);
	
		
	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}


//this timeslot request will be called when the requested timeslot is blocked by soft device and this function will request for new timeslot at the next connection interval set by the master in CONN_REQ packet
static __INLINE void next_timeslot_schedule_block(void)
{
	if (sm_adv_run)
	{ 
  
		
		a = (a + cointerval); // the next  timeslot will be called at next connection interval after the blocking of the last timeslot request
		
		g_timeslot_req_normal.params.normal.distance_us = a-1000 ;
		g_timeslot_req_normal.params.normal.length_us = 10000 ;
		g_signal_callback_return_param.params.request.p_next = &g_timeslot_req_normal;
		
		uint8_t error_code = sd_radio_request(&g_timeslot_req_normal);
		
		APP_ERROR_CHECK(error_code);
		
	}
	else
	{
		g_signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
	}
}


void  new_timeslot_order(void)
{   nrf_gpio_pin_toggle(23);
	  switch (sm_state)
			{
				case STATE_MASTER_TO_SLAVE_RECEIVE:
					
				      sm_adv_run = true;
				      sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;
				      set_channel_connection();                  //find data_channel
				      connection_event_counter = 0 ;
				      next_timeslot_schedule_block(); 
				
				break;
				
				
				case STATE_SLAVE_TO_MASTER_TRANSMIT_2:
					
				    sm_adv_run = true;
				    sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;
				
				    last_unmapped_channel = unmapped_channel ;
				
				    if ((channel_map_instant - connection_event_counter )== 1 )	
															 {     
															 
																 memcpy( (void*) &chmapp, (void*) &new_chmapp[0], 40);
																 memcpy( (void*) &used_channel_map, (void*) &new_used_channel_map[0],40);
																 num_used_channel= new_num_used_channel;

															}	
															 
															
				    connection_event_counter++ ;
				    
				    
				if ((instant > 0)&&(connection_event_counter == instant) )
				     {
					      cointerval = new_cointerval ;
					      instant = 0 ; 
								sm_adv_run =1 ;
				     }
				
						      set_channel_connection();                  //find data_channel
								  next_timeslot_schedule_block(); 
	     
				    
     				break;
				
				case STATE_FIRST_ORDER_TIMESLOT:
				 
			    	 sm_state = STATE_FIRST_ORDER_TIMESLOT;
						 sm_adv_run = true;					
					   next_timeslot_schedule_advertisement(); 
				    
				    break;
				
				default:
					// Shouldn't happen 
			  
					ASSERT(false);
				  
				  break;	
			}
}



void ctrl_timeslot_order(void)
{
	sm_adv_run = true;
	timeslot_req_initial(); //sd_radio_request(&g_timeslot_req_earliest);
}


void ctrl_timeslot_order_first(void)
{
	sm_adv_run = true;
	sm_state = STATE_FIRST_ORDER_TIMESLOT;
	timeslot_req_initial(); //sd_radio_request(&g_timeslot_req_earliest);
}

void ctrl_timeslot_abort(void)
{
	sm_adv_run = false;
}

#endif


/**********************************************************************/

void find_channel_map(void);

 void get_connection_param(void)
{
	
	 memset(&AdvA,0, 6);
 memcpy((void*) AdvA, (void*) &ble_rx_buf[9], BLE_ADDR_LEN);
	

	 memset(&AcessA,0, 4);
 memcpy((void*) AcessA, (void*) &ble_rx_buf[15], 4);
	
		 memset(&CRCinit,0, 3);
 memcpy((void*) CRCinit, (void*) &ble_rx_buf[19], 3);
	

	
	memset(&WinOffset, 0, 2);
 memcpy((void*) WinOffset, (void*) &ble_rx_buf[23], 2);
  woffset = WinOffset[1]<<8;
  woffset =  woffset + WinOffset[0] ;
  woffset = ( woffset* 1250) ;


	memset(&ConnInterval,0, 2);
 memcpy((void*) ConnInterval, (void*) &ble_rx_buf[25], 2); 
 cointerval = ConnInterval[1]<<8;
 cointerval = cointerval + ConnInterval[0] ;
 cointerval = (cointerval * 1250)-300 ;
	
	//find_channel_map();
	
	uint8_t i,j;
	
	memset(&ChMapp,0, 5);
 memcpy( (void*) ChMapp, (void*) &ble_rx_buf[31], 5);
 
 //convert channel map bit to channel array chmapp[40]



	char an= 0x01;	
	char b;
	j=0;
	for (i=0;i<40;i++ )
		    { 
					
					chmapp[i]= ChMapp[j] & an ;
				  ChMapp[j]= ChMapp[j] >> 1;
					if ((i== 7)||(i==15)|| (i== 23)|| (i== 31)) j++ ;
				}
	
	
	
	//find number of used channels and prepare used channel map 
	j=0;
	num_used_channel = 0 ;
				
	for(i=0;i<40;i++)
	{
		 if(chmapp[i]== 1) 
		 {
			 num_used_channel = num_used_channel + 1;
		   used_channel_map[j]= i;
       j=j+1;			 
		 }
	}
	
 
  
	memset(&Hop,0, 1);
 memcpy( (void*) Hop, (void*) &ble_rx_buf[36], 1);
   an= 0x1F;	
   b=0;
 b = Hop[0] & an  ;	
 hopping = b;
 
 

}





void set_channel_connection(void)
{
	uint8_t remapping_index;
  unmapped_channel = (last_unmapped_channel + hopping)% 37 ;
	//data_channel = unmapped_channel ;
	
	if(chmapp[unmapped_channel]== 1)
	     data_channel = unmapped_channel ;
	
	else
	{
	   remapping_index = unmapped_channel % num_used_channel;
		 data_channel= used_channel_map[remapping_index] ;
	}		
}	

 







static bool is_conn_req_for_me(void)
{	
	/* check CRC. Add to number of CRC faults if wrong */
	if (0 == NRF_RADIO->CRCSTATUS) 
	{
		++packet_count_invalid;
		return false;
	}
	
	/* check PDU as CONN_REQ */
	if (0x05 != (ble_rx_buf[0] & 0x0F)) // check whether it is CONNECT_REQ PDU type
	{ 
		return false;
	}
	
	/* check included ADV addr, which must match own ADV addr */
	if (memcmp(	(void*) &ble_adv_data[BLE_ADDR_OFFSET], 
							(void*) &ble_rx_buf[BLE_PAYLOAD_OFFSET], BLE_ADDR_LEN) != 0)
	{
		return false;
	}
	
	/* all fields ok */
	++packet_count_valid;
	
	return true;
}




/**
* Extract the scanner address from the rx buffer
*/
static __INLINE void scan_addr_get(btle_address_type_t *addr_type, uint8_t* addr)
{
	*addr_type = ((ble_rx_buf[BLE_TYPE_OFFSET] & BLE_ADDR_TYPE_MASK) > 0 ? BTLE_ADDR_TYPE_RANDOM : BTLE_ADDR_TYPE_PUBLIC);
	
	memcpy((void*) addr, (void*) &ble_rx_buf[BLE_ADDR_OFFSET], BLE_ADDR_LEN);	
}

/**
* Get data from rx-buffer and dispatch scan req event
*/
static __INLINE void scan_req_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t scan_req_report;
	
	/* packet counters */
	scan_req_report.valid_packets = packet_count_valid;
	scan_req_report.invalid_packets = packet_count_invalid;
	 
	/* event details */
	scan_req_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_SCAN_REQ_REPORT;
	scan_req_report.event.opcode			= BTLE_CMD_NONE;
	
	scan_addr_get(&scan_req_report.event.params.nrf_scan_req_report_event.address_type, 
									scan_req_report.event.params.nrf_scan_req_report_event.address);
	
	periph_radio_rssi_read(&(scan_req_report.event.params.nrf_scan_req_report_event.rssi));
	periph_radio_channel_get(&(scan_req_report.event.params.nrf_scan_req_report_event.channel));
	
	/* send scan req event to user space */
	nrf_report_disp_dispatch(&scan_req_report);
}

static __INLINE void conn_req_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t conn_req_report;
	
	/* packet counters */
	conn_req_report.valid_packets = packet_count_valid;
	conn_req_report.invalid_packets = packet_count_invalid;
	 
	/* event details */
	conn_req_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_CONN_REQ_REPORT;
	conn_req_report.event.opcode			= BTLE_CMD_NONE;
	
	memcpy((void*) &(conn_req_report.event.params.nrf_conn_req_report_event.initiator_address[0]), (void*) &ble_rx_buf[3],6);
	memcpy((void*) &(conn_req_report.event.params.nrf_conn_req_report_event.acess_address[0]), (void*) &AcessA,4);
	memcpy((void*) &(conn_req_report.event.params.nrf_conn_req_report_event.crc_init[0]), (void*) &CRCinit,3);
	conn_req_report.event.params.nrf_conn_req_report_event.win_size = ble_rx_buf[22]; 
	conn_req_report.event.params.nrf_conn_req_report_event.win_offset = woffset;
	conn_req_report.event.params.nrf_conn_req_report_event.conn_interval= cointerval+300;
	memcpy((void*) &(conn_req_report.event.params.nrf_conn_req_report_event.latency[0]), (void*) &ble_rx_buf[27],2);
	memcpy((void*) &(conn_req_report.event.params.nrf_conn_req_report_event.timeout[0]), (void*) &ble_rx_buf[29],2);
	memcpy((void*) &(conn_req_report.event.params.nrf_conn_req_report_event.channel_map[0]), (void*) &ble_rx_buf[31], 5);
	conn_req_report.event.params.nrf_conn_req_report_event.hop = hopping;
	
	periph_radio_channel_get(&(conn_req_report.event.params. nrf_conn_req_report_event.channel));
	
	/* send scan req event to user space */
	nrf_report_disp_dispatch(&conn_req_report);
}


static __INLINE void disconnected_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t disconnected_report;
	uint16_t event_counter_;
	/* packet counters */
	 
	/* event details */
	disconnected_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_DISCONNECTED_REPORT;
	disconnected_report.event.opcode			= BTLE_CMD_NONE;
	event_counter_= connection_event_counter+1;
	disconnected_report.event.params.nrf_disconnected_report_event.event_counter[0] = event_counter_ & (0x00FF);
	disconnected_report.event.params.nrf_disconnected_report_event.event_counter[1] = (event_counter_ >> 8) & (0x00FF);
	
	periph_radio_channel_get(&(disconnected_report.event.params.nrf_disconnected_report_event.channel));
	
	
	/* send scan req event to user space */
	nrf_report_disp_dispatch(&disconnected_report);
}	

static __INLINE void version_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t version_report;
	uint16_t event_counter_;
	/* packet counters */
	 
	/* event details */
	version_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_VERSION_REPORT_RECEIVED;
	version_report.event.opcode			= BTLE_CMD_NONE;
	event_counter_= connection_event_counter+1;
	
	version_report.event.params.nrf_version_report_received_event.event_counter[0] = event_counter_ & (0x00FF);
	version_report.event.params.nrf_version_report_received_event.event_counter[1] = (event_counter_ >> 8) & (0x00FF);
	
	periph_radio_channel_get(&(version_report.event.params.nrf_version_report_received_event.channel));
	version_report.event.params.nrf_version_report_received_event.LLVersNr = ble_rx_buf[4];
	version_report.event.params.nrf_version_report_received_event.CompId[0] = ble_rx_buf[5];
	version_report.event.params.nrf_version_report_received_event.CompId[1] = ble_rx_buf[6];
	version_report.event.params.nrf_version_report_received_event.SubVersNr[0] = ble_rx_buf[7];
	version_report.event.params.nrf_version_report_received_event.SubVersNr[1] = ble_rx_buf[8];
	
	/* send scan req event to user space */
	nrf_report_disp_dispatch(&version_report);
}	

static __INLINE void feature_req_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t feature_req_report;
	uint16_t event_counter_;
	/* packet counters */
	 
	/* event details */
	feature_req_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_FEATURE_REQ_REPORT_RECEIVED;
	feature_req_report.event.opcode			= BTLE_CMD_NONE;
	event_counter_= connection_event_counter+1;
	
	feature_req_report.event.params.nrf_feature_req_report_received_event.event_counter[0] = event_counter_ & (0x00FF);
	feature_req_report.event.params.nrf_feature_req_report_received_event.event_counter[1] = (event_counter_ >> 8) & (0x00FF);
	
	periph_radio_channel_get(&(feature_req_report.event.params.nrf_feature_req_report_received_event.channel));
	
	memcpy((void*) &(feature_req_report.event.params.nrf_feature_req_report_received_event.feature_set[0]), (void*) &ble_rx_buf[4],8);	
		
	/* send scan req event to user space */
	nrf_report_disp_dispatch(&feature_req_report);
}	


static __INLINE void channel_map_update_req_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t channel_map_update_req_report;
	uint16_t event_counter_;
	/* packet counters */
	 
	/* event details */
	channel_map_update_req_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_CHANNEL_MAP_UPDATE_REQ_REPORT_RECEIVED;
	channel_map_update_req_report.event.opcode			= BTLE_CMD_NONE;
	event_counter_= connection_event_counter+1;
	
	channel_map_update_req_report.event.params.nrf_channel_map_update_req_report_received_event.event_counter[0] = event_counter_ & (0x00FF);
	channel_map_update_req_report.event.params.nrf_channel_map_update_req_report_received_event.event_counter[1] = (event_counter_ >> 8) & (0x00FF);
	
	periph_radio_channel_get(&(channel_map_update_req_report.event.params.nrf_channel_map_update_req_report_received_event.channel));
	
	memcpy((void*) &(channel_map_update_req_report.event.params.nrf_channel_map_update_req_report_received_event.channel_map[0]), (void*) &ble_rx_buf[4],5);	
	memcpy((void*) &(channel_map_update_req_report.event.params.nrf_channel_map_update_req_report_received_event.instant[0]), (void*) &ble_rx_buf[9],2);	
	/* send channel mapupdate req event to user space */
	nrf_report_disp_dispatch(&channel_map_update_req_report);
}	

static __INLINE void conn_update_req_evt_dispatch(void)
{
	/* prepare scan req report */
	nrf_report_t conn_update_req_report;
	uint16_t event_counter_;
	/* packet counters */
	 
	/* event details */
	conn_update_req_report.event.event_code = BTLE_VS_EVENT_NRF_LL_EVENT_CONNECTION_UPDATE_REQ_REPORT_RECEIVED;
	conn_update_req_report.event.opcode			= BTLE_CMD_NONE;
	event_counter_= connection_event_counter+1;
	
	conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.event_counter[0] = event_counter_ & (0x00FF);
	conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.event_counter[1] = (event_counter_ >> 8) & (0x00FF);
	
	periph_radio_channel_get(&(conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.channel));
	
	conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.win_size = ble_rx_buf[4]; 
	conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.win_offset = new_woffset;
	conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.conn_interval= new_cointerval+300;
	memcpy((void*) &(conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.latency[0]), (void*) &ble_rx_buf[9],2);
	memcpy((void*) &(conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.timeout[0]), (void*) &ble_rx_buf[11],2);
	
		
	memcpy((void*) &(conn_update_req_report.event.params.nrf_connecion_update_req_report_received_event.instant[0]), (void*) &ble_rx_buf[13],2);	
	/* send channel mapupdate req event to user space */
	nrf_report_disp_dispatch(&conn_update_req_report);
}	


/**
* Short check to verify that the incomming 
* message was a scan request for this unit
*/
static bool is_scan_req_for_me(void)
{	
	/* check CRC. Add to number of CRC faults if wrong */
	if (0 == NRF_RADIO->CRCSTATUS) 
	{
		++packet_count_invalid;
		return false;
	}
	
	/* check message type and length */
	
	if ((0x03 != (ble_rx_buf[0] & 0x0F)) || 
			(0x0C != ble_rx_buf[1])) //C=12=1100
	{
		return false;
	}
	/* check included ADV addr, which must match own ADV addr */
	if (memcmp(	(void*) &ble_adv_data[BLE_ADDR_OFFSET], 
							(void*) &ble_rx_buf[BLE_PAYLOAD_OFFSET], BLE_ADDR_LEN) != 0)
	{
		return false;
	}
	
	/* all fields ok */
	++packet_count_valid;
	
	return true;
}





static __INLINE void channel_iterate(void)
{
	channel_map = BTLE_CHANNEL_MAP_ALL ;
	while (((channel_map & (1 << (++channel - 37))) == 0) && channel < 40);	
}




static __INLINE void adv_evt_setup(void)
{
	periph_radio_setup();
	
	/* set channel to first in sequence */
	channel = 36; /* will be iterated by channel_iterate() */
	channel_iterate();

}	


static void sm_enter_adv_send(void)
{
	sm_state = STATE_ADV_SEND;
	

	periph_radio_ch_set(channel);

	
	/* trigger task early, the rest of the setup can be done in RXRU */

	PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_TXEN);
	
	periph_radio_packet_ptr_set(&ble_adv_data[0]);
	
#if TS_SEND_SCAN_RSP
	periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
														RADIO_SHORTS_END_DISABLE_Msk |
														RADIO_SHORTS_DISABLED_RXEN_Msk);
#else
	periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
														RADIO_SHORTS_END_DISABLE_Msk);
#endif    
	
	periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
//	periph_radio_tifs_set(148);
	
}



	
void sm_enter_master_to_slave_receive (void)
{
	
							sm_state = STATE_MASTER_TO_SLAVE_RECEIVE_2;
							set_channel_connection(); //find data_channel index

							NRF_RADIO->FREQUENCY = data_freq_bins[data_channel]; 
	            NRF_RADIO->DATAWHITEIV = 0;
	            NRF_RADIO->DATAWHITEIV = (uint32_t) data_channel;
	
							NRF_RADIO->TXADDRESS = 0x00;					// Use logical address 0 (prefix0 + base0)  when transmitting
							NRF_RADIO->RXADDRESSES = 0x01;        // Use logical address 0 (prefix0 + base0)  when receiving
	
							/* configure radio for master to slave data receive*/
							periph_radio_packet_ptr_set(&ble_rx_buf[0]);
	
							NRF_RADIO->EVENTS_DISABLED = 0;
							NRF_RADIO->EVENTS_ADDRESS = 0;
								
							PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_RXEN);
							periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																			RADIO_SHORTS_END_DISABLE_Msk |
																			RADIO_SHORTS_DISABLED_TXEN_Msk);
							periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
							
							periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
							
							/* change the tifs in order to be able to capture all packets */
							periph_radio_tifs_set(152);
							
}

void enter_wait_for_idle(void)
{

               sm_state = STATE_WAIT_FOR_IDLE;
							
							 periph_timer_abort(0);
							 periph_ppi_clear(0);
							
							 NRF_TIMER0->TASKS_CLEAR = 1;
							
						//	periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						//  PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
							
							periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	            PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
							

							periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
							periph_radio_shorts_set(0);

							
	            PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_DISABLE);

}


void set_base_prefix_crc_with_new_perameter_from_conn_req (void)
{

              NRF_RADIO->CRCINIT = CRCinit[2];
	            NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
	            NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[1];
							NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
							NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[0];				
	
							NRF_RADIO->PREFIX0	 = AcessA[3]; //0x8e;
	            NRF_RADIO->BASE0 = AcessA[2];
							NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
							NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[1];
							NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
							NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[0];
							NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
												
							NRF_RADIO->TXADDRESS = 0x00;					// Use logical address 0 (prefix0 + base0)  when transmitting
							NRF_RADIO->RXADDRESSES = 0x01;        // Use logical address 0 (prefix0 + base0)  when receiving


}




void get_new_connection_perameter (void)
	
{
                   memset(&new_ConnInterval,0, 2);
									 memcpy((void*) new_ConnInterval, (void*) &ble_rx_buf[7], 2); 
									 new_cointerval = new_ConnInterval[1]<<8;
									 new_cointerval = new_cointerval + new_ConnInterval[0] ;
									 new_cointerval = (new_cointerval * 1250)-300 ;
								 
								   memset(&new_WinOffset, 0, 2);
									 memcpy((void*) new_WinOffset, (void*) &ble_rx_buf[5], 2);
								
								   new_woffset = new_WinOffset[1]<<8;
									 new_woffset = new_woffset + new_WinOffset[0] ;
									 new_woffset = (new_woffset * 1250) ;
								 
								   memset(&Instant, 0, 2);
									 memcpy((void*) Instant, (void*) &ble_rx_buf[13], 2);
								   instant = Instant[1]<<8;
									 instant = instant + Instant[0] ;




}

void get_new_channel_map_perameter (void)
{										
								   memset(&Channel_Map_Instant, 0, 2);
									 memcpy((void*) Channel_Map_Instant, (void*) &ble_rx_buf[9], 2);
								   channel_map_instant = Channel_Map_Instant[1]<<8;
									 channel_map_instant = channel_map_instant + Channel_Map_Instant[0] ;
							 
								   memset(&New_ChMapp,0, 5);
								   memcpy( (void*) New_ChMapp, (void*) &ble_rx_buf[4], 5);


                  uint8_t i,j;
									char an= 0x01;	
								
									j=0;
									for (i=0;i<40;i++ )
												{ 
													
													new_chmapp[i]= New_ChMapp[j] & an ;
													New_ChMapp[j]= New_ChMapp[j] >> 1;
													if ((i== 7)||(i==15)|| (i== 23)|| (i== 31)) j++ ;
												}
									
									
									
									//find number of used channels and prepare used channel map 
									j=0;
									new_num_used_channel = 0 ;
												
									for(i=0;i<40;i++)
									{
										 if(new_chmapp[i]== 1) 
										 {
											 new_num_used_channel = new_num_used_channel + 1;
											 new_used_channel_map[j]= i;
											 j=j+1;			 
										 }
									}
}



//#if nosoftdevice
#if defined(NO_SOFTDEVICE)


void TIMER0_IRQHandler(void)
{
	switch (sm_state)
			{
				case STATE_WAIT_FOR_IDLE_1:
					 if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
						{  
							 periph_timer_abort(0);
							 NRF_TIMER0->TASKS_CLEAR = 1;
							
							instant = 0 ;
						  missed_counter = 0 ;
						 
							
							adv_evt_setup();
							sm_enter_adv_send();
						
						}  
			     break;
						
				case STATE_REQ:
					 if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
						{  
	  
							enter_wait_for_idle();
							
						}  
			     break;

        case STATE_MASTER_TO_SLAVE_RECEIVE:
					 if (NRF_TIMER0->EVENTS_COMPARE[0] != 0) 
						{  
							
							set_base_prefix_crc_with_new_perameter_from_conn_req ();
							
							sm_enter_master_to_slave_receive();
						
						}  
			     break;	
						
				case STATE_SLAVE_TO_MASTER_TRANSMIT_2:
					 if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
						{  
							 sm_state = STATE_MASTER_TO_SLAVE_RECEIVE_3;
											
						   memset(&ble_rx_buf[0], 0, 15);  // clear the RAM to avoid interference from last data
					     NRF_TIMER0->POWER = 1;
							
							
							last_unmapped_channel = unmapped_channel ;
							
							if ((channel_map_instant - connection_event_counter )== 1 )	
														 {     
																 memcpy( (void*) &chmapp, (void*) &new_chmapp[0], 40);
																 memcpy( (void*) &used_channel_map, (void*) &new_used_channel_map[0],40);
																 num_used_channel= new_num_used_channel;
															}						
															 
							
							
							set_channel_connection(); //find data_channel
							
							reconfigure_radio_for_connection () ;
							start_master_to_slave_receive ();
							
							periph_timer_start(0, 5000, true);
						
							
							
							
			
						}  
			     break;	
						
					case STATE_MASTER_TO_SLAVE_RECEIVE_3:
									 if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
										{ 
											 sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;	
											
											 periph_timer_abort(0);
											 NRF_TIMER0->TASKS_CLEAR = 1;
											 x1=0;
											
											
											 missed_counter ++ ;
											
											if (missed_counter == 10 ) 
											{     
												    missed_counter = 0 ;
												
														periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
														PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
														NRF_RADIO->SHORTS = 0;
														
														last_unmapped_channel= 0;
														unmapped_channel= 0 ;
												    
                            adv_evt_setup();
						                sm_enter_adv_send();												
												
														
											}

											connection_event_counter ++ ;

											if ((instant > 0)&&(connection_event_counter == instant ))
											{										   
														   instant = 0 ; 
															 sm_adv_run =1 ;
												       cointerval = new_cointerval ;

											}
											
                        periph_timer_start(0, cointerval-5400, true);
											
						            
										}
								   break;	

						
         default:
					// Shouldn't happen 
					ASSERT(false);
				  break;						
			}
}

void next_advertisement (void)
{
	 periph_timer_start(0, adv_int_min*1000/*120000*/, true); // Advertisement interval in us
   NVIC_EnableIRQ(TIMER0_IRQn);
	
}


void RADIO_IRQHandler(void)

{
	// check state, and act accordingly 
			switch (sm_state)
			{
				case STATE_ADV_SEND:
						if (NRF_RADIO->EVENTS_DISABLED != 0)
						{
							
							
							periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
							
							periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
							
							memset(&ble_rx_buf[0], 0, 15);  
							
							periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
							
							sm_enter_req();
						}
						break;
					
				case STATE_REQ:
					
						if(RADIO_EVENT(EVENTS_ADDRESS))  	
						 {

							periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
						 }
						
						 if (RADIO_EVENT(EVENTS_DISABLED))
							{
							periph_timer_abort(0);
							
							periph_ppi_clear(0);
							
							periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						
         
						
									if(is_conn_req_for_me())
									{  
										 sm_state = STATE_MASTER_TO_SLAVE_RECEIVE;
							
										 get_connection_param();
										
										 periph_timer_abort(0);
										 NRF_TIMER0->TASKS_CLEAR = 1;
										
										 periph_timer_start(0, 800+woffset, true);
										
										 NRF_RADIO->TASKS_DISABLE = 1; // to clear TX state from sm_enter_req() 
										 NRF_RADIO->SHORTS = 0;
										 conn_req_evt_dispatch();
										
					    	}
								else if (is_scan_req_for_me())
								{  
									
									periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
									PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
									
									periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
									PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
									
									sm_state = STATE_WAIT_FOR_IDLE;
									
								
									periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
									periph_radio_packet_ptr_set(&ble_scan_rsp_data[0]);
									periph_radio_shorts_set(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
									
									periph_radio_tifs_set(150);
									scan_req_evt_dispatch();

								
								}	
								else 
									
								{  

									sm_state = STATE_WAIT_FOR_IDLE;
	
									periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
									PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
									
									periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
									periph_radio_shorts_set(0);
	
									
									PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_DISABLE);
										
									
									
								}
					}
				
					break;
					
       case STATE_MASTER_TO_SLAVE_RECEIVE_3:
					
						if(RADIO_EVENT(EVENTS_ADDRESS))  	
							 {

								periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
								PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
								 
					  	 }
					
				  	if (RADIO_EVENT(EVENTS_DISABLED)) //
					    {    
						   

							periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	            PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						
								if (((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x02))  // check for termination end
								 {
	
											if(ble_rx_buf[0] == 0x03)
													 ble_terminate_data[0]=0x07;
											else if (ble_rx_buf[0] == 0x0F) 
													ble_terminate_data[0]=0x0B;
										
											ble_terminate_data[4]= ble_rx_buf[4]; 
											periph_radio_packet_ptr_set(&ble_terminate_data[0]);
											periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
											periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
											periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);///////address mask
											sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_END;	
											
                      disconnected_evt_dispatch();												
										
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x00)/*&& (ble_rx_buf[1]== 0x0C)*/)   // check for CONN_UPDATE_REQ
							 {
							 
								     get_new_connection_perameter () ;
								 
											 if(ble_rx_buf[0] == 0x03)
													 ble_linklayer_data[0]=0x05;
											else if (ble_rx_buf[0] == 0x0F) 
													ble_linklayer_data[0]=0x09;
											
											
	
											periph_radio_packet_ptr_set(&ble_linklayer_data[0]);
								
											periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						
											periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
										
										
										
											periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
										
											sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;		
										  conn_update_req_evt_dispatch();
							 
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x01)/*&& (ble_rx_buf[1]== 0x08)*/)   // check for CHANNEL_MAP_REQ
							 {   

								  
														
										if(ble_rx_buf[0] == 0x03)
													 ble_acknow_data[0]=0x05;
										else if (ble_rx_buf[0] == 0x0F) 
													ble_acknow_data[0]=0x09;

											periph_radio_packet_ptr_set(&ble_acknow_data[0]);
											get_new_channel_map_perameter ();
										
											periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
											periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
												
											periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
										
											sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;	
                      channel_map_update_req_evt_dispatch();											
										
								 
								 
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x0C)/*&& (ble_rx_buf[1]== 0x08)*/)   // check for LL_VERSION_IND
							 { 
 								 
								 if((ble_rx_buf[0]& (0x0F)) == 0x03)
								    ble_version_data[0]=0x07;
						     else if ((ble_rx_buf[0] & (0x0F)) == 0x0F) 
								    ble_version_data[0]=0x0B;  
								 
								 
								    periph_radio_packet_ptr_set(&ble_version_data[0]);
											
										periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
										periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
										
										periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);///////address mask
										
										sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;	
								 
								    version_evt_dispatch();	
								 
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x08)/*&& (ble_rx_buf[1]== 0x08)*/)   // check for LL_FEATURE_REQ
							 { 

								 if((ble_rx_buf[0]& (0x0F)) == 0x03)
								    ble_feature_rsp_data[0]=0x07;
						     else if ((ble_rx_buf[0] & (0x0F)) == 0x0F) 
								    ble_feature_rsp_data[0]=0x0B;  
								 
								 
								    periph_radio_packet_ptr_set(&ble_feature_rsp_data[0]);
								
										periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
										periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

										periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);///////address mask
										
										sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;	
								 
								    feature_req_evt_dispatch();
								 
							 }
							 
						  else
							{

//							 if((ble_rx_buf[0] & (0x0C)) == 0x00)
//								ble_linklayer_data[0]=0x15;
//						  else if ((ble_rx_buf[0] & (0x0C))== 0x0C) 
//								ble_linklayer_data[0]=0x19;
//							
//							 ble_linklayer_data [0] = ble_linklayer_data [0] & 0xEF;//(0b11101111);
//	             ble_linklayer_data [0] = ble_linklayer_data [0] | (MD_bit << 4 );
							
							if ((connection_event_counter >= disconnect_event_counter) && ( disconnect_activate ==true)) 
							{   
								  	if((ble_rx_buf[0] & (0x0C)) == 0x00)
													 ble_terminate_data[0]=0x07;
											else if ((ble_rx_buf[0] & (0x0C))== 0x0C) 
													ble_terminate_data[0]=0x0B;
											
								  periph_radio_packet_ptr_set(&ble_terminate_data[0]);
							    sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_END;
											
									disconnected_evt_dispatch();
							}   
							else 
							{  
								
							if((ble_rx_buf[0] & (0x0C)) == 0x00)
								  ble_linklayer_data[0]=0x05;
						  else if ((ble_rx_buf[0] & (0x0C))== 0x0C) 
								  ble_linklayer_data[0]=0x09;
							
						 	    ble_linklayer_data [0] = ble_linklayer_data [0] & 0xEF;//(0b11101111);
	                ble_linklayer_data [0] = ble_linklayer_data [0] | (MD_bit << 4 );
								  periph_radio_packet_ptr_set(&ble_linklayer_data[0]);
							    sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;
							
							}
						
							/* configure radio for master to slave data receive*/
						//	periph_radio_packet_ptr_set(&ble_linklayer_data[0]);
					
							periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																			RADIO_SHORTS_END_DISABLE_Msk );
																			//RADIO_SHORTS_DISABLED_TXEN_Msk);
							periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
	
							periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);///////address mask
							
						 // sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;
							
				
							}
							
					}
					break;					
					
				case	STATE_SLAVE_TO_MASTER_TRANSMIT_END:
						if (RADIO_EVENT(EVENTS_DISABLED))
					 {    
						  sm_state = STATE_START_FROM_INITIAL;
						 
           periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
						 
						periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	          PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						 
						 
								 
							periph_timer_abort(0);
		          NRF_TIMER0->TASKS_CLEAR = 1;
								 
							periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
							NRF_RADIO->TASKS_DISABLE = 1; // to clear TX state from sm_enter_req() 
	            NRF_RADIO->SHORTS = 0;
					
					}
					
					break;
					
       case STATE_SLAVE_TO_MASTER_TRANSMIT_3:
					{	
				
						if(RADIO_EVENT(EVENTS_ADDRESS))  	
						 {

							periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
						 }
					 
					 
							if (RADIO_EVENT(EVENTS_DISABLED))
							{    

								sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;		
								
								periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
								PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
								NRF_RADIO->SHORTS = 0;		

								NRF_TIMER0->TASKS_CAPTURE[1] = 1;
								x1 = NRF_TIMER0->CC[1] ;	
								NRF_TIMER0->TASKS_STOP = 1;		
								missed_counter = 0 ;
								
								connection_event_counter ++ ;
								
								if ((instant > 0)&&(connection_event_counter == instant ))
													{
																	 instant = 0 ; 
																	 sm_adv_run =1 ;
																	 cointerval = new_cointerval ;

													}
													
									periph_timer_start(0, (cointerval-400), true);
													
								 
							}
					
				}
					break;		
				
    	case STATE_MASTER_TO_SLAVE_RECEIVE_2:
					
						if(RADIO_EVENT(EVENTS_ADDRESS))  	
						 {
							 
							periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);

						 }
						
						if (RADIO_EVENT(EVENTS_DISABLED)) //
						{    
								 
								 connection_event_counter= 0;
							
								 periph_timer_abort(0);
								 NRF_TIMER0->TASKS_CLEAR = 1;

								periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
								PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
							
								
//								if((ble_rx_buf[0] & (0x0F))== 0x01) ble_linklayer_data[0]=0x05;
//								else if ((ble_rx_buf[0] & (0x0F))== 0x0D) ble_linklayer_data[0]=0x09;
							
							    if((ble_rx_buf[0] & (0x0C))== 0x00) ble_linklayer_data[0]=0x05;
								else if ((ble_rx_buf[0] & (0x0C))== 0x0C) ble_linklayer_data[0]=0x09;
								
							 ble_linklayer_data [0] = ble_linklayer_data [0] & 0xEF;//(0b11101111);
	              ble_linklayer_data [0] = ble_linklayer_data [0] | (MD_bit << 4 );
							
								/* configure radio for master to slave data receive*/
								 periph_radio_packet_ptr_set(&ble_linklayer_data[0]);
						
								periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																				RADIO_SHORTS_END_DISABLE_Msk );
																				//RADIO_SHORTS_DISABLED_TXEN_Msk);
								periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

								periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);///////address mask
								
								sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT;
								
						}
					break;
					
        case STATE_SLAVE_TO_MASTER_TRANSMIT:
						
					if(RADIO_EVENT(EVENTS_ADDRESS))  	
					 {

						periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
					 }
					 
					 
					if (RADIO_EVENT(EVENTS_DISABLED))
					{    
			
						sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;
						
						periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	          PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						NRF_RADIO->SHORTS = 0;		

						periph_timer_start(0, (cointerval-400) , true);      //anchor point 
            						
					}
					break;

					
				case STATE_WAIT_FOR_IDLE:
					if (RADIO_EVENT(EVENTS_DISABLED))
					{
						// state exit function returns whether the adv event is complete 
						
						bool adv_evt_done = sm_exit_wait_for_idle();
						
						if (adv_evt_done)
						{ 
							sm_state= STATE_WAIT_FOR_IDLE_1;
							next_advertisement();
						}
						else
						{
							sm_enter_adv_send();
						}
					}
					break;		

       case STATE_START_FROM_INITIAL:
				 
					if (RADIO_EVENT(EVENTS_DISABLED))
					{ 
						
						periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	          PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						NRF_RADIO->SHORTS = 0;
						
						last_unmapped_channel= 0;
						unmapped_channel= 0 ;
						
						instant = 0 ;
						missed_counter = 0 ;
				
						
						adv_evt_setup();
						sm_enter_adv_send();
						
					}
					break;								

					
			
				default:
					// Shouldn't happen 
					ASSERT(false);
				  break;
			}	
}

#endif
/***********************************************************************************/

 void sm_enter_req(void)
{
	sm_state = STATE_REQ;

	
	periph_radio_packet_ptr_set(&ble_rx_buf[0]);
	
	 
	
	periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
														RADIO_SHORTS_END_DISABLE_Msk |
	                          RADIO_SHORTS_DISABLED_TXEN_Msk );
	
	periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

	/* change the tifs in order to be able to capture all packets */
	periph_radio_tifs_set(148);
	
	/* start the timer that aborts the RX if no address is received. */
	periph_timer_start(0, 200, true);
	
	/* set PPI pipe to stop the timer as soon as an address is received */
	periph_ppi_set(0, &(NRF_TIMER0->TASKS_STOP), &(NRF_RADIO->EVENTS_ADDRESS));
	
}



/*****************************************
* Functions for start/end of WAIT_FOR_IDLE
******************************************/


static bool sm_exit_wait_for_idle(void)
{
	periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
	
	channel_iterate();
	return (channel > 39);
}


//#if nosoftdevice
#if defined(NO_SOFTDEVICE)
void radio_2(void)
{
			adv_evt_setup();
}


void adv_2 ()
{ 		
			sm_enter_adv_send();
}


#endif










/***************************************************************************/


void reconfigure_radio_for_connection (void)
{
                      NRF_RADIO->POWER = 1;
	
											NRF_RADIO->EVENTS_DISABLED = 0;
											NRF_RADIO->EVENTS_READY = 0;
											
											/* Timer: 1us resolution, disable peripheral */
											 
											NRF_TIMER0->PRESCALER = 4;
											
											NRF_TIMER0->TASKS_STOP = 1;
	
						          NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
                      NRF_RADIO->MODE 	 = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

											NRF_RADIO->PREFIX0	 = AcessA[3]; //0x8e;
											NRF_RADIO->BASE0 = AcessA[2];
											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[1];
											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[0];
											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
						
							NRF_RADIO->FREQUENCY = data_freq_bins[data_channel]; 
	            NRF_RADIO->DATAWHITEIV = 0;
	            NRF_RADIO->DATAWHITEIV = (uint32_t) data_channel;
							

							NRF_RADIO->TXADDRESS = 0x00;					// Use logical address 0 (prefix0 + base0)  when transmitting
							NRF_RADIO->RXADDRESSES = 0x01;        // Use logical address 0 (prefix0 + base0)  when receiving
							
							/* PCNF-> Packet Configuration. Now we need to configure the sizes S0, S1 and length field to match the datapacket format of the advertisement packets. */
   NRF_RADIO->PCNF0 =  (
                          (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)    // length of S0 field in bytes 0-1. // 8UL and 1UL 
                        | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)    // length of S1 field in bits 0-8. // 16UL and FUL
                        | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)    // length of length field in bits 0-8. // 0UL and FUL
                      );
 
	/* Packet configuration */
  NRF_RADIO->PCNF1 =  (
                          (((37UL)                      << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)   // maximum length of payload in bytes [0-255]
                        | (((0UL)                       << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk)	// expand the payload with N bytes in addition to LENGTH [0-255]
                        | (((3UL)                       << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)    // base address length in number of bytes.
                        | (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)   // endianess of the S0, LENGTH, S1 and PAYLOAD fields.
                        | (((1UL)                       << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)	// enable packet whitening
                      );
	NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | 
                       (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); // Skip Address when computing crc
											 
											 
	                    NRF_RADIO->CRCINIT = CRCinit[2];
											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[1];
											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[0];			
											
											NRF_RADIO->CRCPOLY = 0x00065B;    // CRC polynomial function
											
											NVIC_EnableIRQ(RADIO_IRQn);



}

void start_master_to_slave_receive (void)
{
/* configure radio for master to slave data receive*/
							periph_radio_packet_ptr_set(&ble_rx_buf[0]);
							NRF_RADIO->EVENTS_DISABLED = 0;
							
							NRF_RADIO->EVENTS_ADDRESS = 0;
								
							PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_RXEN);
							periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																			RADIO_SHORTS_END_DISABLE_Msk |
																			RADIO_SHORTS_DISABLED_TXEN_Msk);
							periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
							
							periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
							
							/* change the tifs in order to be able to capture all packets */
							periph_radio_tifs_set(153);

}

//#if softdevice
 #if defined(WITH_SOFTDEVICE)
__INLINE void ctrl_signal_handler(uint8_t sig)
{ 
	switch (sig)
	{
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:	/// timeslot start 
		{
					
			
				switch (sm_state)
					{
						case STATE_FIRST_ORDER_TIMESLOT:
							        
						
									  
						          NRF_TIMER0->POWER = 1;
						
										  adv_evt_setup();
									  	sm_enter_adv_send();
						          
						          instant = 0 ;
						          missed_counter = 0 ;
						          
						
			        		break;
						
						case STATE_MASTER_TO_SLAVE_RECEIVE:
									     
						           NRF_TIMER0->POWER = 1;
                       reconfigure_radio_for_connection () ;
						
						
											//set_base_prefix_crc_with_new_perameter_from_conn_req ();
											
//											NRF_RADIO->CRCINIT = CRCinit[2];
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[1];
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[0];				
//					
//											NRF_RADIO->PREFIX0	 = AcessA[3]; //0x8e;
//											NRF_RADIO->BASE0 = AcessA[2];
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[1];
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[0];
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
//																
//											NRF_RADIO->TXADDRESS = 0x00;					// Use logical address 0 (prefix0 + base0)  when transmitting
//											NRF_RADIO->RXADDRESSES = 0x01;        // Use logical address 0 (prefix0 + base0)  when receiving
											
											 sm_enter_master_to_slave_receive();
										   periph_timer_start(0, 6000, true);///////////////////////////////////////////////
										 
    									 x1=0;
										
									 break;	
						
						case STATE_SLAVE_TO_MASTER_TRANSMIT_2:
									  
							        sm_state = STATE_MASTER_TO_SLAVE_RECEIVE_3;
											
						          memset(&ble_rx_buf[0], 0, 15);  // clear the RAM to avoid interference from last data
						
						            NRF_TIMER0->POWER = 1;
						
						               last_unmapped_channel = unmapped_channel ; // update last_unmapped_channel
						
														if ((channel_map_instant - connection_event_counter )== 1 )	
															 {     									
																 memcpy( (void*) &chmapp, (void*) &new_chmapp[0], 40);
																 memcpy( (void*) &used_channel_map, (void*) &new_used_channel_map[0],40);
																 num_used_channel= new_num_used_channel;

															}						
						
											set_channel_connection(); //find data_channel
															
//											}
												
						          reconfigure_radio_for_connection () ;
						
											start_master_to_slave_receive ();

							        // start TIMER0 to calculate the distance from timeslot start to slave to master reception and store the time in x1 to use it for next timeslot order 
						          // this x1 value is used to compensate the time drift
						          periph_timer_start(0, 6000, true);///////////////////////////////////////////////
										 
    									x1=0;
						
                  
						
			            break;
						
						default:
					// Shouldn't happen 
					ASSERT(false);
				  break;
					}
		}
			break;

		
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO: /**************************************************************/
		{
		
			/* check state, and act accordingly */
			switch (sm_state)
			{
				case STATE_ADV_SEND:
					if (RADIO_EVENT(EVENTS_DISABLED))
					{
	
					  periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk); // always clear interrupt at the start of new state 
	          PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);   // always clear event at the start of new state 
						
						periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
						
						memset(&ble_rx_buf[0], 0, 15);  // clear the RAM to avoid interference from last data 
						
						periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);

						sm_enter_req();
						
					
						
					}
					break;
					
			 case STATE_REQ:
					if(RADIO_EVENT(EVENTS_ADDRESS))  	
					 {
						 
						periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk); // always clear interrupt at the start of new state 
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);   // always clear event at the start of new state 
						 
					 }
					
				 if (RADIO_EVENT(EVENTS_DISABLED))
				 	{
						periph_timer_abort(0);   // clear TIMER0 after receiving either SCAN_RSP or CONN_REQ  
						periph_ppi_clear(0);     // clear PPI after receiving either SCAN_RSP or CONN_REQ  
						
						periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk); // always clear interrupt at the start of new state 
          	PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);   // always clear event at the start of new state 

						  nrf_gpio_pin_toggle(21);
						
						if(is_conn_req_for_me())
						 {  
							  NRF_RADIO->TASKS_DISABLE = 1; // to clear TX state from sm_enter_req() 
	              NRF_RADIO->SHORTS = 0;
							  a=0;
							 
							 nrf_gpio_pin_toggle(22);
							 sm_state = STATE_MASTER_TO_SLAVE_RECEIVE; // NEXT state
              
							 //sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2; // NEXT state
							 
							 get_connection_param(); // get connection parameter from CONN_REQ PDU from master 
							 
							 NRF_TIMER0->CC[0] = 0;
						   periph_timer_abort(0);
							 NRF_TIMER0->TASKS_CLEAR = 1;
		           
//               NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE2_Pos));
//							 NVIC_EnableIRQ(TIMER0_IRQn);
//               NRF_TIMER0->PRESCALER = 4;
//               NRF_TIMER0->BITMODE=TIMER_BITMODE_BITMODE_32Bit;
//											
//							 NRF_TIMER0->TASKS_CLEAR = 1;
//							 NRF_TIMER0->EVENTS_COMPARE[2] = 0; 
//							 NRF_TIMER0->CC[2] = 800+woffset; /* timeout for RX abort */
//							 NRF_TIMER0->TASKS_START = 1;
												
//							 NRF_RADIO->TASKS_DISABLE = 1; // to clear TX state from sm_enter_req() 
//	             NRF_RADIO->SHORTS = 0;
							 
							  conn_req_evt_dispatch();
							 
							 next_timeslot_schedule_connection_woffset();
							 
					 
						}
						
						else if (is_scan_req_for_me())
						{ 
              
							 sm_state = STATE_WAIT_FOR_IDLE; // NEXT state 
							
						   periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk); // always clear interrupt at the start of new state
						   PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);  // always clear event at the start of new state 
							 
							 periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk); // always clear interrupt at the start of new state
	             PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);  // always clear event at the start of new state 

	            /* enable disabled interrupt to avoid race conditions */
	             periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
							
							 periph_radio_packet_ptr_set(&ble_scan_rsp_data[0]);  // prepare to send SCAN_RSP 
		      		 periph_radio_shorts_set(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
							
		         	/* wait exactly 150us to send response. NOTE: the Reference manual is wrong */
							 periph_radio_tifs_set(150);
							 
							/* send scan req to user space */
	           	scan_req_evt_dispatch();
							
						}	
						
						else 
							
						{  

							sm_state = STATE_WAIT_FOR_IDLE;  // NEXT state 
							/* enable disabled interrupt to avoid race conditions */
							
							periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk); // always clear interrupt at the start of new state
						  PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);   // always clear event at the start of new state 
							
							periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
							periph_radio_shorts_set(0);

	            PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_DISABLE);	
							
						}
					}
				
					break;

					case STATE_MASTER_TO_SLAVE_RECEIVE_3:
					
					 if(RADIO_EVENT(EVENTS_ADDRESS))  	
					 {

						periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
						 
					 }
					
					if (RADIO_EVENT(EVENTS_DISABLED)) //
					{    

							periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	            PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						
							if (((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x02))  // check for termination end
							 {

											 if(ble_rx_buf[0] == 0x03)
													 ble_terminate_data[0]=0x07;
											else if (ble_rx_buf[0] == 0x0F) 
													ble_terminate_data[0]=0x0B;
										
											ble_terminate_data[4]= ble_rx_buf[4]; 

											
											periph_radio_packet_ptr_set(&ble_terminate_data[0]);
								
											periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
											periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
										
										
											periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
										
											sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_END;	
											
                      disconnected_evt_dispatch();											
										
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x00)/*&& (ble_rx_buf[1]== 0x0C)*/)   // check for CONN_UPDATE_REQ
							 {
							 
								      get_new_connection_perameter () ;
												
											 if(ble_rx_buf[0] == 0x03)
													 ble_linklayer_data[0]=0x05;
											else if (ble_rx_buf[0] == 0x0F) 
													ble_linklayer_data[0]=0x09;

											periph_radio_packet_ptr_set(&ble_linklayer_data[0]);
								
											periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
											periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
										
											periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
										
											sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;	
											
										  conn_update_req_evt_dispatch();
							 
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x01)/*&& (ble_rx_buf[1]== 0x08)*/)   // check for CHANNEL_MAP_REQ
							 {   

										if(ble_rx_buf[0] == 0x03)
													 ble_acknow_data[0]=0x05;
										else if (ble_rx_buf[0] == 0x0F) 
													ble_acknow_data[0]=0x09;

										periph_radio_packet_ptr_set(&ble_acknow_data[0]);
											
										get_new_channel_map_perameter ();

										periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
										periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

										periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
										
										sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;		
									
										channel_map_update_req_evt_dispatch();	
								 
								 
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x0C)/*&& (ble_rx_buf[1]== 0x08)*/)   // check for LL_VERSION_IND
							 { 
								 
								 if((ble_rx_buf[0]& (0x0F)) == 0x03)
								    ble_version_data[0]=0x07;
						     else if ((ble_rx_buf[0] & (0x0F)) == 0x0F) 
								    ble_version_data[0]=0x0B;  
								 
								    	//periph_radio_packet_ptr_set(&ble_terminate_data[0]);
								    periph_radio_packet_ptr_set(&ble_version_data[0]);

										periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
										periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

										periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
										
										sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;
                    	
                    version_evt_dispatch();								 
								 
							 }
							 
							 else if(((ble_rx_buf[0] &(0x03)) == 0x03) && (ble_rx_buf[3]==0x08)/*&& (ble_rx_buf[1]== 0x08)*/)   // check for LL_FEATURE_REQ
							 { 

								 if((ble_rx_buf[0]& (0x0F)) == 0x03)
								    ble_feature_rsp_data[0]=0x07;
						     else if ((ble_rx_buf[0] & (0x0F)) == 0x0F) 
								    ble_feature_rsp_data[0]=0x0B;  

								   periph_radio_packet_ptr_set(&ble_feature_rsp_data[0]);
								
								   periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																						RADIO_SHORTS_END_DISABLE_Msk );
																						//RADIO_SHORTS_DISABLED_TXEN_Msk);
									 periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

									 periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
										
									 sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;	
								 
								   feature_req_evt_dispatch();
								 
							 }
						  else
							{
								

							
//							 if((ble_rx_buf[0] & (0x0C)) == 0x00)
//								ble_linklayer_data[0]=0x05;
//						  else if ((ble_rx_buf[0] & (0x0C))== 0x0C) 
//								ble_linklayer_data[0]=0x09;
//							
//						 	ble_linklayer_data [0] = ble_linklayer_data [0] & 0xEF;//(0b11101111);
//	            ble_linklayer_data [0] = ble_linklayer_data [0] | (MD_bit << 4 );
							
							/* configure radio for master to slave data receive*/
							 
							//disconnect_event_counter = disconnect;
              //disconnect_activate = enable ;
							
					    if ((connection_event_counter >= disconnect_event_counter) && ( disconnect_activate ==true)) 
							{   
								  	if((ble_rx_buf[0] & (0x0C)) == 0x00)
													 ble_terminate_data[0]=0x07;
											else if ((ble_rx_buf[0] & (0x0C))== 0x0C) 
													ble_terminate_data[0]=0x0B;
											
								  periph_radio_packet_ptr_set(&ble_terminate_data[0]);
							    sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_END;
											
									disconnected_evt_dispatch();
							}   
							else 
							{  
								
							if((ble_rx_buf[0] & (0x0C)) == 0x00)
								  ble_linklayer_data[0]=0x05;
						  else if ((ble_rx_buf[0] & (0x0C))== 0x0C) 
								  ble_linklayer_data[0]=0x09;
							
						 	    ble_linklayer_data [0] = ble_linklayer_data [0] & 0xEF;//(0b11101111);
	                ble_linklayer_data [0] = ble_linklayer_data [0] | (MD_bit << 4 );
								  periph_radio_packet_ptr_set(&ble_linklayer_data[0]);
							    sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;
							
							}
						   periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																			RADIO_SHORTS_END_DISABLE_Msk );
																			//RADIO_SHORTS_DISABLED_TXEN_Msk);
						   periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

							 periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);///////address mask
							
						  // sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_3;
							
							}
							
					}
					break;
					 
						
					case	STATE_SLAVE_TO_MASTER_TRANSMIT_END:
						if (RADIO_EVENT(EVENTS_DISABLED))
					 {    
							periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
							 
							periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
							PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						 
						  sm_state = STATE_START_FROM_INITIAL;
								 
							periph_timer_abort(0);
		          NRF_TIMER0->TASKS_CLEAR = 1;
								 
							periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);
							NRF_RADIO->TASKS_DISABLE = 1; // to clear TX state from sm_enter_req() 
	            NRF_RADIO->SHORTS = 0;
					
					}
					
					break;
					
					case STATE_SLAVE_TO_MASTER_TRANSMIT_3:
					{	
					
					if(RADIO_EVENT(EVENTS_ADDRESS))  	
					 {
					
						 
						periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
					 }
					 
					 
					if (RADIO_EVENT(EVENTS_DISABLED))
					{    
						
						
						sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;		
						
						periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	          PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						NRF_RADIO->SHORTS = 0;		

            NRF_TIMER0->TASKS_CAPTURE[1] = 1;
						x1 = NRF_TIMER0->CC[1] ;	
						NRF_TIMER0->TASKS_STOP = 1;		
						missed_counter = 0 ;
						
						connection_event_counter ++ ;
						
						 
						if ((instant > 0)&&(connection_event_counter == instant ))
											{
														   instant = 0 ; 
															 sm_adv_run =1 ;
												       cointerval = new_cointerval ;

											}

						  next_timeslot_schedule_connection_2();
					}
					
				}
					break;
					 
					case STATE_MASTER_TO_SLAVE_RECEIVE_2:
					
					if(RADIO_EVENT(EVENTS_ADDRESS))  	
					 {

						periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
						 
					 }
					
					if (RADIO_EVENT(EVENTS_DISABLED)) 
					{    
						   connection_event_counter= 0 ;
						
//						   periph_timer_abort(0);
//						
//						   NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE1_Pos ));
//						
//							 NRF_TIMER0->TASKS_CLEAR = 1;
						
							
							periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	            PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						
							
						  
//						  if((ble_rx_buf[0] & (0x0F))== 0x01) ble_linklayer_data[0]=0x05;
//						  else if ((ble_rx_buf[0] & (0x0F))== 0x0D) ble_linklayer_data[0]=0x09;
						
						    if((ble_rx_buf[0] & (0x0C))== 0x00) ble_linklayer_data[0]=0x05;
								else if ((ble_rx_buf[0] & (0x0C))== 0x0C) ble_linklayer_data[0]=0x09;
						
						    ble_linklayer_data [0] = ble_linklayer_data [0] & 0xEF;//(0b11101111);
	              ble_linklayer_data [0] = ble_linklayer_data [0] | (MD_bit << 4 );
	
							/* configure radio for master to slave data receive*/
							 periph_radio_packet_ptr_set(&ble_linklayer_data[0]);
					
							periph_radio_shorts_set(	RADIO_SHORTS_READY_START_Msk | 
																			RADIO_SHORTS_END_DISABLE_Msk );
																			//RADIO_SHORTS_DISABLED_TXEN_Msk);
							periph_radio_intenset(	RADIO_INTENSET_DISABLED_Msk);

							periph_radio_intenset(	RADIO_INTENSET_ADDRESS_Msk);
							
						  sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT;
							
					}
					break;
					
				case STATE_SLAVE_TO_MASTER_TRANSMIT:
						
					if(RADIO_EVENT(EVENTS_ADDRESS))  	
					 {
					
						 
						periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
						PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
					 }
					 
					 
					if (RADIO_EVENT(EVENTS_DISABLED))
					{    

						sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;		
						
						periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	          PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						NRF_RADIO->SHORTS = 0;		
            
						NRF_TIMER0->TASKS_CAPTURE[1] = 1;
						x1 = NRF_TIMER0->CC[1] ;	
						NRF_TIMER0->TASKS_STOP = 1;		
						missed_counter = 0 ;
						
						connection_event_counter= 0 ;
						
						next_timeslot_schedule_connection();
						
            				
					}
					break;
					
				case STATE_WAIT_FOR_IDLE:
					if (RADIO_EVENT(EVENTS_DISABLED))
					{
						
						/* state exit function returns whether the adv event is complete */
						bool adv_evt_done = sm_exit_wait_for_idle();
						
						if (adv_evt_done)
						{ 
							nrf_gpio_pin_toggle(20);
							sm_state= STATE_FIRST_ORDER_TIMESLOT;
							next_timeslot_schedule();
						}
						else
						{ 
							sm_enter_adv_send();
						}
					}
					break;	

        case STATE_START_FROM_INITIAL:
				 
					if (RADIO_EVENT(EVENTS_DISABLED))
					{ 
						
						periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
	          PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
						NRF_RADIO->SHORTS = 0;
						
						last_unmapped_channel= 0;
						unmapped_channel= 0 ;
						
						sm_state= STATE_FIRST_ORDER_TIMESLOT;
						next_timeslot_schedule();
						

						
					}
					break;					
			
				default:
					/* Shouldn't happen */
					ASSERT(false);
				}
		}
			break;
		
		
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:/**************************************************************/
		{  
				 switch (sm_state)
							{
								case STATE_REQ: 
									
										if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
													{  
														
														nrf_gpio_pin_toggle(24);
														periph_timer_abort(0);
														NRF_TIMER0->TASKS_CLEAR = 1;
														periph_ppi_clear(0);
														
														periph_radio_intenclr(RADIO_INTENCLR_ADDRESS_Msk);
														PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_ADDRESS);
														
														periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
														PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
														
														sm_state = STATE_WAIT_FOR_IDLE;
														/* enable disabled interrupt to avoid race conditions */
														
														periph_radio_intenset(RADIO_INTENSET_DISABLED_Msk);
														periph_radio_shorts_set(0);						
														
														PERIPHERAL_TASK_TRIGGER(NRF_RADIO->TASKS_DISABLE);
													
													}  
								break;
								
//						  case STATE_MASTER_TO_SLAVE_RECEIVE:
//									 if (NRF_TIMER0->EVENTS_COMPARE[2] != 0)
//										{  

//											//set_base_prefix_crc_with_new_perameter_from_conn_req ();
//											
//											NRF_RADIO->CRCINIT = CRCinit[2];
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[1];
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT)<<8;
//											NRF_RADIO->CRCINIT = (NRF_RADIO->CRCINIT) | CRCinit[0];				
//					
//											NRF_RADIO->PREFIX0	 = AcessA[3]; //0x8e;
//											NRF_RADIO->BASE0 = AcessA[2];
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[1];
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0) | AcessA[0];
//											NRF_RADIO->BASE0 = (NRF_RADIO->BASE0)<<8;
//																
//											NRF_RADIO->TXADDRESS = 0x00;					// Use logical address 0 (prefix0 + base0)  when transmitting
//											NRF_RADIO->RXADDRESSES = 0x01;        // Use logical address 0 (prefix0 + base0)  when receiving
//											
//											sm_enter_master_to_slave_receive();
//										
//										}  
//									 break;	
								case STATE_MASTER_TO_SLAVE_RECEIVE_2:
									 if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
										{ 
											 sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;	
											
											 periph_timer_abort(0);
											 NRF_TIMER0->TASKS_CLEAR = 1;
											 x1=0;

											 missed_counter =0 ;
											


											 connection_event_counter =0 ;



											
						                next_timeslot_schedule_connection_timer();/****************************************************************/
										}
								   break;
										
               case STATE_MASTER_TO_SLAVE_RECEIVE_3:
									 if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
										{ 
											 sm_state = STATE_SLAVE_TO_MASTER_TRANSMIT_2;	
											
											 periph_timer_abort(0);
											 NRF_TIMER0->TASKS_CLEAR = 1;
											 x1=0;

											 missed_counter ++ ;
											
											if (missed_counter == 10 ) 
											{     
												    missed_counter = 0 ;
												
														periph_radio_intenclr(RADIO_INTENCLR_DISABLED_Msk);
														PERIPHERAL_EVENT_CLR(NRF_RADIO->EVENTS_DISABLED);
														NRF_RADIO->SHORTS = 0;
														
														last_unmapped_channel= 0;
														unmapped_channel= 0 ;
														
														sm_state= STATE_FIRST_ORDER_TIMESLOT;
														next_timeslot_schedule();
											}

											 connection_event_counter ++ ;


											if ((instant > 0)&&(connection_event_counter == instant ))
											{
														   instant = 0 ; 
															 sm_adv_run =1 ;
												       cointerval = new_cointerval ;
		
											
											}
											
						                next_timeslot_schedule_connection_timer();/****************************************************************/
										}
								   break;
										
								default:
							// Shouldn't happen /*/**/
				
							ASSERT(false);
							break;
							}
		}
					break;
		
		


		default:
			DEBUG_PIN_POKE(14);
			/* shouldn't happen in this advertiser. */	
      break;		
	}
	
}	




#endif

/***************************************************************************/


void ctrl_init(void)
{
	/* set the contents of advertisement and scan response to something
	that is in line with BLE spec */
	
	/* erase package buffers */
	memset(&ble_adv_data[0], 0, 40);  
#if TS_SEND_SCAN_RSP	
	memset(&ble_scan_rsp_data[0], 0, 40);
#endif 
	memset(&ble_linklayer_data[0], 0, (BLE_DATA_PAYLOAD_OFFSET + BLE_DATA_PAYLOAD_MAXLEN));
	
	
#if TS_SEND_SCAN_RSP	
	/* set message type to ADV_IND_DISC, RANDOM in type byte of adv data */
	ble_adv_data[BLE_TYPE_OFFSET] = 0x46;
	/* set message type to SCAN_RSP, RANDOM in type byte of scan rsp data */
	ble_scan_rsp_data[BLE_TYPE_OFFSET] = 0x44;
#else
	/* set message type to ADV_IND_NONCONN, RANDOM in type byte of adv data */
	ble_adv_data[BLE_TYPE_OFFSET] = 0x42;
#endif
	

	ble_adv_data[BLE_ADFLAG_OFFSET]  = 0x00;
  ble_adv_data[BLE_ADFLAG_OFFSET] |= (1 << 1);  /* General discoverable mode */
  ble_adv_data[BLE_ADFLAG_OFFSET] |= (1 << 2);    /* BR/EDR not supported      */

	/* set message length to only address */
	ble_adv_data[BLE_SIZE_OFFSET] 			= 0x06;
#if TS_SEND_SCAN_RSP
	ble_scan_rsp_data[BLE_SIZE_OFFSET] 	= 0x06;
#endif	
	
	/* generate rng sequence */
	//#if softdevice
	 #if defined(WITH_SOFTDEVICE)
	    //adv_rng_init(rng_pool); ///  ?+??
	#endif
}

bool ctrl_data_param_set(btle_data_channel_data_packet_parameters_t* data_params)
{
	
	ble_linklayer_data [0] = ble_linklayer_data [0] & 0xFC;//(0b11111100);
	ble_linklayer_data [0] = ble_linklayer_data [0] | data_params->LLID;
	ble_linklayer_data [0] = ble_linklayer_data [0] & 0xEF;//(0b11101111);
	ble_linklayer_data [0] = ble_linklayer_data [0] | (data_params->MD << 4 );
	MD_bit = data_params->MD;
	return true;

}

bool ctrl_data_datapayload_set(btle_data_channel_data_packet_data_t* data_payload)
{
  ble_linklayer_data [1] = data_payload->data_length;
  memcpy((void*) &ble_linklayer_data[3],(void*) &data_payload->data_packet_data[0],data_payload->data_length);
	return true;
}

bool ctrl_data_version_data_set(btle_control_packet_version_data_t* version_data)
{
													
		 ble_version_data[4]	= version_data->LLVersNr;		
	   ble_version_data[5]	= (0x00FF) & version_data->CompId;														
     ble_version_data[6]	= (0xFF00) & version_data->CompId;		
		 ble_version_data[7]	= (0x00FF) & version_data->SubVersNr;														
     ble_version_data[8]	= (0xFF00) & version_data->SubVersNr;	
     return true;	

}

bool ctrl_data_feature_rsp_data_set(btle_control_packet_feature_rsp_data_t* feature_rsp_data)
{

   ble_feature_rsp_data[4] = feature_rsp_data->FeatureSet ;
   return true;
}

bool ctrl_disconnect_connection_event_set(uint16_t disconnect, bool enable) 
{

disconnect_event_counter = disconnect;
disconnect_activate = enable ;	
return true;
}


bool ctrl_adv_param_set(btle_cmd_param_le_write_advertising_parameters_t* adv_params)
{
	ASSERT(adv_params != NULL);
	/* Checks for error */

	/* channel map */
	if (0x00 == adv_params->channel_map || 0x07 < adv_params->channel_map)
	{
		return false;
	}

	/* address */
	if (NULL == adv_params->direct_address)
	{
		return false;
	}

	/* set channel map */
	channel_map = adv_params->channel_map;


	/* put address into advertisement packet buffer */
	static uint8_t ble_addr[] = {0x4e, 0x6f, 0x72, 0x64, 0x69, 0x63};   //DEFAULT_DEVICE_ADDRESS;
	memcpy((void*) &ble_adv_data[BLE_ADDR_OFFSET],
				 (void*) &ble_addr[0], BLE_ADDR_LEN);

#if TS_SEND_SCAN_RSP
	/* put address into scan response packet buffer */
	memcpy((void*) &ble_scan_rsp_data[BLE_ADDR_OFFSET], 
					(void*) &adv_params->direct_address[0], BLE_ADDR_LEN);
#endif
	
	/* address type */
	ble_adv_data[BLE_TYPE_OFFSET] &= BLE_ADDR_TYPE_MASK; //0x40  scan response data type 0100 
	ble_adv_data[BLE_TYPE_OFFSET] |= (adv_params->own_address_type) << 6;
#if TS_SEND_SCAN_RSP
	ble_scan_rsp_data[BLE_TYPE_OFFSET] &= BLE_ADDR_TYPE_MASK;
	ble_scan_rsp_data[BLE_TYPE_OFFSET] |= (adv_params->own_address_type) << 6;
#endif
	
	/* whitelist */
	if (BTLE_ADV_FILTER_ALLOW_ANY 		== adv_params->filter_policy || 
			BTLE_ADV_FILTER_ALLOW_LEVEL2 	== adv_params->filter_policy)
	{
		wl_disable();
	}
	else
	{
		wl_enable();
	}

	/* Advertisement interval */
	adv_int_min = adv_params->interval_min;
	adv_int_max = adv_params->interval_max;

#if TS_SEND_SCAN_RSP	
	/* adv type */
	ble_adv_data[BLE_TYPE_OFFSET] &= ~BLE_TYPE_MASK; //0x3F     ADV_SCAN_IND
	ble_adv_data[BLE_TYPE_OFFSET] |= (ble_adv_type_raw[adv_params->type] & BLE_TYPE_MASK);  //0000 0110

	/* scan rsp type */
	ble_scan_rsp_data[BLE_TYPE_OFFSET] &= ~BLE_TYPE_MASK;  //SCAN _RSP
	ble_scan_rsp_data[BLE_TYPE_OFFSET] |= 0x04;
#else
	/* adv type is locked to nonconn */
	ble_adv_data[BLE_TYPE_OFFSET] &= ~BLE_TYPE_MASK;
	ble_adv_data[BLE_TYPE_OFFSET] |= (ble_adv_type_raw[BTLE_ADV_TYPE_NONCONN_IND] & BLE_TYPE_MASK); //ADV_NONCONN_IND
#endif
	return true;
}



bool ctrl_adv_data_set(btle_cmd_param_le_write_advertising_data_t* adv_data)
{	
	ASSERT(adv_data != NULL);
	
	/* length cannot exceed 31 bytes */
	uint8_t len = ((adv_data->data_length <= BLE_PAYLOAD_MAXLEN)? 
									adv_data->data_length : 
									BLE_PAYLOAD_MAXLEN);

	/* put into packet buffer */
	memcpy((void*) &ble_adv_data[BLE_PAYLOAD_OFFSET], //9
					(void*) &adv_data->advertising_data[0], len);

	/* set length of packet in length byte. Account for 6 address bytes */
	ble_adv_data[BLE_SIZE_OFFSET] &= 0x00;
	ble_adv_data[BLE_SIZE_OFFSET] = (BLE_ADDR_LEN + len);

	return true;
}

bool ctrl_scan_data_set(btle_cmd_param_le_write_scan_response_data_t* data)
{
#if TS_SEND_SCAN_RSP	
	ASSERT(data != NULL);

	/* length cannot exceed 31 bytes */
	uint8_t len = ((data->data_length <= BLE_PAYLOAD_MAXLEN)? 
									data->data_length : 
									BLE_PAYLOAD_MAXLEN);

	/* put into packet buffer */
	memcpy((void*) &ble_scan_rsp_data[BLE_PAYLOAD_OFFSET], //9
					(void*) &data->response_data[0], len);

	/* set length of packet in length byte. Account for 3 header bytes
	* and 6 address bytes */
	ble_scan_rsp_data[BLE_SIZE_OFFSET] = (BLE_ADDR_LEN + len); //6 + ?
#endif	

	return true;
}
