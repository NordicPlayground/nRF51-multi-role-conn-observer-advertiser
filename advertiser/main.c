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


#include "nrf_adv_conn.h"


#include "boards.h"
#include "nrf_sdm.h"
#include "app_error.h"  
#include "app_uart.h"
#include "nrf_advertiser.h"
#include "nrf_assert.h"

#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Disable logging to UART by commenting out this line. It is recomended to do this if
 * if you want to study timing in the example using a logic analyzer.
 */
#define USE_UART_LOGGING
#define UART_RX_BUF_SIZE  2
#define UART_TX_BUF_SIZE  64

/**@brief Macro defined to output log data on the UART or not, based on the USE_UART_LOGGING flag.
 * If logging is disabled, it will just yield a NOP instruction.
 */
#ifdef USE_UART_LOGGING
#define __LOG(F, ...) (test_logf("TIMESLOT_TEST_LOG: %s: %d: " #F "\r\n", __FILE__, __LINE__, ##__VA_ARGS__))
#else
  #define __LOG(F, ...) (void)__NOP()
#endif

#define BLE_ADV_INTERVAL_100MS 0x00A0
#define BLE_ADV_INTERVAL_150MS 0x00F0

/*****************************************************************************
* Static Globals
*****************************************************************************/

/**@brief Global variables used for storing assert information from the SoftDevice.
 */
static uint32_t g_sd_assert_line_num;
static uint32_t g_sd_assert_pc;
static uint8_t  g_sd_assert_file_name[100];

/**@brief Global variables used for storing assert information from the nRF51 SDK.
 */
static uint32_t g_nrf_assert_line_num;
static uint8_t  g_nrf_assert_file_name[100];

/**@brief Global variables used for storing assert information from the timeslot event handler.
 */
static uint32_t g_evt;

/* BLE on-air address */
static uint8_t ble_addr[] = {0x5a, 0x0c, 0xbd, 0x6e, 0x2d, 0xdf};

/* Advertisement data fed to the tsa. Keeps the complete local name of the device */
static uint8_t ble_adv_data[] =
{
  0x0D,                   /* length */
  0x09,                   /* complete local name */
  'T', 'i', 'm', 'e', 's', 'l', 'o', 't', ' ', 'a', 'd', 'v',
};

/*****************************************************************************
* Static Functions
*****************************************************************************/

static void uart_putstring(const uint8_t * string);

/**@brief Callback handlers
 */
/**
* Assert callback for Softdevice. Sends message over USB UART (at baud38400, 8n1, RTS/CTS)
* indicating file and line number. Is blocking, as the timeslot API probably would not be
* able to recover from this. Note that an assert in Include/softdevice_timeslot.c, line 192
* is most probably related to a prematurely aborted timeslot 
*/
static void sd_assert_cb(uint32_t pc, uint16_t line_num, const uint8_t *file_name);

/**
* App error check callback. Something went wrong, and we go into a blocking state, as continued 
* executing may lead to undefined behavoiur.
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
  char buf[256];
  
  sprintf(&buf[0], "ERROR: 0x%X, line: %d, file: %s\n", error_code, line_num, p_file_name); 
  uart_putstring((uint8_t* ) buf);
  nrf_gpio_pin_clear(LED_1);
  nrf_gpio_pin_clear(LED_2);
  while(1);
}

/**
 * UART event handler function.
 */
static void uart_event_handler(app_uart_evt_t * uart_event)
{
#ifdef USE_UART_LOGGING
  int status;

  switch(uart_event->evt_type)
  {
    case APP_UART_DATA_READY:
      do {
        uint8_t c;
        status = app_uart_get(&c); // Discard input
      } while(status == NRF_SUCCESS);
      break;
    case APP_UART_FIFO_ERROR:
      APP_ERROR_HANDLER(uart_event->data.error_code);
      break;
    case APP_UART_COMMUNICATION_ERROR:
      APP_ERROR_HANDLER(uart_event->data.error_communication);
      break;
    case APP_UART_DATA:
    default:
      break;
  }
#endif
}

/**@brief Local function prototypes.
 */
static void test_logf(const char *fmt, ...);
static void ble_setup(void);

/**
* Set up all advertiser parameters and data before the advertisement start
*/
static void ble_setup(void)
{
  /* we use software interrupt 0 */
  btle_hci_adv_init(SWI0_IRQn);
  
  btle_cmd_param_le_write_advertising_parameters_t adv_params;
  
  
  memcpy((void*) &adv_params.direct_address[0], (void*) &ble_addr[0], BTLE_DEVICE_ADDRESS__SIZE);
  
  
  /* want to maximize potential scan requests */
  adv_params.channel_map = BTLE_CHANNEL_MAP_ALL;
  adv_params.direct_address_type = BTLE_ADDR_TYPE_RANDOM;
  adv_params.filter_policy = BTLE_ADV_FILTER_ALLOW_ANY;
  adv_params.interval_min = BLE_ADV_INTERVAL_100MS;
  adv_params.interval_max = BLE_ADV_INTERVAL_150MS;
  
  adv_params.own_address_type = BTLE_ADDR_TYPE_RANDOM;
  
  /* Only want scan requests */
  adv_params.type = BTLE_ADV_TYPE_SCAN_IND;

  btle_hci_adv_params_set(&adv_params);
  
  
  
  btle_cmd_param_le_write_advertising_data_t adv_data;
  
  /* Only advertising name of unit */
  memcpy((void*) &adv_data.advertising_data[0], (void*) &ble_adv_data[0], sizeof(ble_adv_data));
  adv_data.data_length = sizeof(ble_adv_data);
  
  btle_hci_adv_data_set(&adv_data);
  
  
  
  btle_cmd_param_le_write_scan_response_data_t scan_rsp_data;
  
  memcpy((void*) &scan_rsp_data.response_data[0], (void*) &ble_adv_data[0], sizeof(ble_adv_data));
  scan_rsp_data.data_length = sizeof(ble_adv_data);
  
  btle_hci_adv_scan_rsp_data_set(&scan_rsp_data);
  
  /* all parameters are set up, enable advertisement */
  btle_hci_adv_enable(BTLE_ADV_ENABLE);
}

static void uart_init(void)
{
#ifdef USE_UART_LOGGING
  int status = NRF_SUCCESS;
  const app_uart_comm_params_t uart_params = {
    .rx_pin_no = RX_PIN_NUMBER,
    .tx_pin_no = TX_PIN_NUMBER,
    .rts_pin_no = RTS_PIN_NUMBER,
    .cts_pin_no = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
    .use_parity = false,
    .baud_rate = UART_BAUDRATE_BAUDRATE_Baud38400
  };
  APP_UART_FIFO_INIT(&uart_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_event_handler, APP_IRQ_PRIORITY_LOW, status);
  APP_ERROR_CHECK(status);
#endif
}

static void uart_putstring(const uint8_t * string)
{
#ifdef USE_UART_LOGGING
  for(int i = 0; string[i] != 0; ++i)
    while(app_uart_put(string[i]) != NRF_SUCCESS);
#endif
}

/**
* Interrupt handler for Softdevice events
*/
void SD_EVT_IRQHandler(void)
{
  uint32_t evt;
  ble_evt_t ble_evt;
  uint16_t len;
  
  while(sd_evt_get(&evt) == NRF_SUCCESS)
  {
    btle_hci_adv_sd_evt_handler(evt);
  }
  
  while (sd_ble_evt_get((uint8_t *) &evt, &len) == NRF_SUCCESS)
  {
    nrf_adv_conn_evt_handler(&ble_evt);
  }
}

int main(void)
{
  /* Silence the compiler */
  (void) g_sd_assert_pc;
  (void) g_evt;

  uart_init();

  char start_msg[128];
  sprintf(&start_msg[0], "\n| %s |---------------------------------------------------\n\n", __TIME__);
  uart_putstring((uint8_t*) &start_msg[0]);

  nrf_gpio_range_cfg_output(0, 30);
#if defined(BOARD_PCA10028)
  nrf_gpio_pin_set(LED_1);
  nrf_gpio_pin_set(LED_2);
  nrf_gpio_pin_set(LED_3);
  nrf_gpio_pin_set(LED_4);
#endif
 
    uint32_t error_code = sd_softdevice_enable((uint32_t)NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_cb);
  APP_ERROR_CHECK(error_code);
 
 
  error_code = sd_nvic_EnableIRQ(SD_EVT_IRQn);
  APP_ERROR_CHECK(error_code);

  ble_setup();
 
  /* Enable a generic SD advertiser to display concurrent operation */
  nrf_adv_conn_init();

  while (true)
  {
    sd_app_evt_wait();
  }
}

/**@brief Assert callback handler for SoftDevice asserts. */
void sd_assert_cb (uint32_t pc, uint16_t line_num, const uint8_t *file_name)
{
  g_sd_assert_line_num = line_num;
  g_sd_assert_pc = pc;
  memset ((void*)g_sd_assert_file_name, 0x00, sizeof(g_sd_assert_file_name));
  (void) strncpy ((char*) g_sd_assert_file_name, (const char*) file_name, sizeof(g_sd_assert_file_name) - 1);

  nrf_gpio_pin_clear(LED_1);
  
   __LOG("%s: SOFTDEVICE ASSERT: line = %d file = %s", __FUNCTION__, g_sd_assert_line_num, g_sd_assert_file_name);

  while(1);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name)
{
  g_nrf_assert_line_num = line_num;
  memset((void*)g_nrf_assert_file_name, 0x00, sizeof (g_nrf_assert_file_name));
  (void) strncpy ((char*) g_nrf_assert_file_name, (const char*) file_name, sizeof (g_nrf_assert_file_name) - 1);

  nrf_gpio_pin_clear(LED_2);

   __LOG("%s: NRF ASSERT: line = %d file = %s", __FUNCTION__, g_nrf_assert_line_num, g_nrf_assert_file_name);

  while (1);
}

/**
* Interrupt handler for advertiser reports
*/
void SWI0_IRQHandler(void)
{
  nrf_report_t report;
  while(btle_hci_adv_report_get(&report))
  {
    char buf[128];
    switch (report.event.event_code)
    {
      /* Send data about scan requests to terminal */
      case BTLE_VS_EVENT_NRF_LL_EVENT_SCAN_REQ_REPORT:
        sprintf(buf, "Received scan req on ch. %i. Addr: %X:%X:%X:%X:%X:%X \tRSSI: -%i \t Packets (valid/invalid): %i/%i\n", 
          report.event.params.nrf_scan_req_report_event.channel,
          report.event.params.nrf_scan_req_report_event.address[5],
          report.event.params.nrf_scan_req_report_event.address[4],
          report.event.params.nrf_scan_req_report_event.address[3],
          report.event.params.nrf_scan_req_report_event.address[2],
          report.event.params.nrf_scan_req_report_event.address[1],
          report.event.params.nrf_scan_req_report_event.address[0],
          report.event.params.nrf_scan_req_report_event.rssi,
          report.valid_packets,
          report.invalid_packets);
        uart_putstring((uint8_t*) buf);
        break;
      
      /* For now, the only event we care about is the scan req event. */
      default:
        uart_putstring((uint8_t*) "Unknown adv evt.\n");
    }
  }
}

/**@brief Logging function, used for formated output on the UART.
 */
void test_logf(const char *fmt, ...)
{
  int16_t res = 0;
  static uint8_t buf[150];
  
  va_list args;
  va_start(args, fmt);
  
  res = vsnprintf((char*) buf, sizeof(buf), fmt,  args);
  ASSERT(res >= 0 && res <= (sizeof buf) - 1);

  uart_putstring(buf);
  
  va_end(args);
}
