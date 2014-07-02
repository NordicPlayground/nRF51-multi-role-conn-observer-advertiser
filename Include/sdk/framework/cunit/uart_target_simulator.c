/** @file
 *
 * @author Ole Saether
 * @author Asbjørn Sæbø
 */
#include <stdint.h>
#include <stdio.h>
#include "uart.h"

#include "nrf.h"

void  sendchar(int ch)  /* in Serial.c */
{
  hal_uart_putchar(ch);
}
int  getkey(void){return 0;}      /* in Serial.c */
long timeval;           /* in Time.c   */

/** @brief The baudrate settings. Use @ref hal_uart_baudrate_t as index.*/
static const uint32_t m_baudrates[HAL_UART_BAUD_TABLE_MAX_SIZE] = UART_BAUDRATE_DEVISORS_ARRAY;

void hal_uart_init(hal_uart_baudrate_t     const baud_rate)
{
  NRF_SYSCONF->PERPOWER |= (1 << (unsigned int)UART0_IRQn);

  NRF_UART0->PSELRTS = 0;
  NRF_UART0->PSELTXD = 1;
  NRF_UART0->PSELCTS = 2;
  NRF_UART0->PSELRXD = 3;

  NRF_UART0->ENABLE           = 0x04;       // Enable uart
  NRF_UART0->BAUDRATE         = m_baudrates[baud_rate];
  NRF_UART0->TASKS_STARTTX    = 1;
  NRF_UART0->TASKS_STARTRX    = 1;
  NRF_UART0->EVENTS_RX  = 0;
}

bool hal_uart_tx_buf_empty(void)
{
  return true;
}

void hal_uart_putchar(uint8_t ch)
{
  NRF_UART0->TXD = ch;
  while(NRF_UART0->EVENTS_TX!=1){}  // Wait for TXD data to be sent
  NRF_UART0->EVENTS_TX=0;
}

uint8_t hal_uart_getchar(void)
{
  while(NRF_UART0->EVENTS_RX!=1){}  // Wait for RXD data to be received
  NRF_UART0->EVENTS_RX=0;
  return (uint8_t)NRF_UART0->RXD;
}

bool hal_uart_chars_available(void)
{
  return NRF_UART0->EVENTS_RX == 1;
}
