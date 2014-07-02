/** @file
 * Interface functions for the UART serial interface
 *
 * @author Ole Saether
 * @author Asbjørn Sæbø
 * @defgroup nordic_uart UART HAL
 * @{
 * @ingroup nordic_hal
 */
#ifndef HAL_UART_H__
#define HAL_UART_H__

#include <stdint.h>
#include <stdbool.h>

/* Define the following, if the UART1 is used as console, else it will be UART0. */
//#define HAL_UART_SERIAL1

/** Available Baud rates for UART. */
typedef enum
{
    HAL_UART_BAUD_2K4 = 0,        ///< 2400 baud
    HAL_UART_BAUD_4K8,            ///< 4800 baud
    HAL_UART_BAUD_9K6,            ///< 9600 baud
    HAL_UART_BAUD_14K4,           ///< 14.4 kbaud
    HAL_UART_BAUD_19K2,           ///< 19.2 kbaud
    HAL_UART_BAUD_28K8,           ///< 28.8 kbaud
    HAL_UART_BAUD_38K4,           ///< 38.4 kbaud
    HAL_UART_BAUD_57K6,           ///< 57.6 kbaud
    HAL_UART_BAUD_76K8,           ///< 76.8 kbaud
    HAL_UART_BAUD_115K2,          ///< 115.2 kbaud
    HAL_UART_BAUD_230K4,          ///< 230.4 kbaud
    HAL_UART_BAUD_250K0,          ///< 250.0 kbaud
    HAL_UART_BAUD_500K0,          ///< 500.0 kbaud
    HAL_UART_BAUD_1M0,            ///< 1 mbaud
    HAL_UART_BAUD_TABLE_MAX_SIZE  ///< Used to specify the size of the baudrate table.
} hal_uart_baudrate_t;

/** @brief The baudrate devisors array, calculated for standard baudrates.
    Number of elements defined by ::HAL_UART_BAUD_TABLE_MAX_SIZE*/
#define UART_BAUDRATE_DEVISORS_ARRAY    { \
    0x0009D000, 0x0013B000, 0x00275000, 0x003B0000, 0x004EA000, 0x003AF000, 0x009D5000, \
    0x00EBE000, 0x013A9000, 0x01D7E000, 0x03AFB000, 0x04000000, 0x08000000, 0x10000000  }


/** @brief This function initializes the UART.
 *
 * @param baud_rate - the baud rate to be used, ::hal_uart_baudrate_t.
 *
 */
void
hal_uart_init(hal_uart_baudrate_t     const baud_rate);

/** Function to write a character to the UART transmit buffer.
 * @param ch Character to write
 */
void hal_uart_putchar(uint8_t ch);

/** @brief Find number of characters in the UART receive buffer
 *
 * This function returns the number of characters available for reading
 * in the UART receive buffer.
 * 
 * @return Number of characters available
 */
bool hal_uart_chars_available(void);

/** Function to read a character from the UART receive buffer.
 * @return Character read
 */
uint8_t hal_uart_getchar(void);

/** @brief This function checks whether Tx buffer is empty (Tx completed)
 *
 * @return  TRUE if Tx buffer is empty.
 * @return  FALSE if Tx buffer is not empty.
 */
bool hal_uart_tx_buf_empty(void);

/** @brief Callback function that will be called to notify t0 whenever a new
 *         data byte has been received.
 *
 * @param rx_byte - the received data byte.
 *
 * @return  none.
 */
//void hal_uart_byte_received_t0_callback(void);

/** @brief Callback function that will be called to ackowledge t0 that a data
 *         byte has been transmitted.
 *
 * @param none.
 *
 * @return none.
 */
//void hal_uart_byte_transmitted_t0_callback(void);

/** @brief Callback function that will be called to notify t1 whenever a new
 *         data byte has been received.
 *
 * @param rx_byte - the received data byte.
 *
 * @return  none.
 */
//#define hal_uart_byte_received_t1_callback()  /**/ 
//void hal_uart_byte_received_t1_callback(void);

/** @brief Callback function that will be called to ackowledge t1 that a data
 *         byte has been transmitted.
 *
 * @param none.
 *
 * @return none.
 */
//void hal_uart_byte_transmitted_t1_callback(void);

#endif // HAL_UART_H__
/** @} */
