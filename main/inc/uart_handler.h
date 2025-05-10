#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/**
 * @brief Initialize UART2 with default configuration
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t uart2_init(void);

/**
 * @brief Send data through UART2
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return Number of bytes written, -1 on error
 */
int uart2_send_data(const void *data, size_t len);

/**
 * @brief Read data from UART2
 *
 * @param data Buffer to store data
 * @param len Maximum number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes read, -1 on error
 */
int uart2_read_data(void *data, size_t len, uint32_t timeout_ms);

#endif /* UART_HANDLER_H */