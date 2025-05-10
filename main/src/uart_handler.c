#include "uart_handler.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "uart2_driver";

// UART2 configuration
#define UART_NUM UART_NUM_2
#define TXD_PIN 17 // Change to your preferred TX pin
#define RXD_PIN 16 // Change to your preferred RX pin
#define BAUD_RATE 115200
#define BUF_SIZE 1024

esp_err_t uart2_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Configure UART parameters
    esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters");
        return ret;
    }

    // Set pins
    ret = uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins");
        return ret;
    }

    // Install UART driver
    ret = uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return ret;
    }

    ESP_LOGI(TAG, "UART2 initialized successfully");
    return ESP_OK;
}

int uart2_send_data(const void *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        ESP_LOGW(TAG, "Invalid data or length");
        return -1;
    }

    int bytes_written = uart_write_bytes(UART_NUM, data, len);
    if (bytes_written < 0)
    {
        ESP_LOGE(TAG, "Failed to write data to UART2");
    }

    return bytes_written;
}

int uart2_read_data(void *data, size_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0)
    {
        ESP_LOGW(TAG, "Invalid data buffer or length");
        return -1;
    }

    int bytes_read = uart_read_bytes(UART_NUM, data, len, pdMS_TO_TICKS(timeout_ms));
    if (bytes_read < 0)
    {
        ESP_LOGE(TAG, "Failed to read data from UART2");
    }

    return bytes_read;
}