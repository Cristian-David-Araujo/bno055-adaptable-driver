/**
 * @file platform_esp32s3.c
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @brief 
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "platform_esp32s3.h"

/**
 * @brief Find UART number for the ESP32-S3 microcontroller
 * 
 * @param gpio_tx 
 * @param gpio_rx 
 * @param gpio_rts 
 * @param gpio_cts 
 * @return uart_port_t 
 */
static uart_port_t find_uart_num(uint8_t gpio_tx, uint8_t gpio_rx, uint8_t gpio_rts, uint8_t gpio_cts)
{
    // UART number for the ESP32-S3 microcontroller
    if (gpio_tx == 43 && gpio_rx == 44)
    {
        return UART_NUM_0;
    }
    else
    {
        return UART_NUM_1;
    }

    return UART_NUM_MAX;
}

bool uart_init(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, uint8_t gpio_tx, uint8_t gpio_rx, uint8_t gpio_rts, uint8_t gpio_cts)
{
    // Check if the UART gpio pins are valid
    if (gpio_tx == UART_PIN_NO_USE || gpio_rx == UART_PIN_NO_USE)
    {
        ESP_LOG("UART_INIT", "Invalid UART TX or RX pin");
        return false;
    }
    else if ((gpio_tx != 43 && gpio_rx != 44) || (gpio_tx != 17 && gpio_rx != 18))
    {
        ESP_LOG("UART_INIT", "Invalid UART TX or RX pin");
        return false;
    }

    
    // Find UART number for the ESP32-S3 microcontroller
    uart_port_t uart_num = find_uart_num(gpio_tx, gpio_rx, gpio_rts, gpio_cts);

    // Configure UART parameters
    uart_config->baud_rate = baud_rate;
    uart_config->buffer_size = buffer_size;
    uart_config->uart_num = uart_num;
    uart_config->gpio_tx = gpio_tx;
    uart_config->gpio_rx = gpio_rx;
    uart_config->gpio_rts = gpio_rts;
    uart_config->gpio_cts = gpio_cts;

    // Configure UART configuration structure
    uart_config->uart_config.baud_rate = baud_rate;
    uart_config->uart_config.data_bits = UART_DATA_8_BITS;
    uart_config->uart_config.parity = UART_PARITY_DISABLE;
    uart_config->uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config->uart_config.flow_ctrl = (gpio_rts != UART_PIN_NO_USE && gpio_cts != UART_PIN_NO_USE) 
                                ? UART_HW_FLOWCTRL_CTS_RTS 
                                : UART_HW_FLOWCTRL_DISABLE;
    uart_config->uart_config.source_clk = UART_SCLK_DEFAULT;

    


    // Configure UART parameters and check if UART was initialized successfully
    if (uart_param_config(uart_num, uart_config) != ESP_OK)
    {
        ESP_LOG("UART_INIT", "Failed to configure UART parameters");
        return false;
    }

    // Configure UART pins
    if (uart_set_pin(uart_num, gpio_tx, gpio_rx, gpio_rts, gpio_cts) != ESP_OK)
    {
        ESP_LOG("UART_INIT", "Failed to configure UART pins");
        return false;
    }

    // Install UART driver
    if (uart_driver_install(uart_num, buffer_size, 0, 0, NULL, 0) != ESP_OK)
    {
        ESP_LOG("UART_INIT", "Failed to install UART driver");
        return false;
    }

    ESP_LOG("UART_INIT", "UART initialized successfully");
    return true;
}
