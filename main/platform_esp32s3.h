/**
 * @file platform_esp32s3.h
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @brief 
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PLATFORM_ESP32S3_H
#define PLATFORM_ESP32S3_H

// Standard C includes
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// ESP32-S3 specific includes
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"

// Constants for UART configuration
#define UART_PIN_NO_USE UART_PIN_NO_CHANGE // Do not use pin for UART configuration

// Structure for harware abstraction of UART configuration
typedef struct {
    uint32_t baud_rate;         // Baud rate for UART communication
    uint16_t buffer_size;       // Buffer size for UART communication
    uart_port_t uart_num;       // UART port number
    uint8_t gpio_tx;            // GPIO pin for UART TX
    uint8_t gpio_rx;            // GPIO pin for UART RX
    uint8_t gpio_rts;           // GPIO pin for UART RTS
    uint8_t gpio_cts;           // GPIO pin for UART CTS
    
    uart_config_t uart_config;  // UART configuration structure
    uart_event_t uart_event;    // UART event structure
    uint32_t event_queue_size;  // Event queue size for UART communication
    QueueHandle_t event_queue;  // Event queue for UART communication
} uart_t;

/**
 * @brief Initialize UART peripheral with custom configuration
 * 
 * @param uart_config UART configuration structure
 * @param baud_rate Baud rate for UART communication
 * @param buffer_size Buffer size for UART communication
 * @param gpio_tx GPIO pin for UART TX
 * @param gpio_rx GPIO pin for UART RX
 * @param gpio_rts GPIO pin for UART RTS
 * @param gpio_cts GPIO pin for UART CTS
 * 
 * @return true if UART initialization was successful
 * @return false if UART initialization failed
 */
bool uart_init(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, uint8_t gpio_tx, uint8_t gpio_rx, uint8_t gpio_rts, uint8_t gpio_cts);

/**
 * @brief Initialize UART peripheral with default configuration
 * 
 * @param uart_t UART configuration structure
 * @param baud_rate Baud rate for UART communication
 * @param buffer_size Buffer size for UART communication
 * @param gpio_tx GPIO pin for UART TX
 * @param gpio_rx GPIO pin for UART RX
 * 
 * @return true if UART initialization was successful
 * @return false if UART initialization failed
 */
bool uart_init_with_defaults(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, uint8_t gpio_tx, uint8_t gpio_rx) {
    return uart_init(uart_config, baud_rate, buffer_size, gpio_tx, gpio_rx, UART_PIN_NO_USE, UART_PIN_NO_USE);
}


#endif // PLATFORM_ESP32S3_H