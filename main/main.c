/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"


#include "bno055.h"
#include "platform_esp32s3.h"

// UART configuration structure
uart_t uart_config;



void app_main(void)
{
    uart_init(&uart_config, 115200, 1024, 17, 18, UART_PIN_NO_USE, UART_PIN_NO_USE);

    
    
}
