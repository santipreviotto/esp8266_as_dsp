/*
 * Copyright 2022 Santiago Previotto.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
/** \file main.c */
/* standard */
#include <string.h>

/* third party libs */
#include <FreeRTOS.h>
#include <task.h>
#include <espressif/esp_common.h>
#include <espressif/user_interface.h>
#include <esp/uart.h>

/* third party libs */
#include <log.h>

/* configuration includes */
#include <pinout_configuration.h>

/* macros */
#define UART_BAUD           115200  /**< \brief Default UART baud rate. */

/* system level for logs */
uint8_t SYSTEM_LOG_LEVEL = LOG_INFO;

/**
 * \brief   It's an example task.
 */
void sample_task(void *pvParameters) {
    for (;;) {
        log_info("Sample task executing ...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * \brief   Program entrypoint.
 */
void user_init(void) {
    uart_set_baud(0, UART_BAUD);
    log_set_level(SYSTEM_LOG_LEVEL);
    log_info("SDK version: %s ", sdk_system_get_sdk_version());

    /* initialize tasks */
    xTaskCreate(&sample_task, "sample task", 256, NULL, 2, NULL);
}
