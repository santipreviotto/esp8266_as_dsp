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
#include <stdio.h>

/* third party libs */
#include <espressif/esp_common.h>
#include <espressif/user_interface.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <esp8266.h>
#include <i2c/i2c.h>
#include <mcp4725/mcp4725.h>

/* configuration includes */
#include <pinout_configuration.h>

/* third party libs */
#include <log.h>

/* macros */
#define UART_BAUD           115200  /**< \brief Default UART baud rate. */
#define FREQ_FRC1           5000    /**< \brief Frequency TIMER. */

static volatile uint32_t frc1_count;

uint8_t SYSTEM_LOG_LEVEL = LOG_DEBUG;

void frc1_interrupt_handler(void *arg) {
    gpio_toggle(GPIO_FRC1);
    frc1_count++;
}

void user_init(void) {
    uart_set_baud(0, UART_BAUD);
    log_set_level(SYSTEM_LOG_LEVEL);

    /* configure GPIOs */
    gpio_enable(GPIO_FRC1, GPIO_OUTPUT);
    gpio_write(GPIO_FRC1, true);

    /* stop both timers and mask their interrupts as a precaution */
    timer_set_interrupts(FRC1, false);
    timer_set_run(FRC1, false);

    /* set up ISRs */
    _xt_isr_attach(INUM_TIMER_FRC1, frc1_interrupt_handler, NULL);

    /* configure timer frequencies */
    timer_set_frequency(FRC1, FREQ_FRC1);

    /* unmask interrupts and start timers */
    timer_set_interrupts(FRC1, true);
    timer_set_run(FRC1, true);

    gpio_write(GPIO_FRC1, false);
}
