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
#define UART_BAUD           115200                      /**< \brief Default UART baud rate. */
#define FREQ_FRC1           1000                         /**< \brief Frequency TIMER. */
#define I2C_BUS             0                           /**< \brief I2C bus. */
#define ADDR                MCP4725A0_ADDR0             /**< \brief DAC address. */
#define SYSTEM_VOLTAGE      3.3                         /**< \brief Default system voltage */
#define ADC_RESOLUTION      1023                        /**< \brief ADC resolution excluding 0 */
#define PERIOD_MS           10                         /**< \brief ADC time in milliseconds. */
#define PERIOD              pdMS_TO_TICKS(PERIOD_MS)    /**< \brief ADC time in ticks. */
#define N                   10

/**
 * \brief   I2C configuration structure.
 */
i2c_dev_t dev = {
    .addr = ADDR,
    .bus = I2C_BUS,
};

volatile uint32_t frc1_count;
volatile uint16_t adc_value;
volatile float adc_voltage;
volatile float input[N];
volatile float filter_output;
volatile float add;

uint8_t SYSTEM_LOG_LEVEL = LOG_DEBUG;

/**
 * \brief   DAC memory release function.
 */
void wait_for_eeprom(i2c_dev_t *dev) {
    TickType_t xPeriodicity =  PERIOD;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (mcp4725_eeprom_busy(dev)) {
        log_debug("DAC is busy, waiting...");
        vTaskDelayUntil(&xLastWakeTime , xPeriodicity);
    }
}

volatile float filter_calc() {
    input[0] = adc_voltage;
    add = 0;
    for (int i = 0; i < N; i ++) {
        add = add + input[i];
    }
    filter_output = add / N;
    for (int i = 0; i < N-1; i ++) {
        input[i+1] = input[i];
    }
    return filter_output;
}

/**
 * \brief   Interrupt routine function.
 */
void frc1_interrupt_handler(void *arg) {
    adc_value = sdk_system_adc_read();
    adc_voltage = adc_value*(SYSTEM_VOLTAGE/ADC_RESOLUTION);
    /*
    input[0] = adc_voltage;
    add = 0;
    for (int i = 0; i < N; i ++) {
        add = add + input[i];
    }
    filter_output = add / N;
    for (int i = 0; i < N-1; i ++) {
        input[i+1] = input[i];
    }
    */
    filter_output = filter_calc();
    mcp4725_set_voltage(&dev, SYSTEM_VOLTAGE, filter_output, false);
    frc1_count++;
}

void user_init(void) {
    uart_set_baud(0, UART_BAUD);
    log_set_level(SYSTEM_LOG_LEVEL);

    for (int i = 0; i < N; i ++) {
        input[i] = 0;
    }

    /* I2C initialization */
    i2c_init(I2C_BUS, GPIO_SCL, GPIO_SDA, I2C_FREQ_400K);

    /* DAC setup */
    if (mcp4725_get_power_mode(&dev, true) != MCP4725_PM_NORMAL) {
        mcp4725_set_power_mode(&dev, MCP4725_PM_NORMAL, true);
        wait_for_eeprom(&dev);
    }

    /* stop timer and mask their interrupt as a precaution */
    timer_set_interrupts(FRC1, false);
    timer_set_run(FRC1, false);

    /* ISR setup */
    _xt_isr_attach(INUM_TIMER_FRC1, frc1_interrupt_handler, NULL);

    /* configure timer frequencie */
    timer_set_frequency(FRC1, FREQ_FRC1);

    /* unmask interrupt and start timer */
    timer_set_interrupts(FRC1, true);
    timer_set_run(FRC1, true);
}
