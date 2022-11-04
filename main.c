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
#include <signal.h>

/* third party libs */
#include <log.h>

/* macros */
#define UART_BAUD           115200          /**< \brief Default UART baud rate. */
#define FREQ_FRC1           200             /**< \brief Frequency TIMER. */
#define I2C_BUS             0               /**< \brief I2C bus. */
#define ADDR                MCP4725A0_ADDR0 /**< \brief DAC address. */
#define VDD                 3.3             /**< \brief DAC voltage supply. */
#define SYSTEM_VOLTAGE      3.3             /**< \brief Default system voltage */
#define ADC_RESOLUTION      1023            /**< \brief ADC resolution excluding 0 */


/**
 * \brief   I2C configuration structure.
 */
i2c_dev_t dev = {
    .addr = ADDR,
    .bus = I2C_BUS,
};

void adc_task(void *pvParameters);

static volatile uint32_t frc1_count;
volatile uint8_t sample;
uint16_t adc_value;
float adc_voltage;

uint8_t SYSTEM_LOG_LEVEL = LOG_DEBUG;

/**
 * \brief   DAC memory release function.
 */
static void wait_for_eeprom(i2c_dev_t *dev) {
    while (mcp4725_eeprom_busy(dev)) {
        log_debug("...DAC is busy, waiting...");
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/**
 * \brief   Interrupt routine function.
 */
void frc1_interrupt_handler(void *arg) {
    if (sample > N_COS-1) {
        sample = 0;
        mcp4725_set_voltage(&dev, VDD, signal_cos[sample], false);
    }
    mcp4725_set_voltage(&dev, VDD, signal_cos[sample], false);
    gpio_toggle(GPIO_FRC1);
    sample++;
    frc1_count++;
}

void user_init(void) {
    uart_set_baud(0, UART_BAUD);
    log_set_level(SYSTEM_LOG_LEVEL);

    sample = 0;

    /* I2C initialization */
    i2c_init(I2C_BUS, GPIO_SCL, GPIO_SDA, I2C_FREQ_400K);

    /* DAC setup */
    if (mcp4725_get_power_mode(&dev, true) != MCP4725_PM_NORMAL) {
        mcp4725_set_power_mode(&dev, MCP4725_PM_NORMAL, true);
        wait_for_eeprom(&dev);
    }

    /* GPIO setup */
    gpio_enable(GPIO_FRC1, GPIO_OUTPUT);
    gpio_write(GPIO_FRC1, true);

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

    gpio_write(GPIO_FRC1, false);
    TaskHandle_t xHandle1 = NULL;
    BaseType_t xReturned;
    xReturned = xTaskCreate(&adc_task,
                            "adc_task",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            tskIDLE_PRIORITY+1,
                            &xHandle1);
    if (xReturned == pdPASS) {
        log_trace("Task for adc is created");
    } else {
        log_error("Could not allocate memory for adc task");
    }
}

void adc_task(void *pvParameters) {
    while (true) {
        adc_value = sdk_system_adc_read();
        adc_voltage = adc_value*(SYSTEM_VOLTAGE/ADC_RESOLUTION);
        log_debug("ADC Voltage: %.1f", adc_voltage);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    } 
}
