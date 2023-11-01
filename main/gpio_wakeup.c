/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

/* Most development boards have "boot" button attached to GPIO0.
 * You can also change this to another pin.
 */
#define BOOT_BUTTON_NUM         9
#define BOOT_BUTTON_NUM1        10
#define BOOT_BUTTON_NUM2        11
#define BOOT_BUTTON_NUM3        12
#define BOOT_BUTTON_NUM4        13
#define BOOT_BUTTON_NUM5        14

/* Use boot button as gpio input */
#define GPIO_WAKEUP_NUM         BOOT_BUTTON_NUM
#define GPIO_WAKEUP_NUM1        BOOT_BUTTON_NUM1
#define GPIO_WAKEUP_NUM2        BOOT_BUTTON_NUM2
#define GPIO_WAKEUP_NUM3        BOOT_BUTTON_NUM3
#define GPIO_WAKEUP_NUM4        BOOT_BUTTON_NUM4
#define GPIO_WAKEUP_NUM5        BOOT_BUTTON_NUM5
/* "Boot" button is active low */
#define GPIO_WAKEUP_LEVEL       0
#define GPIO_WAKEUP_LEVEL1      1

static const char *TAG = "gpio_wakeup";
uint8_t wakeup_gpio;

uint8_t example_wait_gpio_inactive(void)
{
    // printf("Waiting for GPIO%d to go high...\n", GPIO_WAKEUP_NUM);
    if (gpio_get_level(GPIO_WAKEUP_NUM) == GPIO_WAKEUP_LEVEL) {
        wakeup_gpio = GPIO_WAKEUP_NUM;
    }
    if (gpio_get_level(GPIO_WAKEUP_NUM1) == GPIO_WAKEUP_LEVEL1) {
        wakeup_gpio = GPIO_WAKEUP_NUM1;
    }
    if (gpio_get_level(GPIO_WAKEUP_NUM2) == GPIO_WAKEUP_LEVEL1) {
        wakeup_gpio = GPIO_WAKEUP_NUM2;
    }
    if (gpio_get_level(GPIO_WAKEUP_NUM3) == GPIO_WAKEUP_LEVEL1) {
        wakeup_gpio = GPIO_WAKEUP_NUM3;
    }
    if (gpio_get_level(GPIO_WAKEUP_NUM4) == GPIO_WAKEUP_LEVEL1) {
        wakeup_gpio = GPIO_WAKEUP_NUM4;
    }
    if (gpio_get_level(GPIO_WAKEUP_NUM5) == GPIO_WAKEUP_LEVEL1) {
        wakeup_gpio = GPIO_WAKEUP_NUM5;
    }

    while (gpio_get_level(GPIO_WAKEUP_NUM) == GPIO_WAKEUP_LEVEL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (gpio_get_level(GPIO_WAKEUP_NUM1) == GPIO_WAKEUP_LEVEL1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (gpio_get_level(GPIO_WAKEUP_NUM2) == GPIO_WAKEUP_LEVEL1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (gpio_get_level(GPIO_WAKEUP_NUM3) == GPIO_WAKEUP_LEVEL1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (gpio_get_level(GPIO_WAKEUP_NUM4) == GPIO_WAKEUP_LEVEL1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (gpio_get_level(GPIO_WAKEUP_NUM5) == GPIO_WAKEUP_LEVEL1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return wakeup_gpio;
}

esp_err_t example_register_gpio_wakeup(void)
{
    /* Initialize GPIO */
    gpio_config_t config = {
            .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = false,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t config1 = {
            .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM1),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = true,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t config2 = {
            .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM2),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = true,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t config3 = {
            .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM3),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = true,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t config4 = {
            .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM4),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = true,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t config5 = {
            .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM5),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = true,
            .pull_up_en = false,
            .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM);
    ESP_RETURN_ON_ERROR(gpio_config(&config1), TAG, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM1);
    ESP_RETURN_ON_ERROR(gpio_config(&config2), TAG, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM2);
    ESP_RETURN_ON_ERROR(gpio_config(&config3), TAG, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM3);
    ESP_RETURN_ON_ERROR(gpio_config(&config4), TAG, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM4);
    ESP_RETURN_ON_ERROR(gpio_config(&config5), TAG, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM5);

    /* Enable wake up from GPIO */
    //调用 gpio_wakeup_enable() 函数可以将任意管脚单独配置为在高电平或低电平触发唤醒
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM, GPIO_WAKEUP_LEVEL == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM1, GPIO_WAKEUP_LEVEL1 == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM2, GPIO_WAKEUP_LEVEL1 == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM3, GPIO_WAKEUP_LEVEL1 == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM4, GPIO_WAKEUP_LEVEL1 == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM5, GPIO_WAKEUP_LEVEL1 == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    //调用 esp_sleep_enable_gpio_wakeup() 函数来启用此唤醒源
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG, "Configure gpio as wakeup source failed");

    /* Make sure the GPIO is inactive and it won't trigger wakeup immediately */
    example_wait_gpio_inactive();
    ESP_LOGI(TAG, "gpio wakeup source is ready");

    return ESP_OK;
}
