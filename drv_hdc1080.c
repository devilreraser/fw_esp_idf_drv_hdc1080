/* *****************************************************************************
 * File:   drv_hdc1080.c
 * Author: XX
 *
 * Created on YYYY MM DD
 * 
 * Description: ...
 * 
 **************************************************************************** */

/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "drv_hdc1080.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_err.h"

#if CONFIG_DRV_I2C_USE
#include "drv_i2c.h"
#endif /* CONFIG_DRV_I2C_USE */

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define TAG "drv_hdc1080"

#define DRV_HDC1080_DEVICE_ADDRESS  0b01000000      /* HDC1080 address is 64 decimal */

/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */
typedef enum
{
    HDC1080_SERIAL_ID_0 = 0xFB,
    HDC1080_SERIAL_ID_1 = 0xFC,
    HDC1080_SERIAL_ID_2 = 0xFD,
    HDC1080_MANUFACTURER_ID = 0xFE,
    HDC1080_DEVICE_ID = 0xFF,
    HDC1080_TEMPERATURE = 0x00,
    HDC1080_HUMIDITY = 0x01,
    HDC1080_CONFIGURATION = 0x02,
    HDC1080_REGISTER_LAST = ((0x100 - HDC1080_SERIAL_ID_0) + HDC1080_CONFIGURATION),
    HDC1080_REGISTER_COUNT = (HDC1080_REGISTER_LAST + 1),
} i2c_e_register_t;

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */
typedef struct 
{
    uint16_t HDC1080_RSVL : 8;       /* Reserved */
    uint16_t HDC1080_HRES : 2;       /* 00: 14 bit, 01: 11 bit, 10: 8 bit */
    uint16_t HDC1080_TRES : 1;       /* 0: 14 bit, 1: 11 bit */
    uint16_t HDC1080_BTST : 1;       /* 1: Battery voltage > 2.8V, 0: Battery voltage < 2.8V */
    uint16_t HDC1080_MODE : 1;       /* 1: Temperature or Humidity. 0: Temperature and Humidity */
    uint16_t HDC1080_HEAT : 1;       /* 1: Heater enabled. 0: Heater disabled */
    uint16_t HDC1080_RSRV : 1;       /* Reserved */
    uint16_t HDC1080_RST  : 1;       /* 1: Software reset. 0: Normal operation */
} i2c_s_register_configuration_t;


/* *****************************************************************************
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */
uint8_t i2c_dev_addr = DRV_HDC1080_DEVICE_ADDRESS;
uint16_t i2c_reg_data[HDC1080_REGISTER_COUNT];  /* 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02 -> registers (each 2 bytes) */
char* i2c_reg_name[HDC1080_REGISTER_COUNT] =   
{   
    /* 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02 -> registers' names */
    "Serial ID[0]", "Serial ID[1]", "Serial ID[2]", "Manufacturer ID", "Device ID", "Temperature", "Humidity", "Configuration",
};
TickType_t ticks_to_wait = pdMS_TO_TICKS(1000);
bool hdc1080_task_active = false;

/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */
static void hdc1080_task(void* arg)
{
    hdc1080_task_active = true;

    while (hdc1080_task_active)
    {
        esp_err_t i2c_err;
        esp_err_t i2c_err_was_timeout = ESP_OK;
        i2c_e_register_t i2c_reg_start = HDC1080_SERIAL_ID_0;   /* HDC1080 start read from Serial ID[0] */

        uint8_t i2c_reg_addr = i2c_reg_start;   

        for (int index = 0; index < HDC1080_REGISTER_COUNT; index++)
        {

            i2c_err = drv_i2c_master_read_from_register(DRV_I2C_INDEX_0, 
                                                        i2c_dev_addr,
                                                        (uint8_t*)&i2c_reg_addr, sizeof(i2c_reg_addr),
                                                        (uint8_t*)&i2c_reg_data[index], sizeof(i2c_reg_data[0]),
                                                        ticks_to_wait);
            if (i2c_err != ESP_OK)
            {
                ESP_LOGE(TAG, "drv_i2c_master_read_from_register failure reg[%d] error:%d (%s) name:%s", i2c_reg_addr, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[index]);
                if (i2c_err == ESP_ERR_TIMEOUT)
                {
                    i2c_err_was_timeout = i2c_err;
                }
            }
            else
            {
                ESP_LOGI(TAG, "drv_i2c_master_read_from_register success reg[%d]=%5d (0x%04X) name:%s", i2c_reg_addr, i2c_reg_data[index], i2c_reg_data[index], i2c_reg_name[index]);
            }

            i2c_reg_addr++;
        }
        if (i2c_err_was_timeout != ESP_ERR_TIMEOUT)
        {
            vTaskDelay(ticks_to_wait);
        } 
    }
    vTaskDelete(NULL);
}


void drv_hdc1080_init(void)
{
    uint8_t i2c_reg_addr_local = HDC1080_CONFIGURATION;
    i2c_s_register_configuration_t s_i2c_reg_cfg = 
    {
        .HDC1080_RSVL = 0,      /* Reserved */
        .HDC1080_HRES = 0,      /* 00: 14 bit, 01: 11 bit, 10: 8 bit */
        .HDC1080_BTST = 0,      /* 1: Battery voltage > 2.8V, 0: Battery voltage < 2.8V */
        .HDC1080_TRES = 0,      /* 0: 14 bit, 1: 11 bit */
        .HDC1080_MODE = 1,      /* 1: Temperature and Humidity are acquired in sequence, Temperature first. 0: Temperature or Humidity is acquired. */
        .HDC1080_HEAT = 0,      /* 1: Heater enabled. 0: Heater disabled */
        .HDC1080_RSRV = 0,      /* Reserved */
        .HDC1080_RST  = 0,      /* 1: Software reset. 0: Normal operation */
    };

    uint16_t i2c_reg_cfg = ((uint16_t*)&s_i2c_reg_cfg)[0];              /* Initialize data for write */

    esp_err_t i2c_err;
    i2c_err = drv_i2c_master_write_to_register( DRV_I2C_INDEX_0, 
                                                i2c_dev_addr,
                                                (uint8_t*)&i2c_reg_addr_local, sizeof(i2c_reg_addr_local),
                                                (uint8_t*)&i2c_reg_cfg, sizeof(i2c_reg_cfg),
                                                ticks_to_wait);
    if (i2c_err != ESP_OK)
    {
        ESP_LOGE(TAG, "drv_i2c_master_write_to_register failure reg[%d] error:%d (%s) name:%s", i2c_reg_addr_local, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[HDC1080_CONFIGURATION + 0x100 - HDC1080_SERIAL_ID_0]);
    }
    else
    {
        ESP_LOGI(TAG, "drv_i2c_master_write_to_register success reg[%d]=%5d (0x%04X) name:%s", i2c_reg_addr_local, i2c_reg_cfg, i2c_reg_cfg, i2c_reg_name[HDC1080_CONFIGURATION + 0x100 - HDC1080_SERIAL_ID_0]);
    }

    xTaskCreate(&hdc1080_task, "hdc1080_task", 2048, NULL, 5, NULL);

}

void drv_hdc1080_deinit(void)
{
    hdc1080_task_active = false;
}   

