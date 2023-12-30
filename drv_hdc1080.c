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

//#if CONFIG_DRV_I2C_USE
#include "drv_i2c.h"
//#endif /* CONFIG_DRV_I2C_USE */

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define TAG "drv_hdc1080"

#define DRV_HDC1080_DEVICE_ADDRESS  0b01000000      /* HDC1080 address is 64 decimal */

/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */
#if CONFIG_DRV_HDC1080_SELECT_I2C_INDEX_0
#define DRV_HDC1080_PORT  DRV_I2C_INDEX_0
#elif  CONFIG_DRV_HDC1080_SELECT_I2C_INDEX_1
#define DRV_HDC1080_PORT  DRV_I2C_INDEX_1
#else
#define DRV_HDC1080_PORT  -1
#endif

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

typedef enum
{
    HDC1080_INDEX_SERIAL_ID_0,
    HDC1080_INDEX_SERIAL_ID_1,
    HDC1080_INDEX_SERIAL_ID_2,
    HDC1080_INDEX_MANUFACTURER_ID,
    HDC1080_INDEX_DEVICE_ID,
    HDC1080_INDEX_ID_REG_COUNT,
    HDC1080_INDEX_TEMPERATURE = HDC1080_INDEX_ID_REG_COUNT,
    HDC1080_INDEX_HUMIDITY,
    HDC1080_INDEX_CONFIGURATION,
    HDC1080_INDEX_COUNT,
} i2c_e_register_index_t;




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

typedef union 
{
    uint16_t u16_register;
    i2c_s_register_configuration_t s_configuration;
} i2c_u_register_configuration_t;

#define HDC1080_CONFIGURATION_CHECK_MASK 0x3700 /* exclude reserved, BTST, RST */

/* *****************************************************************************
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */
uint8_t i2c_dev_addr = DRV_HDC1080_DEVICE_ADDRESS;

uint16_t i2c_reg_addr[HDC1080_REGISTER_COUNT] = 
{
    /* 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02 -> registers' addresses */
    HDC1080_SERIAL_ID_0, 
    HDC1080_SERIAL_ID_1, 
    HDC1080_SERIAL_ID_2, 
    HDC1080_MANUFACTURER_ID,
    HDC1080_DEVICE_ID, 
    HDC1080_TEMPERATURE, 
    HDC1080_HUMIDITY, 
    HDC1080_CONFIGURATION,
};

uint16_t i2c_reg_data[HDC1080_REGISTER_COUNT];  /* 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02 -> registers (each 2 bytes) */
uint16_t i2c_reg_cfg = 0xFFFF;
char* i2c_reg_name[HDC1080_REGISTER_COUNT] =   
{   
    /* 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02 -> registers' names */
    "Serial ID[0]", "Serial ID[1]", "Serial ID[2]", "Manufacturer ID", "Device ID", "Temperature", "Humidity", "Configuration",
};
TickType_t rdwr_timeout = pdMS_TO_TICKS(10);
TickType_t meas_timeout = pdMS_TO_TICKS(20);
TickType_t loop_timeout = pdMS_TO_TICKS(1000);
TickType_t measure_wait = pdMS_TO_TICKS(20);
bool hdc1080_task_active = false;

drv_i2c_e_index_t i2c_index = DRV_HDC1080_PORT;



/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */

#if TO_DO_UNCOMMENT_IF_NEEDED

static void hdc1080_read_registers(i2c_e_register_index_t reg_start_index, size_t reg_count)
{
    esp_err_t i2c_err;

    uint8_t i2c_reg_addr_local = i2c_reg_addr[(int)reg_start_index];   

    uint16_t i2c_reg_cfg_swap[reg_count];

    i2c_err = drv_i2c_master_read_from_register(i2c_index, 
                                                i2c_dev_addr,
                                                (uint8_t*)&i2c_reg_addr_local, sizeof(i2c_reg_addr_local),
                                                (uint8_t*)&i2c_reg_cfg_swap[0], sizeof(i2c_reg_data[0]) * reg_count,
                                                rdwr_timeout);
    for(int index = 0; index < reg_count; index++)
    {
        i2c_reg_data[reg_start_index + index] = __bswap16(i2c_reg_cfg_swap[index]);
    }
    if (i2c_err != ESP_OK)
    {                  hdc1080_read_registers           
        ESP_LOGE(TAG, "hdc1080_read_registers            failure reg[0x%02X] error:%d (%s) name:%s", i2c_reg_addr_local, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[reg_start_index]);
    }
    else
    {
        for (int index = reg_start_index; index < (reg_start_index + reg_count); index++)
        {
            ESP_LOGI(TAG, "hdc1080_read_registers            success reg[0x%02x]=%5d (0x%04X) name:%s", i2c_reg_addr_local, i2c_reg_data[index], i2c_reg_data[index], i2c_reg_name[index]);
            i2c_reg_addr_local++;
        }

    }
}

static void hdc1080_write_registers(i2c_e_register_index_t reg_start_index, size_t reg_count)
{
    esp_err_t i2c_err;

    uint8_t i2c_reg_addr_local = i2c_reg_addr[(int)reg_start_index];   

    uint16_t i2c_reg_cfg_swap[reg_count];
    for(int index = 0; index < reg_count; index++)
    {
        i2c_reg_cfg_swap[index] = __bswap16(i2c_reg_data[reg_start_index + index]);
    }

    i2c_err = drv_i2c_master_write_to_register( i2c_index, 
                                                i2c_dev_addr,
                                                (uint8_t*)&i2c_reg_addr_local, sizeof(i2c_reg_addr_local),
                                                (uint8_t*)&i2c_reg_cfg_swap[0], sizeof(i2c_reg_cfg_swap[0]) * reg_count,
                                                rdwr_timeout);
    if (i2c_err != ESP_OK)
    {
        ESP_LOGE(TAG, "drv_i2c_master_write_to_register failure reg[0x%02x] error:%d (%s) name:%s", i2c_reg_addr_local, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[reg_start_index]);
    }
    else
    {
        // maybe not needed trigger implement saparately because no stop condition needed
        // if (reg_count == 0)
        // {
        //     ESP_LOGI(TAG, "drv_i2c_master_write_to_register success reg[0x%02x] point only name:%s", i2c_reg_addr_local, i2c_reg_name[reg_start_index]);
        // }
        for (int index = reg_start_index; index < (reg_start_index + reg_count); index++)
        {
            ESP_LOGI(TAG, "drv_i2c_master_write_to_register success reg[0x%02x]=%5d (0x%04X) name:%s", i2c_reg_addr_local, i2c_reg_data[index], i2c_reg_data[index], i2c_reg_name[index]);
            i2c_reg_addr_local++;
        }

    }
}

#endif /* TO_DO_UNCOMMENT_IF_NEEDED */

static void hdc1080_trigger_measurement(void)
{
    esp_err_t i2c_err;

    uint8_t i2c_reg_addr_local = HDC1080_TEMPERATURE;   

    i2c_err = drv_i2c_master_point_to_register( i2c_index, 
                                                i2c_dev_addr,
                                                (uint8_t*)&i2c_reg_addr_local, sizeof(i2c_reg_addr_local),
                                                rdwr_timeout);
    if (i2c_err != ESP_OK)
    {
        ESP_LOGE(TAG, "drv_i2c_master_point_to_register failure reg[0x%02x] error:%d (%s) name:%s", i2c_reg_addr_local, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[HDC1080_INDEX_TEMPERATURE]);
    }
    else
    {
        ESP_LOGI(TAG, "drv_i2c_master_point_to_register success reg[0x%02x] point only name:%s", i2c_reg_addr_local, i2c_reg_name[HDC1080_INDEX_TEMPERATURE]);
    }

}
static void hdc1080_read_measurement(size_t reg_count)
{
    esp_err_t i2c_err;

    uint16_t i2c_reg_cfg_swap[reg_count];
    uint8_t i2c_reg_addr_local = HDC1080_TEMPERATURE;  

    i2c_err = drv_i2c_master_read_pointed_register(i2c_index, 
                                                i2c_dev_addr,
                                                (uint8_t*)&i2c_reg_cfg_swap[0], sizeof(i2c_reg_data[0]) * reg_count,
                                                meas_timeout);
    for(int index = 0; index < reg_count; index++)
    {
        i2c_reg_data[HDC1080_INDEX_TEMPERATURE + index] = __bswap16(i2c_reg_cfg_swap[index]);
    }
    if (i2c_err != ESP_OK)
    {
        ESP_LOGE(TAG, "hdc1080_read_measurement          failure reg[0x%02X] error:%d (%s) name:%s", i2c_reg_addr_local, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[HDC1080_INDEX_TEMPERATURE]);
    }
    else
    {
        for (int index = HDC1080_INDEX_TEMPERATURE; index < (HDC1080_INDEX_TEMPERATURE + reg_count); index++)
        {                           
            ESP_LOGI(TAG, "hdc1080_read_measurement          success reg[0x%02x]=%5d (0x%04X) name:%s", i2c_reg_addr_local, i2c_reg_data[index], i2c_reg_data[index], i2c_reg_name[index]);
            i2c_reg_addr_local++;
        }

    }
}


static void hdc1080_task(void* arg)
{
    hdc1080_task_active = true;

    while (hdc1080_task_active)
    {
        hdc1080_trigger_measurement();
        vTaskDelay(measure_wait);
        hdc1080_read_measurement(2);
        vTaskDelay(loop_timeout);
    }
    vTaskDelete(NULL);
}

static void hdc1080_read_id_registers(void)
{
    esp_err_t i2c_err;
    i2c_e_register_t i2c_reg_start = HDC1080_SERIAL_ID_0;   /* HDC1080 start read from Serial ID[0] */

    uint8_t i2c_reg_addr = i2c_reg_start;   

    for (int index = HDC1080_INDEX_SERIAL_ID_0; index < HDC1080_INDEX_ID_REG_COUNT; index++)
    {

        uint16_t i2c_reg_cfg_swap;

        i2c_err = drv_i2c_master_read_from_register(i2c_index, 
                                                    i2c_dev_addr,
                                                    (uint8_t*)&i2c_reg_addr, sizeof(i2c_reg_addr),
                                                    (uint8_t*)&i2c_reg_cfg_swap, sizeof(i2c_reg_cfg_swap),
                                                    rdwr_timeout);

        i2c_reg_data[index] = __bswap16(i2c_reg_cfg_swap);

        if (i2c_err != ESP_OK)
        {                        
            ESP_LOGE(TAG, "hdc1080_read_id_registers         failure reg[0x%02X] error:%d (%s) name:%s", i2c_reg_addr, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[index]);
        }
        else
        {                     
            ESP_LOGI(TAG, "hdc1080_read_id_registers         success reg[0x%02x]=%5d (0x%04X) name:%s", i2c_reg_addr, i2c_reg_data[index], i2c_reg_data[index], i2c_reg_name[index]);
        }

        i2c_reg_addr++;
    }

}



static void hdc1080_read_configuration_register(void)
{
    esp_err_t i2c_err;
    i2c_e_register_t i2c_reg_start = HDC1080_CONFIGURATION;   /* HDC1080 start read from Configuration */

    uint8_t i2c_reg_addr = i2c_reg_start;   

    for (int index = HDC1080_INDEX_CONFIGURATION; index < (HDC1080_INDEX_CONFIGURATION + 1); index++)
    {
        uint16_t i2c_reg_cfg_swap;

        i2c_err = drv_i2c_master_read_from_register(i2c_index, 
                                                    i2c_dev_addr,
                                                    (uint8_t*)&i2c_reg_addr, sizeof(i2c_reg_addr),
                                                    (uint8_t*)&i2c_reg_cfg_swap, sizeof(i2c_reg_cfg_swap),
                                                    rdwr_timeout);

        i2c_reg_data[index] = __bswap16(i2c_reg_cfg_swap);

        if (i2c_err != ESP_OK)
        {
            ESP_LOGE(TAG, "hdc1080_read_configuration_register  fail reg[0x%02X] error:%d (%s) name:%s", i2c_reg_addr, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[index]);
        }
        else
        {
            ESP_LOGI(TAG, "hdc1080_read_configuration_register  pass reg[0x%02x]=%5d (0x%04X) name:%s", i2c_reg_addr, i2c_reg_data[index], i2c_reg_data[index], i2c_reg_name[index]);
        }

        i2c_reg_addr++;
    }

}

static void hdc1080_init_configuration_register(void)
{
    uint8_t i2c_reg_addr_local = HDC1080_CONFIGURATION;

    esp_err_t i2c_err;

    uint16_t i2c_reg_cfg_swap = __bswap16(i2c_reg_cfg);


    i2c_err = drv_i2c_master_write_to_register( i2c_index, 
                                                i2c_dev_addr,
                                                (uint8_t*)&i2c_reg_addr_local, sizeof(i2c_reg_addr_local),
                                                (uint8_t*)&i2c_reg_cfg_swap, sizeof(i2c_reg_cfg_swap),
                                                rdwr_timeout);
    if (i2c_err != ESP_OK)
    {
        ESP_LOGE(TAG, "drv_i2c_master_write_to_register  failure reg[0x%02x] error:%d (%s) name:%s", i2c_reg_addr_local, i2c_err, esp_err_to_name(i2c_err), i2c_reg_name[HDC1080_INDEX_CONFIGURATION]);
    }
    else
    {
        ESP_LOGI(TAG, "drv_i2c_master_write_to_register  success reg[0x%02x]=%5d (0x%04X) name:%s", i2c_reg_addr_local, i2c_reg_cfg, i2c_reg_cfg, i2c_reg_name[HDC1080_INDEX_CONFIGURATION]);
    }

}

void drv_hdc1080_init(void)
{
    i2c_u_register_configuration_t u_i2c_reg_cfg = 
    {
        .s_configuration.HDC1080_RSVL = 0,      /* Reserved */
        .s_configuration.HDC1080_HRES = 0,      /* 00: 14 bit, 01: 11 bit, 10: 8 bit */
        .s_configuration.HDC1080_BTST = 0,      /* 1: Battery voltage > 2.8V, 0: Battery voltage < 2.8V */
        .s_configuration.HDC1080_TRES = 0,      /* 0: 14 bit, 1: 11 bit */
        .s_configuration.HDC1080_MODE = 1,      /* 1: Temperature and Humidity are acquired in sequence, Temperature first. 0: Temperature or Humidity is acquired. */
        .s_configuration.HDC1080_HEAT = 0,      /* 1: Heater enabled. 0: Heater disabled */
        .s_configuration.HDC1080_RSRV = 0,      /* Reserved */
        .s_configuration.HDC1080_RST  = 0,      /* 1: Software reset. 0: Normal operation */
    };

    hdc1080_read_id_registers();

    hdc1080_read_configuration_register();

    i2c_reg_cfg = i2c_reg_data[HDC1080_INDEX_CONFIGURATION] & (~HDC1080_CONFIGURATION_CHECK_MASK); /* Initialize data for write */

    i2c_reg_cfg |= u_i2c_reg_cfg.u16_register & HDC1080_CONFIGURATION_CHECK_MASK; /* Initialize data for write */

    hdc1080_init_configuration_register();

    hdc1080_read_configuration_register();

    if ((i2c_reg_data[HDC1080_INDEX_CONFIGURATION] & HDC1080_CONFIGURATION_CHECK_MASK) != (i2c_reg_cfg & HDC1080_CONFIGURATION_CHECK_MASK))
    {
        ESP_LOGE(TAG, "drv_hdc1080_init configuration failure");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "drv_hdc1080_init configuration success");
    }

    xTaskCreate(&hdc1080_task, "hdc1080_task", 2048, NULL, 5, NULL);

}

void drv_hdc1080_deinit(void)
{
    hdc1080_task_active = false;
}   

