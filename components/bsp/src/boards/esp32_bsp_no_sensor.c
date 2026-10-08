/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_check.h"
#include "bsp_board.h"

static const char *TAG = "bsp_sensor";

static bool bsp_get_sleep_mode()
{
    return false;
}

static bottom_id_t bsp_get_bottom_id()
{
    return BOTTOM_ID_UNKNOW;
}

static bool bsp_sensor_get_radar_status()
{
    return false;
}

static void bsp_sensor_set_radar_enable(bool enable)
{
    return;
}

static esp_err_t bsp_sensor_get_humiture(float *temperature, float *humidity)
{
    return ESP_FAIL;
}

void bsp_register_deep_sleep_prepare_cb(bsp_deep_sleep_prepare_cb_t cb)
{
    (void)cb;
}

esp_err_t bsp_board_set_battery_save_mode(bool enable)
{
    (void)enable;
    /* No sensor bottom / no PM locks on this board variant. Nothing to do. */
    return ESP_OK;
}

esp_err_t bsp_sensor_init(bsp_bottom_property_t *handle)
{
    ESP_LOGW(TAG, "This example don't support Sensor!!");

    handle->get_sleep_mode = bsp_get_sleep_mode;
    handle->get_bottom_id = bsp_get_bottom_id;
    handle->get_radar_status = bsp_sensor_get_radar_status;
    handle->set_radar_enable = bsp_sensor_set_radar_enable;
    handle->get_humiture = bsp_sensor_get_humiture;

    return ESP_ERR_NOT_SUPPORTED;
}
