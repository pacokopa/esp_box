/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_wifi_init();

char *app_wifi_get_prov_payload(void);

bool app_wifi_is_connected(void);

esp_err_t app_wifi_start(void);

esp_err_t app_wifi_get_wifi_ssid(char *ssid, size_t len);

esp_err_t app_wifi_prov_start(void);

esp_err_t app_wifi_prov_stop(void);

/**
 * @brief Stop Wi-Fi and the BT controller before deep sleep.
 *
 * Safe to call when either radio was never started.
 */
void app_wifi_prepare_deep_sleep(void);

#ifdef __cplusplus
}
#endif
