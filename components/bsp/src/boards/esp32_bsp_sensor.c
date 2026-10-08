/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_check.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include "bsp_board.h"
#include "aht20.h"
#include "at581x.h"

#define BSP_I2C_EXPAND_NUM              ((1 == BSP_I2C_NUM) ? (0):(1))
#define BSP_I2C_EXPAND_CLK_SPEED_HZ     CONFIG_BSP_I2C_CLK_SPEED_HZ

#define RADAE_POWER_DELAY               (60 * 2) // 2min
#define RADAE_FUNC_STOP                 (RADAE_POWER_DELAY + 1)

static bool sys_sleep_entered = false;
static bottom_id_t sys_bottom_id;

static float sys_temp_result;
static float sys_RH_result;
static uint16_t power_off_delay;

static aht20_dev_handle_t aht20 = NULL;
static esp_pm_lock_handle_t g_pm_apb_lock = NULL;
static esp_pm_lock_handle_t g_pm_light_lock = NULL;
static esp_pm_lock_handle_t g_pm_cpu_lock = NULL;
static bool s_battery_save_mode = false;
static bool s_pm_locks_held = false;

static const char *TAG = "bsp_sensor";

static bsp_deep_sleep_prepare_cb_t s_deep_sleep_prepare_cb = NULL;

static esp_err_t bsp_pm_init();
static esp_err_t bsp_pm_exit_sleep(void);
static esp_err_t bsp_pm_enter_sleep(void);
static void bsp_enter_deep_sleep(void);

static bool bsp_i2c_device_probe(i2c_port_t i2c_num, uint8_t addr);

void bsp_register_deep_sleep_prepare_cb(bsp_deep_sleep_prepare_cb_t cb)
{
    s_deep_sleep_prepare_cb = cb;
}

static bool bsp_get_sleep_mode()
{
    return sys_sleep_entered;
}

static bottom_id_t bsp_get_bottom_id()
{
    return sys_bottom_id;
}

static bool bsp_sensor_get_radar_status()
{
    if (BOTTOM_ID_SENSOR == sys_bottom_id) {
        return ((power_off_delay > RADAE_POWER_DELAY / 2) ? (true) : (false));
    } else {
        return false;
    }
}

static void bsp_sensor_set_radar_onoff(bool enable)
{
    if (enable) {
        power_off_delay = RADAE_POWER_DELAY;
    } else {
        power_off_delay = RADAE_FUNC_STOP;
    }
}

static esp_err_t bsp_sensor_get_humiture(float *temperature, float *humidity)
{
    if (BOTTOM_ID_SENSOR == sys_bottom_id) {
        *temperature = sys_temp_result;
        *humidity = sys_RH_result;
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

static void low_power_monitor_task(void *arg)
{
    static uint8_t gpio_level_prev = 1;
    uint32_t temp_raw, RH_raw;
    uint8_t gpio_level;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << BSP_RADAR_OUT_IO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    vTaskDelay(pdMS_TO_TICKS(1500));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (sys_bottom_id != BOTTOM_ID_UNKNOW) {
            sys_bottom_id = (bsp_i2c_device_probe(BSP_I2C_EXPAND_NUM, AT581X_ADDRRES_0)) ? BOTTOM_ID_SENSOR : BOTTOM_ID_LOST;
        }

        if (BOTTOM_ID_SENSOR == sys_bottom_id) {
            gpio_level = gpio_get_level(BSP_RADAR_OUT_IO);
            if ((RADAE_FUNC_STOP != power_off_delay) && power_off_delay) {
                power_off_delay--;
            }
            aht20_read_temperature_humidity(aht20, &temp_raw, &sys_temp_result, &RH_raw, &sys_RH_result);
        } else {
            gpio_level = 1;
        }

        if (gpio_level_prev ^ gpio_level) {
            gpio_level_prev = gpio_level;
            if (gpio_level && (RADAE_FUNC_STOP != power_off_delay)) {
                power_off_delay = RADAE_POWER_DELAY;
                ESP_LOGD(TAG, "Radar: %s", "active");
            }
        }

        if ((gpio_level) && (true == sys_sleep_entered)) {
            bsp_pm_exit_sleep();

            ESP_LOGD(TAG, "power on");
            bsp_display_exit_sleep();

            lvgl_port_resume();
            iot_button_resume();
            bsp_codec_dev_resume();
            sys_sleep_entered = false;
        } else if ((1 == power_off_delay) && (BOTTOM_ID_SENSOR == sys_bottom_id)) {
            if (gpio_level) {
                /* Output is still high, so deep sleep would wake immediately. */
                power_off_delay = RADAE_POWER_DELAY;
            } else {
                bsp_enter_deep_sleep();
            }
        }
    }
}

static esp_err_t bsp_init_temp_humudity()
{
    esp_err_t ret = ESP_OK;

    aht20_i2c_config_t i2c_conf = {
        .i2c_port = BSP_I2C_EXPAND_NUM,
        .i2c_addr = AHT20_ADDRRES_0,
    };

    ret |= aht20_new_sensor(&i2c_conf, &aht20);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Temp & humidity init ok");
    }
    return ret;
}

static esp_err_t bsp_init_radar()
{
    esp_err_t ret = ESP_OK;
    at581x_dev_handle_t at581x = NULL;

    at581x_default_cfg_t def_cfg = ATH581X_INITIALIZATION_CONFIG();
    /* Gain index steps down about 3 dB each. A larger delta raises the
     * detection threshold and shortens range, so small movements do not
     * keep restarting the two-minute sleep timer. */
    def_cfg.gain_cfg = AT581X_STAGE_GAIN_6;
    def_cfg.delta_cfg = 500;

    at581x_i2c_config_t i2c_conf = {
        .i2c_port = BSP_I2C_EXPAND_NUM,
        .i2c_addr = AT581X_ADDRRES_0,
        .def_conf = &def_cfg,
    };

    ret |= at581x_new_sensor(&i2c_conf, &at581x);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Radar init ok");
    }
    return ret;
}

static void bsp_enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Radar idle, entering deep sleep");
    sys_sleep_entered = true;

    bsp_display_enter_sleep();
    lvgl_port_stop();
    iot_button_stop();
    bsp_codec_dev_stop();
    bsp_pm_enter_sleep();

    if (s_deep_sleep_prepare_cb) {
        s_deep_sleep_prepare_cb();
    }

    /* Backlight is LEDC on a digital GPIO. Hold it low so deep sleep does
     * not let the pad float the LED back on. */
    gpio_set_direction(BSP_LCD_BACKLIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(BSP_LCD_BACKLIGHT, 0);
    gpio_hold_en(BSP_LCD_BACKLIGHT);
    gpio_deep_sleep_hold_en();

    /* GPIO21 is RTC-capable. The radar output is active-high. */
    ESP_ERROR_CHECK(rtc_gpio_init(BSP_RADAR_OUT_IO));
    rtc_gpio_set_direction(BSP_RADAR_OUT_IO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(BSP_RADAR_OUT_IO);
    rtc_gpio_pulldown_dis(BSP_RADAR_OUT_IO);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(1ULL << BSP_RADAR_OUT_IO, ESP_EXT1_WAKEUP_ANY_HIGH));

    esp_deep_sleep_start();
}

static esp_err_t bsp_pm_exit_sleep(void)
{
    esp_err_t ret = ESP_OK;

    /* In battery-save mode we intentionally stay unlocked so DFS +
     * tickless light-sleep can engage. The radar wake path calls this,
     * so it must be a no-op there. */
    if (s_battery_save_mode) {
        return ESP_OK;
    }
    if (s_pm_locks_held) {
        return ESP_OK;
    }
    ret |= esp_pm_lock_acquire(g_pm_apb_lock);
    ret |= esp_pm_lock_acquire(g_pm_light_lock);
    ret |= esp_pm_lock_acquire(g_pm_cpu_lock);
    if (ret == ESP_OK) {
        s_pm_locks_held = true;
    }
    return ret;
}

static esp_err_t bsp_pm_enter_sleep(void)
{
    esp_err_t ret = ESP_OK;

    if (!s_pm_locks_held) {
        return ESP_OK;
    }
    ret |= esp_pm_lock_release(g_pm_apb_lock);
    ret |= esp_pm_lock_release(g_pm_light_lock);
    ret |= esp_pm_lock_release(g_pm_cpu_lock);
    if (ret == ESP_OK) {
        s_pm_locks_held = false;
    }
    return ret;
}

esp_err_t bsp_board_set_battery_save_mode(bool enable)
{
    esp_err_t ret = ESP_OK;

    if (enable) {
        s_battery_save_mode = true;
        ESP_LOGI(TAG, "Battery-save on: release PM locks for DFS + light-sleep");
        ret = bsp_pm_enter_sleep();
    } else {
        s_battery_save_mode = false;
        ESP_LOGI(TAG, "Battery-save off: hold PM locks for max perf");
        ret = bsp_pm_exit_sleep();
    }
    return ret;
}

static esp_err_t bsp_pm_init()
{
    esp_err_t ret = ESP_OK;
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

    if (g_pm_apb_lock == NULL) {
        ESP_RETURN_ON_ERROR(esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "l_apb", &g_pm_apb_lock),
                            TAG, "create l_apb pm lock failed");
    }

    if (g_pm_light_lock == NULL) {
        ESP_RETURN_ON_ERROR(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "l_ls", &g_pm_light_lock),
                            TAG, "create l_ls pm lock failed");
    }
    if (g_pm_cpu_lock == NULL) {
        ESP_RETURN_ON_ERROR(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "l_cpu", &g_pm_cpu_lock),
                            TAG, "create l_cpu pm lock failed");
    }
    bsp_pm_exit_sleep();

    ret = xTaskCreatePinnedToCore(&low_power_monitor_task, "Lowpower Task", 8 * 1024, NULL, 5, NULL, 1);
    ESP_RETURN_ON_ERROR(pdPASS != ret, TAG,  "create Lowpower task failed");
    return ret;
}

static bool bsp_i2c_device_probe(i2c_port_t i2c_num, uint8_t addr)
{
    bool probe_result = false;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(i2c_num, cmd, 2000) == ESP_OK) {
        probe_result = true;
    }
    i2c_cmd_link_delete(cmd);
    return probe_result;
}

static esp_err_t bsp_i2c_expand_init(void)
{
    static bool i2c_initialized = false;

    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_expand_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_EXPAND_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = BSP_I2C_EXPAND_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = BSP_I2C_EXPAND_CLK_SPEED_HZ
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(BSP_I2C_EXPAND_NUM, &i2c_expand_conf),
                        TAG, "create expand i2c failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(BSP_I2C_EXPAND_NUM, i2c_expand_conf.mode, 0, 0, 0),
                        TAG, "install expand i2c failed");

    i2c_initialized = true;

    ESP_LOGI(TAG, "I2C num: %d, [%d, %d], speed:%d",
             BSP_I2C_EXPAND_NUM, BSP_I2C_EXPAND_SCL, BSP_I2C_EXPAND_SDA, BSP_I2C_EXPAND_CLK_SPEED_HZ);

    return ESP_OK;
}

static esp_err_t bsp_i2c_expand_deinit(void)
{
    ESP_RETURN_ON_ERROR(i2c_driver_delete(BSP_I2C_EXPAND_NUM),
                        TAG, "uninstall expand i2c failed");
    return ESP_OK;
}

esp_err_t bsp_sensor_init(bsp_bottom_property_t *handle)
{
    esp_err_t ret = ESP_OK;

    ret |= bsp_pm_init();
    ret |= bsp_i2c_expand_init();

    if (bsp_i2c_device_probe(BSP_I2C_EXPAND_NUM, AT581X_ADDRRES_0)) {
        ESP_LOGW(TAG, "Sensor bottom connected");
        ret |= bsp_init_radar();
        ret |= bsp_init_temp_humudity();
        sys_bottom_id = BOTTOM_ID_SENSOR;
    } else {
        ESP_LOGW(TAG, "Sensor bottom lost");
        sys_bottom_id = BOTTOM_ID_UNKNOW;
        bsp_i2c_expand_deinit();
    }

    handle->get_sleep_mode = bsp_get_sleep_mode;
    handle->get_bottom_id = bsp_get_bottom_id;
    handle->get_radar_status = bsp_sensor_get_radar_status;
    handle->set_radar_enable = bsp_sensor_set_radar_onoff;
    handle->get_humiture = bsp_sensor_get_humiture;

    return ret;
}
