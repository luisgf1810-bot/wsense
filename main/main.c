#include "main.h"



/* --- Initialize and realign GPTimer --- */

static inline int64_t get_synced_time_us(void)
{
    return esp_timer_get_time() + s_time_offset_us;
}

static bool IRAM_ATTR led_timer_alarm_cb(gptimer_handle_t timer,   const gptimer_alarm_event_data_t *edata,  void *user_ctx) {

 BaseType_t high_task_wakeup = pdFALSE;
    
    uint64_t next_alarm = edata->count_value + period; 
    gptimer_alarm_config_t config = {
        .alarm_count = next_alarm,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(timer, &config);

    uint8_t evt = 1;
    xQueueSendFromISR(s_blink_evt_q, &evt, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static bool IRAM_ATTR imu_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    
    
    // Return true if a high-priority task was awakened to trigger a context switch
    return high_task_awoken == pdTRUE;
}

esp_err_t init_gptimer() {
      gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_gptimer_led));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = led_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer_led, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer_led));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = period,             
        .flags.auto_reload_on_alarm = false 
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer_led, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer_led));

    ESP_LOGI(TAG, "GPTimer started, initial phase = %llu us", (unsigned long long)period);

    return ESP_OK;
}




/* --- LED and blink task--- */
void blinker_led_toggle(void)
{
    if (!s_led) {
        return;
    }
    led_strip_set_pixel(s_led, 0, rcolor, gcolor, 0); 
    led_strip_refresh(s_led);
    vTaskDelay(ondelay);
    led_strip_clear(s_led);
}

static void blink_task(void *arg)
{
    uint64_t tick;
    for (;;) {
        if (xQueueReceive(s_blink_evt_q, &tick, portMAX_DELAY) == pdTRUE) {
            blinker_led_toggle();
        }
    }
}

esp_err_t init_led(void) {

    // Enable the power supply to the LED Strip 
    /* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */    
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = LED_STRIP_NUM_PIXELS,
        .led_model = LED_MODEL_SK6812, // SK6805 shares close timing with SK6812/WS2812
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led));
    led_strip_clear(s_led);

    ESP_LOGI(TAG, "LED initialized"); 

    return ESP_OK;

}




/* --- Init IMU and register callbacks --- */
static void on_sensor_data(bno085_handle_t handle, const bno085_sensor_value_t *value, void *ctx)
{
    if (value->sensor_id == BNO085_SENSOR_LINEAR_ACCELERATION) {
        ESP_LOGI(TAG, "(%" PRIu64 ") Linear Acceleration: x=%.4f, y=%.4f, z=%.4f", esp_timer_get_time(),
               value->data.linear_acceleration.x, value->data.linear_acceleration.y,
               value->data.linear_acceleration.z);
    }
}

esp_err_t imu_init() {

    // Create I2C bus for BNO085
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // Create I2C device for BNO085
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x4A,  // AD0 = GND
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t i2c_dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev));

    // Initialize BNO085
    ESP_ERROR_CHECK(bno085_init(NULL, i2c_dev, GPIO_NUM_7, GPIO_NUM_18, &bno085));  
    bno085_register_sensor_callback(bno085, on_sensor_data, NULL);
    bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 100000);  // 10Hz

    return ESP_OK;

}






/* --- Initialize Wi-Fi & ESP-NOW Managed Sync --- */
static void timesync_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case ESP_EVENT_ESPNOW_TIMESYNC_SYNCED: 
            espnow_timesync_event_t *evt = (espnow_timesync_event_t *)event_data;
            s_time_offset_us = evt->synced_time_us - esp_timer_get_time();
            //period = period + s_time_offset_us;
            
            ESP_LOGI(TAG, "synced from " MACSTR ", reported drift %d ms, offset now %lld us",
                    MAC2STR(evt->src_addr), evt->drift_ms, (long long)s_time_offset_us);
        break;


        case ESP_EVENT_ESPNOW_TIMESYNC_TIMEOUT:
            ESP_LOGW(TAG, "time sync request timed out, retrying");
            espnow_time_responder_request();
            break;

        default:
            break;
    }
}

esp_err_t init_espnow_timesync(void) {

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    espnow_config_t espnow_cfg = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_cfg.qsize = 32;
    ESP_ERROR_CHECK(espnow_init(&espnow_cfg));

    // Responder
    espnow_time_responder_config_t config = {
        .max_drift_ms = 100,
    };

    ESP_ERROR_CHECK(esp_event_handler_register(ESP_EVENT_ESPNOW,  ESP_EVENT_ANY_ID, &timesync_event_handler, NULL));
    ESP_ERROR_CHECK(espnow_time_responder_start(&config));
    ESP_ERROR_CHECK(espnow_time_responder_request());

    ESP_LOGI(TAG, "ESPNOW initialized"); 
    
    return ESP_OK;
}




void app_main()
{
    ESP_LOGI(TAG, "Booting Slave Device...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // led blinking 
    s_blink_evt_q = xQueueCreate(4, sizeof(uint64_t));
    xTaskCreate(blink_task, "blink_task", 4096, NULL, 5, &s_ledtask);


    //ESP_ERROR_CHECK(battery.Init());
    ESP_ERROR_CHECK(init_led());
    ESP_ERROR_CHECK(init_espnow_timesync());
    ESP_ERROR_CHECK(init_gptimer());
    ESP_ERROR_CHECK(flashlog_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(ble_control_init());

    ESP_LOGI(TAG, "Slave ready - waiting for the first ESP-NOW time sync...");

}