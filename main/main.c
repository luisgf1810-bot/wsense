#include "main.h"






static inline int64_t get_synced_time_us(void)
{
    return esp_timer_get_time() + s_time_offset_us;
}


/* --- LED Toggle --- */
void blinker_led_toggle(void)
{
    if (!s_led) {
        return;
    }

    led_state = !led_state;
    if (led_state) {
        led_strip_set_pixel(s_led, 0, 0, 7, 0); /* dim green */
        led_strip_refresh(s_led);
    } else {
        led_strip_clear(s_led);
    }
}

/* --- GPTimer ISR Callback --- */
static bool IRAM_ATTR gptimer_on_alarm_cb(gptimer_handle_t timer,   const gptimer_alarm_event_data_t *edata,  void *user_ctx) {
    
    BaseType_t hp_task_woken = pdFALSE;
    uint64_t tick = edata->alarm_value;
    xQueueSendFromISR(s_blink_evt_q, &tick, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

/* Blink task */
static void blink_task(void *arg)
{
    uint64_t tick;
    for (;;) {
        if (xQueueReceive(s_blink_evt_q, &tick, portMAX_DELAY) == pdTRUE) {
            blinker_led_toggle();
            ESP_LOGI(LED_TAG, "blink @ t = %lld us (reference clock)", (long long)esp_timer_get_time());
            
        }
    }
}



// IMU

static void on_sensor_data(bno085_handle_t handle, const bno085_sensor_value_t *value, void *ctx)
{
    if (value->sensor_id == BNO085_SENSOR_LINEAR_ACCELERATION) {
        ESP_LOGI(MAIN_TAG, "(%" PRIu64 ") Linear Acceleration: x=%.4f, y=%.4f, z=%.4f", esp_timer_get_time(),
               value->data.linear_acceleration.x, value->data.linear_acceleration.y,
               value->data.linear_acceleration.z);
    }
}

static bool imu_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    
    
    // Return true if a high-priority task was awakened to trigger a context switch
    return high_task_awoken == pdTRUE;
}

void imu_init() {

  
    // IMU chip
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

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x4A,  // AD0 = GND
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t i2c_dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev));

    ESP_ERROR_CHECK(bno085_init(NULL, i2c_dev, GPIO_NUM_7, GPIO_NUM_18, &bno085));  // NULL = default config
    bno085_register_sensor_callback(bno085, on_sensor_data, NULL);
    bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 100000);  // 10Hz


    // Sampling gtimer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, 
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // Register the alarm callback function
    gptimer_event_callbacks_t cbs = {
        .on_alarm = imu_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // Set alarm period (1,000,000 ticks = 1 Hz sampling rate)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    // Enable and start the hardware timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));


}



/* --- Initialize Hardware GPTimer --- */
static void start_gptimer(uint64_t phase_reference_us) {
      gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_gptimer_led));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer_led, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer_led));

    /* Pre-load the raw counter with our current position inside the
     * 3-second cycle. The alarm is fixed at BLINKER_PERIOD_US, so the
     * very first alarm fires after exactly (BLINKER_PERIOD_US - phase)
     * ticks - i.e. precisely on the next aligned boundary. After that,
     * auto-reload-to-0 keeps every subsequent alarm exactly
     * BLINKER_PERIOD_US ticks apart. */
    uint64_t phase = phase_reference_us % BLINK_PERIOD_US;
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_gptimer_led, phase));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = BLINK_PERIOD_US,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer_led, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer_led));

    ESP_LOGI(MAIN_TAG, "GPTimer started, initial phase = %llu us into the 3s cycle", (unsigned long long)phase);
}

/* --- Discipline GPTimer --- */
esp_err_t realign_gptimer(uint64_t phase_reference_us)
{

    uint64_t target_phase = phase_reference_us % BLINK_PERIOD_US;

    uint64_t current_raw = 0;
    ESP_ERROR_CHECK(gptimer_get_raw_count(s_gptimer_led, &current_raw));
    uint64_t current_phase = current_raw % BLINK_PERIOD_US;

    int64_t error_us = (int64_t)target_phase - (int64_t)current_phase;
    /* Handle wrap-around: pick the shorter path around the 3s circle. */
    if (error_us > (int64_t)(BLINK_PERIOD_US / 2)) {
        error_us -= (int64_t)BLINK_PERIOD_US;
    } else if (error_us < -(int64_t)(BLINK_PERIOD_US / 2)) {
        error_us += (int64_t)BLINK_PERIOD_US;
    }

    if (error_us > -(int64_t)BLINKER_MIN_CORRECTION_US && error_us <  (int64_t)BLINKER_MIN_CORRECTION_US) {
        /* Drift is negligible - don't bother touching the register. */
        return ESP_OK;
    }

    /* Avoid stepping the counter right on top of the alarm point: that
     * is the one moment a raw-count write could cause a missed or
     * double alarm. Defer to the next sync round instead - a few
     * hundred ms of extra drift is invisible on a 3s LED blink. */
    uint64_t distance_to_alarm = (current_phase > target_phase) ? (BLINK_PERIOD_US - current_phase) : (target_phase - current_phase);
    if (distance_to_alarm < 5000ULL /* 5 ms guard band */) {
        ESP_LOGW(MAIN_TAG, "skipping realign, too close to the alarm edge");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(gptimer_set_raw_count(s_gptimer_led, target_phase));
    ESP_LOGI(MAIN_TAG, "disciplined GPTimer: phase error %lld us corrected", (long long)error_us);

    return ESP_OK;
}

/* --- ESP-NOW Time-Sync Event Handler (Slave Side) --- */
static void timesync_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case ESP_EVENT_ESPNOW_TIMESYNC_SYNCED: {
            espnow_timesync_event_t *evt = (espnow_timesync_event_t *)event_data;
            s_time_offset_us = evt->synced_time_us - esp_timer_get_time();

            ESP_LOGI(MAIN_TAG, "synced from " MACSTR ", reported drift %d ms, offset now %lld us",
                    MAC2STR(evt->src_addr), evt->drift_ms, (long long)s_time_offset_us);

            if (!s_timer_started) {
                /* First sync ever: bring the GPTimer up, phase-aligned to
                * the master right from the very first tick. */
                start_gptimer((uint64_t)get_synced_time_us());
                s_timer_started = true;
            } else {
                /* Steady state: discipline the free-running hardware
                * counter to cancel whatever phase error has built up
                * since the last correction. */
                realign_gptimer((uint64_t)get_synced_time_us());
            }
            break;
        }

        case ESP_EVENT_ESPNOW_TIMESYNC_TIMEOUT:
            ESP_LOGW(MAIN_TAG, "time sync request timed out, retrying");
            espnow_time_responder_request();
            break;

        default:
            break;
    }
}

/* --- Initialize Wi-Fi & ESP-NOW Managed Sync --- */
static void init_espnow_timesync(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Wi-Fi Stack Initialization
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // ESP-NOW Managed Core Initialization
    espnow_config_t espnow_cfg = ESPNOW_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(espnow_init(&espnow_cfg));

    // Responder
    espnow_time_responder_config_t config = {
        .max_drift_ms = 100,
    };

    ESP_LOGI(TAG, "Configuring ESP-NOW TimeSync Receiver (SLAVE)...");
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_EVENT_ESPNOW,  ESP_EVENT_ANY_ID, &timesync_event_handler, NULL));
    ESP_ERROR_CHECK(espnow_time_responder_start(&config));

}

/* --- Setup GPIOs --- */
void SetupPins() {
    // Enable the power supply to the LED Strip 
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

}

/* --- Initialize Addressable LED --- */
static void init_led(void) {

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

    ESP_LOGI(LED_TAG, "LED initialized"); 

}


void app_main()
{
    ESP_LOGI(MAIN_TAG, "Booting Slave Device...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_blink_evt_q = xQueueCreate(4, sizeof(uint64_t));
    
    SetupPins();
    init_led();
    init_espnow_timesync();

    xTaskCreate(blink_task, "blink_task", 4096, NULL, 5, NULL);

    /* Don't wait for the master's next periodic broadcast - ask for a
     * sync immediately so the GPTimer starts as soon as possible. */
    ESP_ERROR_CHECK(espnow_time_responder_request());

    ESP_LOGI(MAIN_TAG, "Slave ready - waiting for the first ESP-NOW time sync...");

}