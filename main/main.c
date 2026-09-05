#include "main.h"



// Setup GPIOs
void SetupPins() {
    // Enable the power supply to the LED Strip 
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

}


// Timers

static bool IRAM_ATTR led_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;

    portENTER_CRITICAL_ISR(&s_timer_lock);
    /* Predictive slew: shrink/stretch this 3 s interval by the amount of
     * drift we expect to accumulate, based on the last measured ppm. */
    int64_t slew_us = ((int64_t)BLINK_PERIOD_US * s_ppm_estimate * PPM_SLEW_SIGN) / 1000000;
    uint64_t next_target = edata->alarm_value + BLINK_PERIOD_US - slew_us;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = next_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(timer, &alarm_cfg);
    portEXIT_CRITICAL_ISR(&s_timer_lock);

    uint64_t tick = edata->alarm_value;
    xQueueSendFromISR(s_blink_evt_q, &tick, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

static bool IRAM_ATTR imu_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    
    
    // Return true if a high-priority task was awakened to trigger a context switch
    return high_task_awoken == pdTRUE;
}

static void gptimer_init_from_tsf(void)
{
    int64_t tsf_now = esp_wifi_get_tsf_time(WIFI_IF_AP);
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_gptimer_led, (uint64_t)tsf_now));

    uint64_t first_target = (((uint64_t)tsf_now / BLINK_PERIOD_US) + 1) * BLINK_PERIOD_US;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = first_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer_led, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer_led));

    ESP_LOGI(MAIN_TAG, "Led GPTimer seeded from AP TSF=%lld us", (long long)tsf_now);
}

static void discipline_gptimer_step(int64_t offset_us)
{
    if (offset_us > FTM_MAX_STEP_US)  offset_us = FTM_MAX_STEP_US;
    if (offset_us < -FTM_MAX_STEP_US) offset_us = -FTM_MAX_STEP_US;

    portENTER_CRITICAL(&s_timer_lock);
    uint64_t raw = 0;
    gptimer_get_raw_count(s_gptimer_led, &raw);
    int64_t corrected = (int64_t)raw + offset_us;
    if (corrected < 0) corrected = 0;
    gptimer_set_raw_count(s_gptimer_led, (uint64_t)corrected);

    uint64_t next_target = (((uint64_t)corrected / BLINK_PERIOD_US) + 1) * BLINK_PERIOD_US;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = next_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(s_gptimer_led, &alarm_cfg);
    portEXIT_CRITICAL(&s_timer_lock);
}

void init_timers() {

    // create IMU timer
    gptimer_config_t imu_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, 
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&imu_timer_config, &s_gptimer_imu));

    // register callback for IMU timer
    gptimer_event_callbacks_t imu_cbs = {
        .on_alarm = imu_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer_imu, &imu_cbs, NULL));

    // create alarm for IMU timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer_imu, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer_imu));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer_imu));


    // create LED timer
    gptimer_config_t led_timer_cfg = {
        .clk_src      = GPTIMER_CLK_SRC_DEFAULT,
        .direction    = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   /* 1 tick = 1 us */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&led_timer_cfg, &s_gptimer_led));

    // register callback for LED timer
    gptimer_event_callbacks_t led_cbs = { 
        .on_alarm = led_timer_alarm_cb 
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer_led, &led_cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer_led));

    // Alarm for led timer will be set after the first FTM sync, so that the GPTimer is seeded from the AP TSF.

}




// Led
void led_init() {

     /* Enable the power supply to the LED Strip */
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

    led_rcolor = 7;
    led_gcolor = 0;

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
}

static void blink_task(void *arg)
{
    uint64_t tick;
    for (;;) {
        if (xQueueReceive(s_blink_evt_q, &tick, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "blink @ t=%llu us (reference clock)", (unsigned long long)tick);
            led_strip_set_pixel(s_led, 0, led_rcolor, led_gcolor, 0);   /* green flash */
            led_strip_refresh(s_led);
            vTaskDelay(pdMS_TO_TICKS(BLINK_FLASH_MS));
            led_strip_clear(s_led);
        }
    }
}




// Wifi 
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    }  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            memcpy(s_ap_bssid, ap_info.bssid, 6);
            s_ap_channel = ap_info.primary;
            ESP_LOGI(WIFI_TAG, "Connected to master, chan=%d, ftm_responder=%d",  s_ap_channel, ap_info.ftm_responder);
            xEventGroupSetBits(s_wifi_evt_group, WIFI_CONNECTED_BIT);
        }
    
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_evt_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(WIFI_TAG,"connect to the AP fail");


    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *report = (wifi_event_ftm_report_t *)event_data;
        s_ftm_report_num_entries = report->ftm_report_num_entries;
        
        if (report->status == FTM_STATUS_SUCCESS) {
            xEventGroupSetBits(s_wifi_evt_group, FTM_REPORT_BIT);
        } else if (report->status == FTM_STATUS_USER_TERM) {
            // do nothing, user terminated the FTM session
        } else {
            ESP_LOGI(WIFI_TAG, "FTM with Peer("MACSTR") failed! (Status - %d)",  MAC2STR(report->peer_mac), report->status);
            xEventGroupSetBits(s_wifi_evt_group, FTM_FAILURE_BIT);
        }
        
    }
}

static void wifi_sta_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                        ESP_EVENT_ANY_ID,&event_handler,NULL,&instance_any_id));


    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    /* Modem sleep adds tens-of-ms latency jitter to RX/TX, which is
     * poison for timing precision - keep the radio fully awake. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

}




// FTM

static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static int64_t median_i64(int64_t *arr, int n)
{
    qsort(arr, n, sizeof(int64_t), cmp_i64);
    return (n % 2) ? arr[n / 2] : (arr[n / 2 - 1] + arr[n / 2]) / 2;
}

static void process_ftm_report(uint8_t num_entries)
{
    int i;
    wifi_ftm_report_entry_t *ftm_report = NULL;

    int64_t offsets_ps[s_ftm_report_num_entries];
    int64_t rtts_ps[s_ftm_report_num_entries];
    int64_t ppms[s_ftm_report_num_entries];

    if (s_ftm_report_num_entries == 0) {
        return;
    }

    ftm_report = malloc(sizeof(wifi_ftm_report_entry_t) * s_ftm_report_num_entries);
    if (!ftm_report) {
        ESP_LOGE(FTM_TAG, "Failed to alloc buffer for FTM report");
        goto exit;
    }
    bzero(ftm_report, sizeof(wifi_ftm_report_entry_t) * s_ftm_report_num_entries);
    if (ESP_OK != esp_wifi_ftm_get_report(ftm_report, s_ftm_report_num_entries)) {
        ESP_LOGE(FTM_TAG, "Could not get FTM report");
        goto exit;
    }


    for (i = 0; i < s_ftm_report_num_entries; i++) {

        /* T1,T4: master (responder) clock.  T2,T3: slave (initiator) clock. */
        int64_t t1 = (int64_t)ftm_report[i].t1, t2 = (int64_t)ftm_report[i].t2;
        int64_t t3 = (int64_t)ftm_report[i].t3, t4 = (int64_t)ftm_report[i].t4;
        int64_t offset_ps = ((t1 - t2) + (t4 - t3)) / 2;
        int64_t rtt_ps    = (t4 - t1) - (t3 - t2);  
        
        int64_t rtt_us = rtt_ps / 1000000;
        if (rtt_us < 0 || rtt_us > FTM_MAX_PLAUSIBLE_RTT_US) {
            continue;  /* multipath / bogus entry, discard */
        }
        
        ESP_LOGI(FTM_TAG, "%d-Offset: %lld, RTT: %lld", i, offset_ps, rtt_ps);

        offsets_ps[i]   = offset_ps;
        rtts_ps[i]      = rtt_ps;
        ppms[i]         = ftm_report[i].ppm;

    }

    int64_t off_med_ps  = median_i64(offsets_ps, s_ftm_report_num_entries);
    int64_t rtt_med_ps  = median_i64(rtts_ps, s_ftm_report_num_entries);
    int64_t ppm_med     = median_i64(ppms, s_ftm_report_num_entries);
    int64_t offset_us   = off_med_ps / 1000000;
    int64_t rtt_us      = rtt_med_ps / 1000000;

    discipline_gptimer_step(offset_us);
    s_ppm_estimate = (int32_t)ppm_med;

    ESP_LOGI(FTM_TAG, "Time difference %lld, rtt %lld", offset_us, rtt_us);
    s_sync_count++;

exit:
    if (ftm_report) {
        free(ftm_report);
    }
    s_ftm_report_num_entries = 0;
}

static void start_ftm_session(void)
{

    wifi_ftm_initiator_cfg_t ftmi_cfg = {
        .frm_count    = FTM_FRAME_COUNT,
        .burst_period = FTM_BURST_PERIOD,
    };

    memcpy(ftmi_cfg.resp_mac, s_ap_bssid, ETH_ALEN);
    ftmi_cfg.channel = s_ap_channel;

    ESP_LOGI(FTM_TAG, "Requesting FTM session with Frm Count - %d, Burst Period - %dmSec (0: No Preference)",
             ftmi_cfg.frm_count, 
             ftmi_cfg.burst_period*100);

    esp_err_t err = esp_wifi_ftm_initiate_session(&ftmi_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(FTM_TAG, "esp_wifi_ftm_initiate_session failed: %s", esp_err_to_name(err));
    }
}

static void ftm_sync_task(void *arg)
{
    EventBits_t bits;
    uint32_t wait_time_ms = (FTM_BURST_PERIOD * 100) * (FTM_MAX_BURSTS * 2);

    xEventGroupWaitBits(s_wifi_evt_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    /* Let 802.11 TSF adoption settle for a beacon interval or two before
     * we seed the GPTimer, so the coarse sync is already in place. */
    vTaskDelay(pdMS_TO_TICKS(300));
    gptimer_init_from_tsf();

    while (1) {
        start_ftm_session();

        bits = xEventGroupWaitBits(s_wifi_evt_group, FTM_REPORT_BIT | FTM_FAILURE_BIT, pdTRUE, pdFALSE, wait_time_ms / portTICK_PERIOD_MS);
        
        if (bits & FTM_REPORT_BIT) {
            process_ftm_report(FTM_MAX_ENTRIES);
            esp_wifi_ftm_end_session();
        } else if (bits & FTM_FAILURE_BIT) {
            ESP_LOGE(FTM_TAG, "FTM session failed");
        } else {
            ESP_LOGE(FTM_TAG, "FTM session timed out");
            esp_wifi_ftm_end_session();
        }

        vTaskDelay(pdMS_TO_TICKS(FTM_SYNC_PERIOD_MS));
    }
}

void start_ftm(){
    xTaskCreate(ftm_sync_task, "ftm_sync_task", 4096, NULL, 5, NULL);
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

void imu_init() {

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

}



// App main
void Initialize() {

    // Init GIOs
    SetupPins();
    ESP_LOGI(MAIN_TAG, "GPIO pins initialized");

    // Battery init
    //battery.Init();

    // Timers
    init_timers();

    // FLASH Log init
    ESP_ERROR_CHECK(imu_flash_log_init());
    ESP_LOGI(MAIN_TAG, "IMU flash initialized");

    // IMU init
    imu_init();
    ESP_LOGI(MAIN_TAG, "BNO085 and timer initialized");

    // BLE control init
    ESP_ERROR_CHECK(ble_control_init());
    ESP_LOGI(MAIN_TAG, "BLE control initialized");

    // Wifi/FTM
    start_ftm();
    wifi_sta_init();

    // Init led
    led_init();
    ESP_LOGI(MAIN_TAG, "LED initialized"); 

}


void app_main()
{

     // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_blink_evt_q = xQueueCreate(4, sizeof(uint64_t));
    s_wifi_evt_group  = xEventGroupCreate();
    s_ftm_evt_group  = xEventGroupCreate();

    // Init components
    Initialize();

    xTaskCreate(blink_task, "blink_task", 4096, NULL, 10, NULL);         


    //ESP_LOGI(TAG, "Battery voltage read: %i", battery.BatteryVoltageRead());s_wifi_evt_group

    // WARNING: if program reaches end of function app_main() the MCU will restart.
}