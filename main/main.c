#include "main.h"



// Setup GPIOs
void SetupPins() {
    // Enable the power supply to the LED Strip 
    gpio_set_direction(LED_SLP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_SLP_PIN, 1);

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

static bool IRAM_ATTR on_gptimer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;

    uint64_t next_target = edata->alarm_value + BLINK_PERIOD_US;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = next_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    /* Re-arming a manual-reload alarm from inside its own callback is the
     * documented ESP-IDF gptimer pattern for "one-shot, re-armed each time". */
    gptimer_set_alarm_action(timer, &alarm_cfg);

    uint64_t tick = edata->alarm_value;
    xQueueSendFromISR(s_blink_evt_q, &tick, &hp_task_woken);
    return hp_task_woken == pdTRUE;
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
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(WIFI_TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_sta_init()
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                        ESP_EVENT_ANY_ID,&event_handler,NULL,&instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                        IP_EVENT_STA_GOT_IP,&event_handler,NULL,&instance_got_ip));

    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    /* Modem sleep adds tens-of-ms latency jitter to RX/TX, which is
     * poison for timing precision - keep the radio fully awake. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",  CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to ap SSID:%s, password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
    }
}

static void gptimer_init_from_tsf(void)
{
    /* Seed the GPTimer from this radio's own TSF, then let it free-run. */
    gptimer_config_t timer_cfg = {
        .clk_src      = GPTIMER_CLK_SRC_DEFAULT,
        .direction    = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   /* 1 tick = 1 us */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &s_gptimer));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_gptimer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_gptimer));

    int64_t tsf_now = esp_wifi_get_tsf_time(WIFI_IF_AP);
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_gptimer, (uint64_t)tsf_now));

    uint64_t first_target = (((uint64_t)tsf_now / BLINK_PERIOD_US) + 1) * BLINK_PERIOD_US;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = first_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_start(s_gptimer));

    ESP_LOGI(MAIN_TAG, "GPTimer seeded from AP TSF=%lld us", (long long)tsf_now);
}


/* ---------------------------------------------------------------------
 * FTM report handling (v6.0 API: esp_wifi_ftm_get_report)
 * ------------------------------------------------------------------- */
/* Step-correct the running GPTimer by offset_us (+/-) and re-derive the
 * next 3 s alarm boundary, so the hardware compare register is never
 * left pointing at a stale target after the jump. */
static void discipline_gptimer_step(int64_t offset_us)
{
    if (offset_us > FTM_MAX_STEP_US)  offset_us = FTM_MAX_STEP_US;
    if (offset_us < -FTM_MAX_STEP_US) offset_us = -FTM_MAX_STEP_US;

    portENTER_CRITICAL(&s_timer_lock);
    uint64_t raw = 0;
    gptimer_get_raw_count(s_gptimer, &raw);
    int64_t corrected = (int64_t)raw + offset_us;
    if (corrected < 0) corrected = 0;
    gptimer_set_raw_count(s_gptimer, (uint64_t)corrected);

    uint64_t next_target = (((uint64_t)corrected / BLINK_PERIOD_US) + 1) * BLINK_PERIOD_US;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = next_target,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(s_gptimer, &alarm_cfg);
    portEXIT_CRITICAL(&s_timer_lock);
}

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
    if (num_entries == 0) {
        ESP_LOGW(TAG, "FTM report carried no entries");
        return;
    }
    if (num_entries > FTM_MAX_ENTRIES) num_entries = FTM_MAX_ENTRIES;

    wifi_ftm_report_entry_t entries[FTM_MAX_ENTRIES];
    /* v6.0: the raw report is no longer attached to the event; fetch it
     * explicitly into our own buffer. This also frees the driver's
     * internal copy (attention note in esp_wifi.h). */
    esp_err_t err = esp_wifi_ftm_get_report(entries, num_entries);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_ftm_get_report failed: %s", esp_err_to_name(err));
        return;
    }

    int64_t offsets_ps[FTM_MAX_ENTRIES];
    int64_t rtts_ps[FTM_MAX_ENTRIES];
    int64_t ppms[FTM_MAX_ENTRIES];
    int n = 0;

    for (int i = 0; i < num_entries; i++) {
        const wifi_ftm_report_entry_t *e = &entries[i];

        /* T1,T4: master (responder) clock.  T2,T3: slave (initiator) clock. */
        int64_t t1 = (int64_t)e->t1, t2 = (int64_t)e->t2;
        int64_t t3 = (int64_t)e->t3, t4 = (int64_t)e->t4;

        int64_t rtt_ps    = (t4 - t1) - (t3 - t2);              /* Espressif's own formula */
        int64_t offset_ps = ((t1 - t2) + (t4 - t3)) / 2;         /* derived above: master - slave */

        int64_t rtt_us = rtt_ps / 1000000;
        if (rtt_us < 0 || rtt_us > FTM_MAX_PLAUSIBLE_RTT_US) {
            continue;  /* multipath / bogus entry, discard */
        }
        rtts_ps[n]    = rtt_ps;
        offsets_ps[n] = offset_ps;
        ppms[n]       = e->ppm;
        n++;
    }

    if (n == 0) {
        ESP_LOGW(TAG, "All %d FTM entries rejected as outliers", num_entries);
        return;
    }

    int64_t off_med_ps = median_i64(offsets_ps, n);
    int64_t rtt_med_ps = median_i64(rtts_ps, n);
    int64_t ppm_med     = median_i64(ppms, n);

    int64_t offset_us = off_med_ps / 1000000;
    int64_t rtt_us     = rtt_med_ps / 1000000;

    discipline_gptimer_step(offset_us);
    s_ppm_estimate = (int32_t)ppm_med;   /* used by the ISR for slew between syncs */

    s_sync_count++;
    ESP_LOGI(TAG, "FTM sync #%u: %d/%d valid entries, rtt=%lld us, "
                  "phase step=%lld us, drift=%lld ppm",
             (unsigned)s_sync_count, n, num_entries,
             (long long)rtt_us, (long long)offset_us, (long long)ppm_med);
}

static void start_ftm_session(void)
{
    wifi_ftm_initiator_cfg_t ftmi_cfg = {
        .frm_count    = FTM_FRAME_COUNT,
        .burst_period = FTM_BURST_PERIOD,
    };
    memcpy(ftmi_cfg.resp_mac, s_ap_bssid, 6);
    ftmi_cfg.channel = s_ap_channel;

    esp_err_t err = esp_wifi_ftm_initiate_session(&ftmi_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_ftm_initiate_session failed: %s", esp_err_to_name(err));
    }
}

static void ftm_sync_task(void *arg)
{
    xEventGroupWaitBits(s_wifi_evt_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    /* Let 802.11 TSF adoption settle for a beacon interval or two before
     * we seed the GPTimer, so the coarse sync is already in place. */
    vTaskDelay(pdMS_TO_TICKS(300));
    gptimer_init_from_tsf();

    while (1) {
        start_ftm_session();
        xEventGroupWaitBits(s_wifi_evt_group, FTM_REPORT_BIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
        vTaskDelay(pdMS_TO_TICKS(FTM_SYNC_PERIOD_MS));
    }
}

void start_fmt(){
    xTaskCreate(ftm_sync_task, "ftm_sync_task", 4096, NULL, 5, NULL);
}





// ESPNOW time sync
static void timesync_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{

    if (event_id == ESP_EVENT_ESPNOW_TIMESYNC_SYNCED) {
        espnow_timesync_event_t *evt = (espnow_timesync_event_t *)event_data;
        s_time_offset_us = evt->synced_time_us - esp_timer_get_time();
        //ledcolorb = 7;
        //ledcolorr = 0;
        //ESP_LOGI(ESPNOW_TIMESYNC_TAG, "Time synced from " MACSTR ", drift: %" PRId32 " ms", MAC2STR(evt->src_addr), evt->drift_ms);

        // send to influxdb queue
        /*if (evt->drift_ms < CONFIG_ESPNOW_TIMESYNC_MAX_DRIFT_MS && evt->drift_ms > -CONFIG_ESPNOW_TIMESYNC_MAX_DRIFT_MS) {
            incoming_data.drift = evt->drift_ms;
            xQueueSend(influx_queue, &incoming_data, pdMS_TO_TICKS(10));
        }*/
    }
}

static int64_t get_synced_time_us(void)
{
    return esp_timer_get_time() + s_time_offset_us;
}

void espnow_timesync_init() {

    esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, timesync_event_handler, NULL);
    espnow_time_responder_config_t time_config = {
        .max_drift_ms = CONFIG_ESPNOW_TIMESYNC_MAX_DRIFT_MS,
    };
    ESP_ERROR_CHECK(espnow_time_responder_start(&time_config));
    ESP_ERROR_CHECK(espnow_time_responder_request());
    ESP_LOGI(ESPNOW_TIMESYNC_TAG, "Time sync responder started, max drift: %d ms", CONFIG_ESPNOW_TIMESYNC_MAX_DRIFT_MS);

}



// Espnow
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(ESPNOW_TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(ESPNOW_TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = (uint8_t *)malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(ESPNOW_TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(ESPNOW_TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(ESPNOW_TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB ;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(ESPNOW_TAG, "Send send queue fail");
    }
}

static void espnow_task(void *p)
{
    espnow_event_t evt;
    int ret;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;

    while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {

        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

                ESP_LOGD(ESPNOW_TAG, "Send data to " MACSTR ", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

               
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);

                if (ret == ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(ESPNOW_TAG, " Receive %dth broadcast data from: " MACSTR "", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                }
                else if (ret == ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(ESPNOW_TAG, " Receive %dth unicast data from: " MACSTR ", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                }
                else {
                    ESP_LOGI(ESPNOW_TAG, " Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(ESPNOW_TAG, " Callback type error: %d", evt.id);
                break;
        }
    }
}

static void espnow_close()
{
    vQueueDelete(s_espnow_queue);
    s_espnow_queue = NULL;
    esp_now_deinit();
}

static esp_err_t espnow_start() {

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(ESPNOW_TAG, "Create queue fail");
        espnow_close();
        return ESP_FAIL;;
    }

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_config.qsize = CONFIG_APP_ESPNOW_QUEUE_SIZE;
    ESP_ERROR_CHECK( espnow_init(&espnow_config) );

    const esp_now_peer_info_t master_unicast = {
        .peer_addr = MY_RECEIVER_MAC,
        .channel = CONFIG_ESPNOW_CHANNEL,
        .ifidx = ESPNOW_WIFI_IF
    };
    ESP_ERROR_CHECK( esp_now_add_peer(&master_unicast) );

    //xTaskCreate(espnow_task, "espnow_task", 2048, NULL, 4, NULL);

   return ESP_OK;
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



// App main
void Initialize() {

    // Init GIOs
    SetupPins();
    ESP_LOGI(MAIN_TAG, "GPIO pins initialized");

    // Battery init
    //battery.Init();

    // FLASH Log init
    ESP_ERROR_CHECK(imu_flash_log_init());
    ESP_LOGI(MAIN_TAG, "IMU flash initialized");

    // IMU init
    imu_init();
    ESP_LOGI(MAIN_TAG, "BNO085 and timer initialized");

    // BLE control init
    ESP_ERROR_CHECK(ble_control_init());
    ESP_LOGI(MAIN_TAG, "BLE control initialized");

    // Wifi and hw timer from FTM
    wifi_sta_init();
    gptimer_init_from_tsf();

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

    // Init components
    Initialize();

    
    xTaskCreate(blink_task, "blink_task", 4096, NULL, 10, NULL);         
    

    //ESP_LOGI(TAG, "Battery voltage read: %i", battery.BatteryVoltageRead());

    // WARNING: if program reaches end of function app_main() the MCU will restart.
}