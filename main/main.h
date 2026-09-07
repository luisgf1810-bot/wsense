#pragma once

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_partition.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "esp_sleep.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_now.h"
#include "espnow_time.h"
#include "espnow_utils.h"

#include "led_strip.h"
#include "ble_control.h"
#include "imu_flash_log.h"
#include "bno085.h"


// Logs
static const char *MOTION_TAG           = "IMU";
static const char *ESPNOW_TIMESYNC_TAG  = "ESPNOW_TIMESYNC";
static const char *LED_TAG              = "LED";
static const char *MAIN_TAG             = "MAIN";



// IMU
#define SECTOR_SIZE             4096UL
// 16KB RAM buffer to absorb flash erase latency
#define STREAM_BUFFER_SIZE      (SECTOR_SIZE * 4)  
#define IMU_SAMPLING_RATE_HZ    1000                
#define SENS_ON_PIN 18U
#define MOTION_WAKEUP_PIN 7U

// 10-byte packed structural representation of one IMU reading 
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us; 
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} imu_sample_t;

static StreamBufferHandle_t xImuStreamBuffer = NULL;
static bno085_handle_t      bno085;
static gptimer_handle_t     s_gptimer_imu = NULL;

float _motion_data[23] = { 0.0 };
uint8_t _i2c_write_array[10] = { 0 };
uint8_t _i2c_read_array[10] = { 0 };
uint8_t _i2c_write_size = 0;
float x = 0.0;  
float y = 0.0;  
float z = 0.0;  
static int64_t start_time, end_time  = 0;  


// LED
#define LED_SLP_PIN   20
#define LED_PIN   19                
#define LED_STRIP_NUM_PIXELS 1      
#define BLINK_PERIOD_US 3000000ULL   /* 3 seconds, in GPTimer ticks (1 tick = 1 us) */
#define BLINK_FLASH_MS  150          /* visible on-time of the flash              */
#define ON_DELAY_US  (50  * 1000)   // 50 ms ON
#define OFF_DELAY_US (5000 * 1000)  // 5000 ms OFF


static gptimer_handle_t     s_gptimer_led  = NULL;
static QueueHandle_t        s_blink_evt_q  = NULL;
static led_strip_handle_t   s_led          = NULL;




// ESPNOW time sync
#define TS_REPORT_BIT      BIT0
#define TS_FAILURE_BIT     BIT1
#define TS_SYNC_PERIOD_MS  1000  // 5 seconds

static TaskHandle_t         timesync_task_handle = NULL;
static EventGroupHandle_t   s_ts_evt_group = NULL;
static int64_t              s_time_offset_us = 0;
static int64_t              s_time_offset_ms = 0;

static portMUX_TYPE         s_timer_lock  = portMUX_INITIALIZER_UNLOCKED;
static uint32_t             s_sync_count = 0;
static int64_t              s_next_alarm_target_us = 0;
static int64_t get_synced_time_us(void);
