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


#include "led_strip.h"
#include "ble_control.h"
#include "imu_flash_log.h"
#include "bno085.h"

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

static led_strip_handle_t led_strip;
static uint led_rcolor = 0;
static uint led_gcolor = 0;
static gptimer_handle_t   s_gptimer_led      = NULL;
static QueueHandle_t      s_blink_evt_q  = NULL;
static led_strip_handle_t s_led          = NULL;


// Wifi
static EventGroupHandle_t  s_wifi_evt_group = NULL;
static int s_retry_num = 0;
static bool s_ap_started;


#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_SSID           "ESP32_FTM_MASTER"
#define WIFI_PASS           "ftmsync123"
#define WIFI_CHANNEL        6


// FTM
#define FTM_REPORT_BIT      BIT0
#define FTM_FAILURE_BIT     BIT1
#define FTM_FRAME_COUNT             8           /* frames/burst: 0,16,24,32,64           */
#define FTM_BURST_PERIOD            2            /* x100ms - only matters for >1 burst    */
#define FTM_MAX_BURSTS              8
#define FTM_SYNC_PERIOD_MS          15000        /* re-discipline (step) every 15 s       */
#define FTM_MAX_ENTRIES             64
#define FTM_MAX_PLAUSIBLE_RTT_US    20000        /* reject anything absurd (20 ms)        */
#define FTM_MAX_STEP_US             500000       /* clamp any single phase step to 500 ms */
#define PPM_SLEW_SIGN               (-1)         /* flip to +1 if drift correction has the*/
#define DEFAULT_WAIT_TIME_MS        (10 * 1000)
#define ETH_ALEN                    6

static portMUX_TYPE                 s_timer_lock  = portMUX_INITIALIZER_UNLOCKED;
static uint32_t                     s_sync_count = 0;
static uint8_t                      s_ap_bssid[6];
static uint8_t                      s_ap_channel;
static uint8_t                      s_ftm_report_num_entries = 0;
static EventGroupHandle_t           s_ftm_evt_group = NULL;

/* Latest measured drift rate (ppm), applied as a predictive slew between
 * full FTM syncs. Updated only from the FTM report handler (task
 * context); read from the alarm ISR, so kept as a plain int (single
 * aligned word read/write is atomic enough for this non-safety-critical
 * use - no lock needed). */
static volatile int32_t s_ppm_estimate = 0;

static void process_ftm_report(uint8_t num_entries);



// Logs
static const char *MOTION_TAG           = "IMU";
static const char *WIFI_TAG             = "WIFI";
static const char *FTM_TAG              = "FTM";
static const char *ESPNOW_TAG           = "ESPNOW";
static const char *ESPNOW_TIMESYNC_TAG  = "ESPNOW_TIMESYNC";
static const char *LED_TAG              = "LED";
static const char *MAIN_TAG             = "MAIN";


