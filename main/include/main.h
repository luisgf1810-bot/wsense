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
#include "imu.h"



// Logs
static const char *TAG = "MAIN";




// IMU
#define SECTOR_SIZE             4096UL
// 16KB RAM buffer to absorb flash erase latency
#define STREAM_BUFFER_SIZE      (SECTOR_SIZE * 4)  
#define IMU_SAMPLING_RATE_HZ    1000                
#define SENS_ON_PIN 18UTAG
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



// LED
#define LED_SLP_PIN   20
#define LED_PIN   19                
#define LED_STRIP_NUM_PIXELS 1      
#define BLINKER_MIN_SCHEDULE_AHEAD_US 5000ULL /* 5 ms */

static led_strip_handle_t   s_led_strip    = NULL;
static uint                 gcolor=7;




// LED
#define LED_SLP_PIN   20
#define LED_PIN   19                
#define LED_STRIP_NUM_PIXELS 1      
#define TIMER_RESOLUTION_HZ        (1000000ULL) // 1 MHz (1 tick = 1 us)
#define TIMESYNC_BROADCAST_INTERVAL_MS 5000
#define BLINKER_MIN_CORRECTION_US 200ULL
#define BLINKER_MIN_SCHEDULE_AHEAD_US 5000ULL /* 5 ms */

static TaskHandle_t         s_ledtask      = NULL;
static gptimer_handle_t     s_gptimer_led  = NULL;
static QueueHandle_t        s_blink_evt_q  = NULL;
static led_strip_handle_t   s_led_strip    = NULL;
static bool                 s_timer_started = false;
static bool                 s_sync_state = false;
static uint                 rcolor=7;
static uint                 gcolor=0;
static uint                 sync_count=0;
static uint                 ondelay=80;
static uint64_t             period=3000000;
static int                  s_last_applied_state = -1;
static portMUX_TYPE         s_timer_lock  = portMUX_INITIALIZER_UNLOCKED;

// ESPNOW time sync
#define TS_REPORT_BIT      BIT0
#define TS_FAILURE_BIT     BIT1
#define TS_SYNC_PERIOD_MS  1000  

static int64_t              s_time_offset_us = 0;
static uint32_t             s_sync_count = 0;



/*  Battery */
esp_err_t init_battery() ;

/* Initialize led strip */
esp_err_t init_led(void) ;

/* Initialize Wi-Fi & ESP-NOW TIME Sync */
esp_err_t init_espnow_timesync(void) ;

/* GPTimer Init and ISR Callback  */
static uint64_t ticks_to_next_boundary(uint64_t phase_now);
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,  void *user_ctx);
esp_err_t init_gptimer(uint64_t phase_now) ;
esp_err_t gptimer_arm_next(uint64_t phase_now);
static void timer_task(void *arg);


/* IMU  */
static void on_sensor_data(bno085_handle_t handle, const bno085_sensor_value_t *value, void *ctx);
esp_err_t init_imu() ;


/* Flash functions */
static inline bool seq_is_newer(uint32_t a, uint32_t b);
void imu_flash_log_get_stats(imu_log_stats_t *out);
esp_err_t init_flash(void) ;
esp_err_t flash_log_start(void);
esp_err_t flash_log_stop(void) ;
esp_err_t imu_flash_log_flush_partial(void);
esp_err_t imu_flash_log_read_sector_raw(uint32_t sector_index, void *out_buf_4096_bytes);
static void write_sector_to_flash(log_sector_t *sec);


/* Flash functions */
static inline bool seq_is_newer(uint32_t a, uint32_t b);
void imu_flash_log_get_stats(imu_log_stats_t *out);
esp_err_t init_flash(void);
esp_err_t flash_log_start(void);
esp_err_t flash_log_stop(void);
esp_err_t imu_flash_log_flush_partial(void);
esp_err_t imu_flash_log_read_sector_raw(uint32_t sector_index, void *out_buf_4096_bytes);
static void write_sector_to_flash(log_sector_t *sec);
static void flash_task(void *arg);


/* BLE Commands */
void start_imulogs() ;
void stop_imulogs() ;