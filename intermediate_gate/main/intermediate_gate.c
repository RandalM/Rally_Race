/**
 * @file intermediate_gate.c
 * @brief Frequency/beam checkpoint gate.
 *
 * GPIO falling edges represent sensor activity. Each edge resets a quiet
 * timer; when edges stop, one split packet is broadcast. A local lockout
 * prevents one vehicle crossing from producing multiple splits.
 */

#include <stdio.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_timer.h" // Required for microsecond hardware timers
#include "esp_rom_sys.h"  // Houses the explicit declaration for ets_delay_us
#include "race_config.h"

#define BEAM_BREAK_GPIO   GPIO_NUM_23
#define BURST_COUNT       5   // Number of duplicate packets sent per beam-break
#define BURST_DELAY_MS    5   // Delay between bursts (keeps total burst under 25ms)

/* Change these values when building firmware for another checkpoint. */
#define TARGET_COMMAND_ID  3   // 1 = GPU Start, 2 = Finish, 3 = Intermediate Split
#define CONFIG_GATE_ID     10  // Unique numeric identifier for this track section, INCREMENT FOR OTHER INTERMEDIATE GATES

static const char *TAG = "FREQ_GATE";
static uint32_t global_sequence_counter = 0;

/*
 * The car remembers the highest sequence number it has ever accepted from
 * each gate_id and silently discards anything at or below that value. If
 * this gate reboots, global_sequence_counter resets to 0 while the car's
 * memory of the last value it saw does not -- every real event this gate
 * sends after that is discarded until the counter climbs back above what
 * the car already trusts. Persisting the counter in NVS keeps it
 * monotonically increasing across reboots instead of just within one
 * power-on session.
 */
#define SEQ_NVS_NAMESPACE "gate_seq"
#define SEQ_NVS_KEY       "seq"

static uint32_t load_persisted_sequence_counter(void) {
    nvs_handle_t handle;
    uint32_t value = 0;
    if (nvs_open(SEQ_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u32(handle, SEQ_NVS_KEY, &value);
        nvs_close(handle);
    }
    return value;
}

static void save_persisted_sequence_counter(uint32_t value) {
    nvs_handle_t handle;
    if (nvs_open(SEQ_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u32(handle, SEQ_NVS_KEY, value);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

static esp_timer_handle_t beam_timeout_timer;
static TaskHandle_t beam_activity_task_handle;
static volatile bool is_lockout_active = false;
static int64_t lockout_end_time = 0;

/* Runs after the sensor has been quiet long enough to represent a crossing. */
static void beam_broken_timer_callback(void* arg) {
    int64_t now = esp_timer_get_time();
    
    /* Suppress duplicate events while the same vehicle clears the gate. */
    if (is_lockout_active) {
        if (now < lockout_end_time) {
            return; // Ignore false re-triggers while car is still passing
        } else {
            is_lockout_active = false; // Lockout expired naturally
        }
    }

    /* Start the local two-second lockout interval. */
    is_lockout_active = true;
    lockout_end_time = now + 2000000;

    global_sequence_counter++;
    save_persisted_sequence_counter(global_sequence_counter);
    gate_packet_t packet = {
        .command_id   = TARGET_COMMAND_ID,
        .gate_id      = CONFIG_GATE_ID,
        .sequence_num = global_sequence_counter
    };

    ESP_LOGI(TAG, "Frequency stopped! Beam broken detected. Broadcasting burst...");
    
    /* Send duplicate packets; the car deduplicates them by sequence number. */
    for (int i = 0; i < BURST_COUNT; i++) {
        esp_now_send(BROADCAST_MAC, (uint8_t *)&packet, sizeof(packet));
        // Note: Do not block inside an esp_timer callback. Use basic spin loop or short yield.
        esp_rom_delay_us(BURST_DELAY_MS * 1000); 
    }
}

// Timer APIs are not called from the GPIO ISR. The ISR only wakes this task;
// the task performs the timer restart in normal task context.
static void beam_activity_task(void *arg)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        esp_timer_stop(beam_timeout_timer);
        ESP_ERROR_CHECK(esp_timer_start_once(beam_timeout_timer,
                                             BEAM_BREAK_TIMEOUT_US));
    }
}

/* GPIO ISR: defer timer work to the notification task. */
static void IRAM_ATTR gpio_pulse_isr_handler(void* arg) {
    /* Ignore edges during lockout; otherwise notify the worker task. */
    if (!is_lockout_active) {
        // Notify the task to kick the watchdog outside interrupt context.
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(beam_activity_task_handle,
                               &higher_priority_task_woken);
        if (higher_priority_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

void app_main(void) {
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    global_sequence_counter = load_persisted_sequence_counter();
    ESP_LOGI(TAG, "Resuming gate sequence counter from NVS: %lu",
             (unsigned long)global_sequence_counter);

    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    
    esp_now_init();
    esp_now_peer_info_t peer = { .channel = ESP_NOW_CHANNEL, .ifidx = WIFI_IF_STA, .encrypt = false };
    memcpy(peer.peer_addr, BROADCAST_MAC, 6); 
    esp_now_add_peer(&peer);

    // Create the specialized hardware watchdog timer
    const esp_timer_create_args_t timer_args = {
        .callback = &beam_broken_timer_callback,
        .name = "beam_timeout"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &beam_timeout_timer));

    BaseType_t task_created = xTaskCreate(beam_activity_task,
                                         "beam_activity",
                                         3072,
                                         NULL,
                                         10,
                                         &beam_activity_task_handle);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    /* Falling edges form the sensor heartbeat used by the quiet timer. */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BEAM_BREAK_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE // Trigger when signal drops from High to Low
    };
    gpio_config(&io_conf);

    /* Install and attach the GPIO interrupt handler. */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // Hook our specific pin handler to our pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(BEAM_BREAK_GPIO, gpio_pulse_isr_handler, (void*) BEAM_BREAK_GPIO));

    ESP_LOGI(TAG, "Frequency Watchdog Gate online. Monitoring pulses on GPIO %d...", BEAM_BREAK_GPIO);

    while (1) {
        // Main thread remains idle, saving power. Everything handles inside hardware interrupts.
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}
