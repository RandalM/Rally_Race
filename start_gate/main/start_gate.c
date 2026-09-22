/**
 * @file start_gate.c
 * @brief Start-line analog-comparator gate.
 *
 * The gate is armed by a car staging packet, detects a sustained loss of
 * comparator transitions, and broadcasts a start event. The comparator ISR
 * only wakes a task; timer and radio work stay outside interrupt context.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"// Required for microsecond hardware timers
#include "driver/ana_cmpr.h" // Official ESP32-C5 Analog Comparator Driver
#include "race_config.h" // Contains MAX_INTERMEDIATE_GATES, ESP_NOW_CHANNEL, BROADCAST_MAC

/* GPIO is used for the local status indicator. */
#include <stdio.h>
#include "driver/gpio.h"


#define RED_LED_GPIO   GPIO_NUM_0  // Indicator LED (On = Staged/Racing, Off = Idle/Finished)

//#define BEAM_BREAK_GPIO   GPIO_NUM_9  
#define BURST_COUNT       5   // Number of duplicate packets sent per event
#define BURST_DELAY_MS    5   // Delay between burst frames (keeps entire burst under 25ms)
#define FINISHED_LED_HOLD_MS 2000

/* Hardware role and packet command for this firmware image. */
#define TARGET_COMMAND_ID  1   // 1 = Start, 2 = Finish, 3 = Intermediate
#define CONFIG_GATE_ID     0   // 0 = Start line, 10+ = Intermediate/Finish


static const char *TAG = "COMP_GATE"; //comparator based gate
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


typedef enum { CAR_IDLE, RACE_ACTIVE, RACE_FINISHED } CarState;
static volatile CarState current_state = CAR_IDLE;

static esp_timer_handle_t beam_timeout_timer;
static esp_timer_handle_t idle_reset_timer;
static ana_cmpr_handle_t  cmpr_unit;
static TaskHandle_t beam_activity_task_handle;
#if (CONFIG_GATE_ID == 0) && SINGLE_GATE_TRIPLE_PASS
static volatile bool single_gate_race_active = false;
static volatile uint8_t single_gate_pass_count = 0;
static volatile bool single_gate_pass_lockout = false;
static esp_timer_handle_t single_gate_pass_lockout_timer;
#endif



/** Display the current gate state without changing race logic. */
static void led_indicator_task(void *pvParameters) {
    while (1) {
        switch (current_state) {
           case CAR_IDLE:
                // Finished: Hyper-fast rapid strobe for visual celebration
                gpio_set_level(RED_LED_GPIO, 1);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(RED_LED_GPIO, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;

           case RACE_ACTIVE:
                // Single-gate mode: the gate remains armed for the next pass.
                gpio_set_level(RED_LED_GPIO, 1);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                gpio_set_level(RED_LED_GPIO, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

           case RACE_FINISHED:
                // Finished: Hyper-fast rapid strobe for visual celebration
                gpio_set_level(RED_LED_GPIO, 1);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(RED_LED_GPIO, 0);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                break;
        }
    }
}


/* Only Gate 0 receives the car staging/arming request. */
#if (CONFIG_GATE_ID == 0)
    static volatile bool car_staged_and_waiting = false;

    typedef struct { uint8_t command_id; } car_button_packet_t;

    /** Arm the comparator after receiving a valid staging request. */
    static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
        if (len == sizeof(car_button_packet_t)) {
            car_button_packet_t *packet = (car_button_packet_t *)data;
            if (packet->command_id == 1) {
                if (!car_staged_and_waiting
#if SINGLE_GATE_TRIPLE_PASS
                    && !single_gate_race_active
#endif
                ) {
                    car_staged_and_waiting = true;
#if SINGLE_GATE_TRIPLE_PASS
                    single_gate_pass_count = 0;
#endif
                    ESP_LOGI(TAG, ">>> Car staging requested. Start gate is ARMED; waiting for beam break. <<<");
                }
            }
        }
    }
#endif

#if (CONFIG_GATE_ID == 0) && SINGLE_GATE_TRIPLE_PASS
static void single_gate_pass_lockout_callback(void *arg) {
    single_gate_pass_lockout = false;
    ESP_LOGI(TAG, "Single-gate pass lockout expired; gate is re-armed.");
}
#endif



/** Convert a quiet comparator period into one gate event and broadcast it. */
static void beam_broken_timer_callback(void* arg) {
#if (CONFIG_GATE_ID == 0)
    // A beam break is valid only after the car has requested staging.
    if (!car_staged_and_waiting
#if SINGLE_GATE_TRIPLE_PASS
        && !single_gate_race_active
#endif
    ) {
        return;
    }
#if SINGLE_GATE_TRIPLE_PASS
    if (single_gate_pass_lockout) {
        return;
    }
#endif
#endif

    uint8_t event_command_id = TARGET_COMMAND_ID;

#if (CONFIG_GATE_ID == 0) && SINGLE_GATE_TRIPLE_PASS
    if (!single_gate_race_active) {
        // First pass starts the race and leaves this gate armed.
        single_gate_race_active = true;
        single_gate_pass_count = 1;
        car_staged_and_waiting = false;
        current_state = RACE_ACTIVE;
        event_command_id = 1;
        ESP_LOGI(TAG, ">>> Single-gate pass 1/%d: race started; gate remains armed. <<<",
                 SINGLE_GATE_FINISH_PASS);
    } else {
        single_gate_pass_count++;
        event_command_id = (single_gate_pass_count >= SINGLE_GATE_FINISH_PASS) ? 2 : 3;
        ESP_LOGI(TAG, ">>> Single-gate pass %d/%d: sending command %d. <<<",
                 single_gate_pass_count, SINGLE_GATE_FINISH_PASS, event_command_id);

        if (event_command_id == 2) {
            single_gate_race_active = false;
            current_state = RACE_FINISHED;
        }
    }
#else
    current_state = RACE_FINISHED;
#endif

    global_sequence_counter++;
    save_persisted_sequence_counter(global_sequence_counter);
    gate_packet_t packet = {
        .command_id   = event_command_id,
        .gate_id      = CONFIG_GATE_ID,
        .sequence_num = global_sequence_counter
    };

    ESP_LOGI(TAG, "💥 GATE BROKEN: beam break detected; broadcasting start signal...");
    
    for (int i = 0; i < BURST_COUNT; i++) {
        esp_now_send(BROADCAST_MAC, (uint8_t *)&packet, sizeof(packet));
        esp_rom_delay_us(BURST_DELAY_MS * 1000); 
    }

#if (CONFIG_GATE_ID == 0) && SINGLE_GATE_TRIPLE_PASS
    // Ignore additional comparator activity while this physical pass clears
    // the sensor. The timer re-arms the gate for the next pass.
    single_gate_pass_lockout = true;
    ESP_ERROR_CHECK(esp_timer_start_once(single_gate_pass_lockout_timer,
                                         SINGLE_GATE_PASS_LOCKOUT_MS * 1000));
#endif

#if (CONFIG_GATE_ID == 0)
    // Disarm immediately so comparator activity cannot create a duplicate event
    // while the finished LED pattern is being displayed.
#if SINGLE_GATE_TRIPLE_PASS
    if (event_command_id == 2) {
        car_staged_and_waiting = false;
    }
#else
    car_staged_and_waiting = false;
#endif
#endif

#if !((CONFIG_GATE_ID == 0) && SINGLE_GATE_TRIPLE_PASS)
    // Keep the finished state visible before returning to idle.
    esp_timer_start_once(idle_reset_timer, FINISHED_LED_HOLD_MS * 1000);
#else
    if (event_command_id == 2) {
        // Keep the finished state visible before returning to idle.
        esp_timer_start_once(idle_reset_timer, FINISHED_LED_HOLD_MS * 1000);
    }
#endif
}

/** Return the local state/LED to idle after the finished indication. */
static void idle_reset_timer_callback(void *arg) {
    current_state = CAR_IDLE;
#if (CONFIG_GATE_ID == 0)
    ESP_LOGI(TAG, "Start gate reset to IDLE; waiting for the next staging request.");
#else
    ESP_LOGI(TAG, "Gate reset to IDLE; waiting for the next beam break.");
#endif
}

// Timer APIs are not called from the comparator ISR.  The ISR only wakes this
// task; the task performs the timer restart in normal task context.
static void beam_activity_task(void *arg)
{
    uint32_t notification_value;

    while (true) {
        xTaskNotifyWait(0, UINT32_MAX, &notification_value, portMAX_DELAY);

        esp_timer_stop(beam_timeout_timer);
        ESP_ERROR_CHECK(esp_timer_start_once(beam_timeout_timer,
                                             BEAM_BREAK_TIMEOUT_US));
    }
}

/* Comparator ISR: notify the task and return immediately. */
static bool IRAM_ATTR ana_cmpr_cross_callback(ana_cmpr_handle_t cmpr, const ana_cmpr_cross_event_data_t *edata, void *user_ctx) {
#if (CONFIG_GATE_ID == 0)
    // Ignore comparator activity until the car has staged.
    if (!car_staged_and_waiting
#if SINGLE_GATE_TRIPLE_PASS
        && !single_gate_race_active
#endif
    ) {
        return false;
    }
#endif

    // Do not call esp_timer APIs from the comparator ISR. Notify a task
    // instead; it restarts the inactivity timer outside interrupt context.
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(beam_activity_task_handle,
                           &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}


/** Initialize radio, timers, comparator, LED, and the idle service loop. */
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
    
#if (CONFIG_GATE_ID == 0)
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
#else
    ESP_ERROR_CHECK(esp_now_init());
#endif
    esp_now_peer_info_t peer = { .channel = ESP_NOW_CHANNEL, .ifidx = WIFI_IF_STA, .encrypt = false };
    memcpy(peer.peer_addr, BROADCAST_MAC, 6); 
    esp_now_add_peer(&peer);

    // 1. Create the hardware watchdog timer
    const esp_timer_create_args_t timer_args = {
        .callback = &beam_broken_timer_callback,
        .name = "beam_timeout"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &beam_timeout_timer));

    const esp_timer_create_args_t idle_reset_timer_args = {
        .callback = &idle_reset_timer_callback,
        .name = "idle_reset",
    };
    ESP_ERROR_CHECK(esp_timer_create(&idle_reset_timer_args, &idle_reset_timer));

    BaseType_t task_created = xTaskCreate(beam_activity_task,
                                         "beam_activity",
                                         3072,
                                         NULL,
                                         10,
                                         &beam_activity_task_handle);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

#if (CONFIG_GATE_ID == 0) && SINGLE_GATE_TRIPLE_PASS
    const esp_timer_create_args_t single_gate_pass_lockout_timer_args = {
        .callback = &single_gate_pass_lockout_callback,
        .name = "single_gate_lockout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&single_gate_pass_lockout_timer_args,
                                     &single_gate_pass_lockout_timer));
#endif

    // 2. CONFIGURE ANALOG COMPARATOR UNIT 0
    ana_cmpr_config_t cmpr_config = {
        .unit           = 0,
        .clk_src        = ANA_CMPR_CLK_SRC_DEFAULT,
        .ref_src        = ANA_CMPR_REF_SRC_INTERNAL,  // Use ESP32 internal voltage divider
        .cross_type     = ANA_CMPR_CROSS_ANY,         // Register hooks for any boundary transition
    };
    ESP_ERROR_CHECK(ana_cmpr_new_unit(&cmpr_config, &cmpr_unit)); // Allocate handle

    // 3. SET VOLTAGE COMPARISON THRESHOLD
    // Step configuration assigns V_REF in 10% blocks of V_DD (3.3V)
    // ANA_CMPR_INTERNAL_REF_70_PERCENT_VDD creates a ~2.31V trigger threshold line.
    ana_cmpr_internal_ref_config_t ref_config = {
        .ref_volt = ANA_CMPR_REF_VOLT_70_PCT_VDD
    };
    ESP_ERROR_CHECK(ana_cmpr_set_internal_reference(cmpr_unit, &ref_config)); // Apply reference

    // 4. ATTACH THE ANALOG INTERRUPT CALLBACK
    ana_cmpr_event_callbacks_t cbs = {
        .on_cross = ana_cmpr_cross_callback
    };
    ESP_ERROR_CHECK(ana_cmpr_register_event_callbacks(cmpr_unit, &cbs, NULL)); // Mount interrupt mapping

    // 5. ENABLE HARDWARE SUBSYSTEM
    ESP_ERROR_CHECK(ana_cmpr_enable(cmpr_unit)); // Power on comparator macro block

    ESP_LOGI(TAG, "Start gate is IDLE; waiting for a car staging request.");
    ESP_LOGI(TAG, "Analog comparator online; waiting for beam break after staging (target: 2.31V reference).");
    gpio_config_t led_conf= { 
        .pin_bit_mask = (1ULL << RED_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_LOGI("STARTUP", "LED Almost created!");

    gpio_config(&led_conf);

    xTaskCreate(led_indicator_task, "led_indicator", 2048, NULL, 1, NULL);
    
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}
