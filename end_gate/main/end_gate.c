/**
 * @file end_gate.c
 * @brief Finish-line analog-comparator gate.
 *
 * Every validated quiet period becomes a finish packet. The gate broadcasts
 * duplicates for reliability and briefly holds its finished indication.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/ana_cmpr.h"
#include "driver/gpio.h"
#include "race_config.h"

#define RED_LED_GPIO       GPIO_NUM_0
#define BURST_COUNT        5
#define BURST_DELAY_MS     5
#define FINISHED_LED_HOLD_MS 2000

/* Command and identifier used by the car to recognize a finish event. */
#define TARGET_COMMAND_ID  2
#define CONFIG_GATE_ID     99

static const char *TAG = "FINISH_GATE";
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
static esp_timer_handle_t idle_reset_timer;
static ana_cmpr_handle_t cmpr_unit;
static TaskHandle_t beam_activity_task_handle;

typedef enum { GATE_IDLE, GATE_FINISHED } GateState;
static volatile GateState current_state = GATE_IDLE;

/*
 * A beam break at the finish line only means something once a car is
 * actually out on course. Without this guard, any stray comparator trigger
 * (insects, glare, wind-blown debris) while no one is racing broadcasts a
 * finish event that the car will accept unconditionally. Arm on the shared
 * start broadcast -- the same one the start gate sends to the car -- and
 * disarm after firing, mirroring the start gate's car_staged_and_waiting
 * gate.
 */
static volatile bool race_in_progress = false;
static uint32_t last_processed_start_sequence = 0;

/** Arm the finish gate after observing the shared start-of-race broadcast. */
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(gate_packet_t)) {
        gate_packet_t *packet = (gate_packet_t *)data;
        if (packet->command_id == 1) {
            /* The start gate sends a burst; only arm on the first copy. */
            if (packet->sequence_num <= last_processed_start_sequence) return;
            last_processed_start_sequence = packet->sequence_num;

            race_in_progress = true;
            ESP_LOGI(TAG, ">>> Start event observed; finish gate is ARMED. <<<");
        }
    }
}

/** Render idle and finished states on the local status LED. */
static void led_indicator_task(void *pvParameters)
{
    while (1) {
        if (current_state == GATE_FINISHED) {
            gpio_set_level(RED_LED_GPIO, 1);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            gpio_set_level(RED_LED_GPIO, 0);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        } else {
            gpio_set_level(RED_LED_GPIO, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(RED_LED_GPIO, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

/** End the visible finish indication and re-arm the gate. */
static void idle_reset_timer_callback(void *arg)
{
    current_state = GATE_IDLE;
    ESP_LOGI(TAG, "Finish gate reset to IDLE; waiting for the next beam break.");
}

/** Broadcast one finish event after the comparator quiet period expires. */
static void beam_broken_timer_callback(void *arg)
{
    /* A beam break is only a finish once a start event has armed the gate. */
    if (!race_in_progress) {
        return;
    }

    current_state = GATE_FINISHED;
    /* Disarm immediately so further activity can't create a duplicate finish. */
    race_in_progress = false;

    global_sequence_counter++;
    save_persisted_sequence_counter(global_sequence_counter);
    gate_packet_t packet = {
        .command_id = TARGET_COMMAND_ID,
        .gate_id = CONFIG_GATE_ID,
        .sequence_num = global_sequence_counter,
    };

    ESP_LOGI(TAG, "GATE BROKEN: finish beam detected; broadcasting command 2, gate 99.");

    for (int i = 0; i < BURST_COUNT; i++) {
        esp_now_send(BROADCAST_MAC, (uint8_t *)&packet, sizeof(packet));
        esp_rom_delay_us(BURST_DELAY_MS * 1000);
    }

    // Keep the finished LED pattern visible before returning to idle.
    esp_timer_start_once(idle_reset_timer, FINISHED_LED_HOLD_MS * 1000);
}

// Timer APIs are not called from the comparator ISR.  The ISR only wakes this
// task; the task performs the timer restart in normal task context.
/** Restart the debounce timer in task context after an ISR notification. */
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
static bool IRAM_ATTR ana_cmpr_cross_callback(
    ana_cmpr_handle_t cmpr,
    const ana_cmpr_cross_event_data_t *edata,
    void *user_ctx)
{
    // Ignore comparator activity until a start event has armed this gate.
    if (!race_in_progress) {
        return false;
    }

    // Do not call esp_timer APIs from the comparator ISR. Notify a task
    // instead; it restarts the inactivity timer outside interrupt context.
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(beam_activity_task_handle,
                           &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

/** Initialize Wi-Fi/ESP-NOW, timers, comparator, LED, and idle loop. */
void app_main(void)
{
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    global_sequence_counter = load_persisted_sequence_counter();
    ESP_LOGI(TAG, "Resuming gate sequence counter from NVS: %lu",
             (unsigned long)global_sequence_counter);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    esp_now_peer_info_t peer = {
        .channel = ESP_NOW_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(peer.peer_addr));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    const esp_timer_create_args_t timer_args = {
        .callback = &beam_broken_timer_callback,
        .name = "beam_timeout",
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

    ana_cmpr_config_t cmpr_config = {
        .unit = 0,
        .clk_src = ANA_CMPR_CLK_SRC_DEFAULT,
        .ref_src = ANA_CMPR_REF_SRC_INTERNAL,
        .cross_type = ANA_CMPR_CROSS_ANY,
    };
    ESP_ERROR_CHECK(ana_cmpr_new_unit(&cmpr_config, &cmpr_unit));

    ana_cmpr_internal_ref_config_t ref_config = {
        .ref_volt = ANA_CMPR_REF_VOLT_70_PCT_VDD,
    };
    ESP_ERROR_CHECK(ana_cmpr_set_internal_reference(cmpr_unit, &ref_config));

    ana_cmpr_event_callbacks_t cbs = {
        .on_cross = ana_cmpr_cross_callback,
    };
    ESP_ERROR_CHECK(ana_cmpr_register_event_callbacks(cmpr_unit, &cbs, NULL));
    ESP_ERROR_CHECK(ana_cmpr_enable(cmpr_unit));

    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << RED_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_conf));
    xTaskCreate(led_indicator_task, "led_indicator", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Finish gate ID 99 is IDLE; waiting for a start event to arm.");
    ESP_LOGI(TAG, "Analog comparator online; target reference is 2.31V.");

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
