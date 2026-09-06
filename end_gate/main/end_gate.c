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

// Final gate identifiers: command 2 means finish, gate 99 is the finish gate.
#define TARGET_COMMAND_ID  2
#define CONFIG_GATE_ID     99

static const char *TAG = "FINISH_GATE";
static uint32_t global_sequence_counter = 0;
static esp_timer_handle_t beam_timeout_timer;
static esp_timer_handle_t idle_reset_timer;
static ana_cmpr_handle_t cmpr_unit;

typedef enum { GATE_IDLE, GATE_FINISHED } GateState;
static volatile GateState current_state = GATE_IDLE;

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

static void idle_reset_timer_callback(void *arg)
{
    current_state = GATE_IDLE;
    ESP_LOGI(TAG, "Finish gate reset to IDLE; waiting for the next beam break.");
}

static void beam_broken_timer_callback(void *arg)
{
    current_state = GATE_FINISHED;

    global_sequence_counter++;
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

static bool IRAM_ATTR ana_cmpr_cross_callback(
    ana_cmpr_handle_t cmpr,
    const ana_cmpr_cross_event_data_t *edata,
    void *user_ctx)
{
    // Restart the inactivity timer on every comparator transition. When the
    // beam signal stops, the timer callback reports one finish event.
    esp_timer_stop(beam_timeout_timer);
    esp_timer_start_once(beam_timeout_timer, BEAM_BREAK_TIMEOUT_US);
    return false;
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
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

    ESP_LOGI(TAG, "Finish gate ID 99 is IDLE; waiting for beam break.");
    ESP_LOGI(TAG, "Analog comparator online; target reference is 2.31V.");

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
