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

//RAND SPECIAL
#include <stdio.h>
#include "driver/gpio.h"


#define RED_LED_GPIO   GPIO_NUM_0  // Indicator LED (On = Staged/Racing, Off = Idle/Finished)

//#define BEAM_BREAK_GPIO   GPIO_NUM_9  
#define BURST_COUNT       5   // Number of duplicate packets sent per event
#define BURST_DELAY_MS    5   // Delay between burst frames (keeps entire burst under 25ms)
#define FINISHED_LED_HOLD_MS 2000

// === CONFIGURATION AT THE TOP OF YOUR FILE ===
#define TARGET_COMMAND_ID  1   // 1 = Start, 2 = Finish, 3 = Intermediate
#define CONFIG_GATE_ID     0   // 0 = Start line, 10+ = Intermediate/Finish


static const char *TAG = "COMP_GATE"; //comparator based gate
static uint32_t global_sequence_counter = 0;


typedef enum { CAR_IDLE, RACE_FINISHED } CarState;
static volatile CarState current_state = CAR_IDLE;

static esp_timer_handle_t beam_timeout_timer;
static esp_timer_handle_t idle_reset_timer;
static ana_cmpr_handle_t  cmpr_unit;



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


// Only compile the staging logic if this specific board is flashed as Gate 0
#if (CONFIG_GATE_ID == 0)
    static volatile bool car_staged_and_waiting = false;

    typedef struct { uint8_t command_id; } car_button_packet_t;

    static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
        if (len == sizeof(car_button_packet_t)) {
            car_button_packet_t *packet = (car_button_packet_t *)data;
            if (packet->command_id == 1) {
                if (!car_staged_and_waiting) {
                    car_staged_and_waiting = true;
                    ESP_LOGI(TAG, ">>> Car staging requested. Start gate is ARMED; waiting for beam break. <<<");
                }
            }
        }
    }
#endif



static void beam_broken_timer_callback(void* arg) {
#if (CONFIG_GATE_ID == 0)
    // A beam break is valid only after the car has requested staging.
    if (!car_staged_and_waiting) {
        return;
    }
#endif
    current_state = RACE_FINISHED;

    global_sequence_counter++;
    gate_packet_t packet = {
        .command_id   = TARGET_COMMAND_ID,
        .gate_id      = CONFIG_GATE_ID,
        .sequence_num = global_sequence_counter
    };

    ESP_LOGI(TAG, "💥 GATE BROKEN: beam break detected; broadcasting start signal...");
    
    for (int i = 0; i < BURST_COUNT; i++) {
        esp_now_send(BROADCAST_MAC, (uint8_t *)&packet, sizeof(packet));
        esp_rom_delay_us(BURST_DELAY_MS * 1000); 
    }

#if (CONFIG_GATE_ID == 0)
    // Disarm immediately so comparator activity cannot create a duplicate event
    // while the finished LED pattern is being displayed.
    car_staged_and_waiting = false;
#endif

    // Keep the finished state visible before returning to idle.
    esp_timer_start_once(idle_reset_timer, FINISHED_LED_HOLD_MS * 1000);
}

static void idle_reset_timer_callback(void *arg) {
    current_state = CAR_IDLE;
#if (CONFIG_GATE_ID == 0)
    ESP_LOGI(TAG, "Start gate reset to IDLE; waiting for the next staging request.");
#else
    ESP_LOGI(TAG, "Gate reset to IDLE; waiting for the next beam break.");
#endif
}

// === NEW INTERRUPT HANDLER: Fires whenever the analog wave crosses your threshold ===
static bool IRAM_ATTR ana_cmpr_cross_callback(ana_cmpr_handle_t cmpr, const ana_cmpr_cross_event_data_t *edata, void *user_ctx) {
#if (CONFIG_GATE_ID == 0)
    // Ignore comparator activity until the car has staged.
    if (!car_staged_and_waiting) {
        return false;
    }
#endif

    // Restart the inactivity timer on each comparator transition. When the
    // beam signal stops, the timer callback reports one beam break.
    esp_timer_stop(beam_timeout_timer);
    esp_timer_start_once(beam_timeout_timer, BEAM_BREAK_TIMEOUT_US);
    return false; // Return true if a higher priority task is awoken
}


void app_main(void) {
    nvs_flash_init();
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
