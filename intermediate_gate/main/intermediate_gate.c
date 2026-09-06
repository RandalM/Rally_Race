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

// === OVERRIDE THESE FOR EACH PHYSICAL GATE EMBEDDED ON THE TRACK ===
#define TARGET_COMMAND_ID  3   // 1 = GPU Start, 2 = Finish, 3 = Intermediate Split
#define CONFIG_GATE_ID     10  // Unique numeric identifier for this track section, INCREMENT FOR OTHER INTERMEDIATE GATES

static const char *TAG = "FREQ_GATE";
static uint32_t global_sequence_counter = 0;

static esp_timer_handle_t beam_timeout_timer;
static volatile bool is_lockout_active = false;
static int64_t lockout_end_time = 0;

// 1. HARDWARE TIMER CALLBACK: Executed when the frequency stops (Beam is broken)
static void beam_broken_timer_callback(void* arg) {
    int64_t now = esp_timer_get_time();
    
    // Check if we are within the 2-second lockout window to prevent double triggers
    if (is_lockout_active) {
        if (now < lockout_end_time) {
            return; // Ignore false re-triggers while car is still passing
        } else {
            is_lockout_active = false; // Lockout expired naturally
        }
    }

    // Set lock out window parameters (2 seconds = 2,000,000 microseconds)
    is_lockout_active = true;
    lockout_end_time = now + 2000000;

    global_sequence_counter++;
    gate_packet_t packet = {
        .command_id   = TARGET_COMMAND_ID,
        .gate_id      = CONFIG_GATE_ID,
        .sequence_num = global_sequence_counter
    };

    ESP_LOGI(TAG, "Frequency stopped! Beam broken detected. Broadcasting burst...");
    
    // Send out the robust wireless data burst
    for (int i = 0; i < BURST_COUNT; i++) {
        esp_now_send(BROADCAST_MAC, (uint8_t *)&packet, sizeof(packet));
        // Note: Do not block inside an esp_timer callback. Use basic spin loop or short yield.
        esp_rom_delay_us(BURST_DELAY_MS * 1000); 
    }
}

// 2. HARDWARE INTERRUPT HANDLER (ISR): Fires on every single pulse edge
static void IRAM_ATTR gpio_pulse_isr_handler(void* arg) {
    // Only process pulses if the car isn't currently mid-gate (lockout inactive)
    if (!is_lockout_active) {
        // Kick the watchdog: Stop the old countdown and start a fresh one
        esp_timer_stop(beam_timeout_timer);
        esp_timer_start_once(beam_timeout_timer, BEAM_BREAK_TIMEOUT_US);
    }
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

    // Configure GPIO for incoming frequency pulses (Triggers on falling edges)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BEAM_BREAK_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE // Trigger when signal drops from High to Low
    };
    gpio_config(&io_conf);

    // Install the global GPIO ISR service mapping
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // Hook our specific pin handler to our pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(BEAM_BREAK_GPIO, gpio_pulse_isr_handler, (void*) BEAM_BREAK_GPIO));

    ESP_LOGI(TAG, "Frequency Watchdog Gate online. Monitoring pulses on GPIO %d...", BEAM_BREAK_GPIO);

    while (1) {
        // Main thread remains idle, saving power. Everything handles inside hardware interrupts.
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}
