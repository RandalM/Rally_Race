#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "race_config.h" // Contains MAX_INTERMEDIATE_GATES, ESP_NOW_CHANNEL, BROADCAST_MAC
#include "esp_rom_sys.h" // For delay function

#define BUTTON_GPIO     GPIO_NUM_19  // Onboard BOOT Button for staging
#define GREEN_LED_GPIO   GPIO_NUM_10  // Indicator LED (On = Staged/Racing, Off = Idle/Finished)
#define RED_LED_GPIO   GPIO_NUM_6  // Indicator LED (On = Staged/Racing, Off = Idle/Finished)
#define BLUE_LED_GPIO   GPIO_NUM_0  // Indicator LED (On = Staged/Racing, Off = Idle/Finished)

#define TELEMETRY_BURST_COUNT 5
#define STAGING_TIMEOUT_MS    30000

#define MULTI_LED_PIN_MASK ((1ULL << GREEN_LED_GPIO) | (1ULL << RED_LED_GPIO) | (1ULL << BLUE_LED_GPIO))

static const char *TAG = "CAR_ESP32";

typedef enum { CAR_IDLE, WAITING_FOR_GPU_BEAM, RACING, RACE_FINISHED } CarState;
static volatile CarState current_state = CAR_IDLE;
static uint64_t race_start_time = 0;
static esp_timer_handle_t staging_timeout_timer;

static void staging_timeout_callback(void *arg) {
    if (current_state == WAITING_FOR_GPU_BEAM) {
        current_state = CAR_IDLE;
        ESP_LOGW(TAG, "Staging timed out after %d seconds; car reset to IDLE.",
                 STAGING_TIMEOUT_MS / 1000);
    }
}

// Local storage array for the active race session
static telemetry_packet_t live_race_telemetry;

// Global tracking array for burst sequence deduplication (Index maps to Gate ID)
static uint32_t last_processed_sequence[100] = {0}; 

// Explicit structure matching the gate's broadcast layout
typedef struct {
    uint8_t command_id; 
    uint8_t gate_id;     
    uint32_t sequence_num;  
} inbound_gate_packet_t;


// HELPER: Write telemetry struct directly into NVS Flash Memory
void save_race_to_flash(telemetry_packet_t *data) {
    nvs_handle_t my_handle;
    if (nvs_open("race_storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        // Save the raw byte block safely to flash memory
        nvs_set_blob(my_handle, "last_race", data, sizeof(telemetry_packet_t));
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "💾 Race telemetry securely backed up to NVS Flash memory.");
    }
}

// HELPER: Read and output historical backup on system startup
void load_and_print_flash_backup(void) {
    nvs_handle_t my_handle;
    telemetry_packet_t backup;
    size_t required_size = sizeof(telemetry_packet_t);

    if (nvs_open("race_storage", NVS_READONLY, &my_handle) == ESP_OK) {
        if (nvs_get_blob(my_handle, "last_race", &backup, &required_size) == ESP_OK) {
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "⚠️ UNREPORTED BACKUP RACE FOUND IN FLASH:");
            ESP_LOGI(TAG, "Total Elapsed Time : %.3f seconds", (double)backup.total_race_time_ms / 1000.0);
            ESP_LOGI(TAG, "Checkpoints Passed : %d", backup.recorded_splits_count);
            for (int i = 0; i < backup.recorded_splits_count; i++) {
                ESP_LOGI(TAG, " -> Gate ID %3d | Clock: %.3f s", backup.splits[i].gate_id, (double)backup.splits[i].split_time_ms / 1000.0);
            }
            ESP_LOGI(TAG, "========================================");
        } else {
            ESP_LOGI(TAG, "No historical race records found in flash storage.");
        }
        nvs_close(my_handle);
    }
}

static void led_indicator_task(void *pvParameters) {
    while (1) {
        switch (current_state) {
            case CAR_IDLE:
                ESP_LOGW(TAG, "Waiting patiently!");
                // Normal Idle: Soft, slow blink (1 second on, 1 second off)
                gpio_set_level(BLUE_LED_GPIO, 0);
                gpio_set_level(GREEN_LED_GPIO, 1);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(GREEN_LED_GPIO, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;

            case WAITING_FOR_GPU_BEAM:
                gpio_set_level(GREEN_LED_GPIO, 0);
                // Staged & Armed: Fast double-blink to show it's ready to launch
                gpio_set_level(BLUE_LED_GPIO, 1); vTaskDelay(100 / portTICK_PERIOD_MS);
                gpio_set_level(BLUE_LED_GPIO, 0); vTaskDelay(100 / portTICK_PERIOD_MS);
                gpio_set_level(BLUE_LED_GPIO, 1); vTaskDelay(100 / portTICK_PERIOD_MS);
                gpio_set_level(BLUE_LED_GPIO, 0); vTaskDelay(500 / portTICK_PERIOD_MS);
                break;

            case RACING:
                gpio_set_level(GREEN_LED_GPIO, 0);
                // Active Race: Solid ON so you can see the car moving in the woods
                gpio_set_level(BLUE_LED_GPIO, 1);
                vTaskDelay(100 / portTICK_PERIOD_MS); 
                break;

            case RACE_FINISHED:
                // Finished: Hyper-fast rapid strobe for visual celebration
                gpio_set_level(GREEN_LED_GPIO, 0);
                gpio_set_level(BLUE_LED_GPIO, 0);
                gpio_set_level(RED_LED_GPIO, 1);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(RED_LED_GPIO, 0);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                break;
        }
    }
}

static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(inbound_gate_packet_t)) {
        inbound_gate_packet_t *packet = (inbound_gate_packet_t *)data;
        uint64_t current_time = esp_timer_get_time();

        // Safety array bounds check for Gate IDs
        if (packet->gate_id >= 100) return;

        // DEDUPLICATION FILTER: If sequence is old or repeated, drop it instantly
        if (packet->sequence_num <= last_processed_sequence[packet->gate_id])return;

        // Lock out the rest of this gate's redundancy burst immediately
        last_processed_sequence[packet->gate_id] = packet->sequence_num;

        // 1. START GATE TRIGGER
        if (packet->command_id == 1 && current_state == WAITING_FOR_GPU_BEAM) {
            esp_timer_stop(staging_timeout_timer);
            race_start_time = current_time;
            current_state = RACING;
            
            live_race_telemetry.recorded_splits_count = 0; 
            memset(live_race_telemetry.splits, 0, sizeof(live_race_telemetry.splits));
            
            ESP_LOGI(TAG, ">>> GO! Start Gate beam broken. Race Timer started. <<<");
        }
        
        // 2. INTERMEDIATE SPLIT TRIGGER
        else if (packet->command_id == 3 && current_state == RACING) {
            uint32_t split_ms = (uint32_t)((current_time - race_start_time) / 1000);
            uint8_t index = live_race_telemetry.recorded_splits_count;

            if (index < MAX_INTERMEDIATE_GATES) {
                live_race_telemetry.splits[index].gate_id = packet->gate_id;
                live_race_telemetry.splits[index].split_time_ms = split_ms;
                live_race_telemetry.recorded_splits_count++;
                
                ESP_LOGI(TAG, "Split Recorded | Gate %d: %.3f s", packet->gate_id, (double)split_ms / 1000.0);
            } else {
                ESP_LOGW(TAG, "Max intermediate split array threshold exceeded!");
            }
        }
        
        // 3. FINISH LINE TRIGGER (Fully handled scenario)
        else if (packet->command_id == 2 && current_state == RACING) {
            uint64_t final_us = current_time - race_start_time;
            live_race_telemetry.total_race_time_ms = (uint32_t)(final_us / 1000);
            
            // Assign a unique session ID based on microsecond boot markers
            live_race_telemetry.session_id = (uint32_t)current_time;
            current_state = RACE_FINISHED;
            
            // 1. BACKUP IMMEDIATELY: Save to local flash BEFORE doing any wireless delivery
            save_race_to_flash(&live_race_telemetry);
            

            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "🏁 FINISH LINE CROSSED!                ");
            ESP_LOGI(TAG, "Official Total Time: %.3f seconds       ", (double)live_race_telemetry.total_race_time_ms / 1000.0);
            ESP_LOGI(TAG, "========================================");

            ESP_LOGI(TAG, "Race Complete! Launching %d-packet telemetry burst...", TELEMETRY_BURST_COUNT);

            // 2. TRANSMIT BURST: Broadcast multiple copies to the data collector.
            for (int i = 0; i < TELEMETRY_BURST_COUNT; i++) {
                esp_err_t send_ret = esp_now_send(
                    BROADCAST_MAC,
                    (uint8_t *)&live_race_telemetry,
                    sizeof(telemetry_packet_t));
                if (send_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Telemetry packet %d failed to send: %s",
                             i + 1, esp_err_to_name(send_ret));
                }
                esp_rom_delay_us(5000); // 5ms separation space
            }
            
            // State cleanup: Ready for another run after a short rest period
            vTaskDelay(2000 / portTICK_PERIOD_MS); 
            current_state = CAR_IDLE;
            ESP_LOGI(TAG, "Car reset to IDLE. Ready for next run staging.");
        }
    }
}



void app_main(void) {
    // 1. Initialize NVS Flash (Mandatory first step)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // READ SAVED CONSOLE DATA IMMEDIATELY UPON REBOOT
    load_and_print_flash_backup();

    // 2. Initialize the Network Interfaces
    ESP_ERROR_CHECK(esp_netif_init());
    
    // 3. Create the System Event Loop Task
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 4. Create the Default Station (STA) Network Interface Object
    // --- THIS IS THE CRITICAL MISSING STEP THAT CAUSES THE ESP-NOW INIT PANIC ---
    esp_netif_create_default_wifi_sta();
    
    // 5. Initialize and Start Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // Bypass flash storage degradation
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start()); // Physical radio must be fully awake
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // 6. Safely Initialize ESP-NOW
    // If it still fails, it will gracefully log the string name instead of a cryptic crash
    esp_err_t esp_now_err = esp_now_init();
    if (esp_now_err != ESP_OK) {
        ESP_LOGE("STARTUP", "ESP-NOW Initialization Failed: %s", esp_err_to_name(esp_now_err));
        return;
    } else {
        ESP_LOGI("STARTUP", "ESP-NOW Initialized Successfully!");
    }
    
    // 7. Register standard callback functions
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    const esp_timer_create_args_t staging_timeout_timer_args = {
        .callback = &staging_timeout_callback,
        .name = "staging_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&staging_timeout_timer_args, &staging_timeout_timer));

    // Register the broadcast peer for staging and telemetry packets.
    esp_now_peer_info_t broadcast_peer = { .channel = ESP_NOW_CHANNEL, .ifidx = WIFI_IF_STA, .encrypt = false };
    memcpy(broadcast_peer.peer_addr, BROADCAST_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

    ESP_LOGI("STARTUP", "Peers Logged Successfully!!!");

    // Hardware IO Pins Initialization
    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_conf);

    ESP_LOGI("STARTUP", "Button created!");

    gpio_config_t led_conf= { 
        .pin_bit_mask = MULTI_LED_PIN_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_LOGI("STARTUP", "LED Almost created!");

    gpio_config(&led_conf);

    //init LEDs to off
    gpio_set_level(GREEN_LED_GPIO, 0);
    gpio_set_level(BLUE_LED_GPIO, 0);
    gpio_set_level(RED_LED_GPIO, 0);

    ESP_LOGI("STARTUP", "LEDs created!");

    typedef struct { uint8_t command_id; } button_packet_t;
    button_packet_t stage_packet = { .command_id = 1 };

    ESP_LOGI(TAG, "Vehicle Online. Ready to press button to stage.");
    // Place this inside app_main() right before your main button loop
    xTaskCreate(led_indicator_task, "led_indicator_task", 4096, NULL, 1, NULL);

    while (1) {
        // Handle physical staging request button press
        if (gpio_get_level(BUTTON_GPIO) == 0 && current_state == CAR_IDLE) {
            ESP_LOGI(TAG, "Staging button pressed! Broadcasting request to Start Gate...");
            
            // Broadcast arming signal to trackside gates
            esp_now_send(BROADCAST_MAC, (uint8_t *)&stage_packet, sizeof(stage_packet));
            
            current_state = WAITING_FOR_GPU_BEAM;
            ESP_ERROR_CHECK(esp_timer_start_once(staging_timeout_timer,
                                                 STAGING_TIMEOUT_MS * 1000));
            
            vTaskDelay(500 / portTICK_PERIOD_MS); // Debounce delay
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
