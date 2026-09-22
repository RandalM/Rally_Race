/**
 * @file car.c
 * @brief Race timer and telemetry recorder mounted on the vehicle.
 *
 * The car is the authoritative clock.  It starts its local monotonic timer
 * when a start packet arrives and stops it when a finish packet arrives.
 * Gates only report events; they do not share clocks or communicate with one
 * another.
 */

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
#define STAGING_BURST_COUNT    5   // Duplicate copies of the staging request (broadcast frames are unacknowledged)
#define STAGING_BURST_DELAY_MS 5   // Delay between staging burst frames
#define STAGING_TIMEOUT_MS    30000
#define RACE_FINISHED_HOLD_MS 2000 // How long the finished indication is held before re-arming

#define MULTI_LED_PIN_MASK ((1ULL << GREEN_LED_GPIO) | (1ULL << RED_LED_GPIO) | (1ULL << BLUE_LED_GPIO))

static const char *TAG = "CAR_ESP32";

/* Coarse vehicle lifecycle used by the receive callback and LED task. */
typedef enum { CAR_IDLE, WAITING_FOR_GPU_BEAM, RACING, RACE_FINISHED } CarState;
static volatile CarState current_state = CAR_IDLE;
/* esp_timer_get_time() values are microseconds since boot. */
static uint64_t race_start_time = 0;
static esp_timer_handle_t staging_timeout_timer;
static esp_timer_handle_t finish_hold_timer;

/* Return to idle if the driver stages but never crosses the start gate. */
static void staging_timeout_callback(void *arg) {
    if (current_state == WAITING_FOR_GPU_BEAM) {
        current_state = CAR_IDLE;
        ESP_LOGW(TAG, "Staging timed out after %d seconds; car reset to IDLE.",
                 STAGING_TIMEOUT_MS / 1000);
    }
}

/*
 * Return to idle once the finished indication has been visible for a bit.
 * This runs on the esp_timer task rather than blocking inside the ESP-NOW
 * receive callback, which is shared with every other incoming radio frame.
 */
static void finish_hold_timer_callback(void *arg) {
    current_state = CAR_IDLE;
    ESP_LOGI(TAG, "Car reset to IDLE. Ready for next run staging.");
}

/* Mutable telemetry assembled during the current race. */
static telemetry_packet_t live_race_telemetry;

/*
 * Each gate repeats a packet several times. The gate ID indexes this table,
 * allowing the car to accept the first copy and reject the rest.
 */
static uint32_t last_processed_sequence[100] = {0}; 

/* Local receive type matching gate_packet_t's on-air layout. */
typedef struct {
    uint8_t command_id; 
    uint8_t gate_id;     
    uint32_t sequence_num;  
} inbound_gate_packet_t;


/** Save the most recent completed race so a reboot cannot lose the result. */
void save_race_to_flash(telemetry_packet_t *data) {
    nvs_handle_t my_handle;
    if (nvs_open("race_storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        /* Store the complete fixed-size packet as one NVS blob. */
        nvs_set_blob(my_handle, "last_race", data, sizeof(telemetry_packet_t));
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "💾 Race telemetry securely backed up to NVS Flash memory.");
    }
}

/** Print a previously saved result, if one exists, after boot. */
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

/**
 * Human-facing status LEDs. This task never changes race state; it only
 * reflects the state selected by the packet/button logic.
 */
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

/**
 * Handle gate packets.
 *
 * The callback timestamps receipt immediately. This is important because the
 * start and finish gates have independent clocks and no direct link between
 * them. Only packets valid for the current car state affect the race.
 */
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(inbound_gate_packet_t)) {
        inbound_gate_packet_t *packet = (inbound_gate_packet_t *)data;
        uint64_t current_time = esp_timer_get_time();

        /* Never use an untrusted gate ID as an array index. */
        if (packet->gate_id >= 100) return;

        /* Discard repeated copies and packets from an older gate event. */
        if (packet->sequence_num <= last_processed_sequence[packet->gate_id])return;

        /* Claim the sequence before any state-specific processing. */
        last_processed_sequence[packet->gate_id] = packet->sequence_num;

        /* Start: stop the staging watchdog and establish the car's clock. */
        if (packet->command_id == 1 && current_state == WAITING_FOR_GPU_BEAM) {
            esp_timer_stop(staging_timeout_timer);
            race_start_time = current_time;
            current_state = RACING;
            
            live_race_telemetry.recorded_splits_count = 0; 
            memset(live_race_telemetry.splits, 0, sizeof(live_race_telemetry.splits));
            
            ESP_LOGI(TAG, ">>> GO! Start Gate beam broken. Race Timer started. <<<");
        }
        
        /* Intermediate: append elapsed time without changing race state. */
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
        
        /* Finish: finalize, persist, broadcast, then return to idle. */
        else if (packet->command_id == 2 && current_state == RACING) {
            uint64_t final_us = current_time - race_start_time;
            live_race_telemetry.total_race_time_ms = (uint32_t)(final_us / 1000);
            
            /* The receive timestamp makes the session ID unique after boot. */
            live_race_telemetry.session_id = (uint32_t)current_time;
            current_state = RACE_FINISHED;
            
            /* Persist before radio transmission so the result survives failure. */
            save_race_to_flash(&live_race_telemetry);
            

            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "🏁 FINISH LINE CROSSED!                ");
            ESP_LOGI(TAG, "Official Total Time: %.3f seconds       ", (double)live_race_telemetry.total_race_time_ms / 1000.0);
            ESP_LOGI(TAG, "========================================");

            ESP_LOGI(TAG, "Race Complete! Launching %d-packet telemetry burst...", TELEMETRY_BURST_COUNT);

            /* Repeat the completed result to improve delivery probability. */
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
            
            /*
             * Give the finished indication time to be visible before
             * rearming, without blocking this callback. The ESP-NOW
             * receive path is shared across all incoming frames, so
             * blocking here would leave the car deaf to other radio
             * traffic for the entire hold period.
             */
            ESP_ERROR_CHECK(esp_timer_start_once(finish_hold_timer,
                                                 RACE_FINISHED_HOLD_MS * 1000));
        }
    }
}



void app_main(void) {
    /* NVS is required by the Wi-Fi stack and stores the backup result. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Report any result left behind by a previous interrupted run. */
    load_and_print_flash_backup();

    /* Build the event loop and station interface used by ESP-NOW. */
    ESP_ERROR_CHECK(esp_netif_init());
    
    // 3. Create the System Event Loop Task
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 4. Create the Default Station (STA) Network Interface Object
    // --- THIS IS THE CRITICAL MISSING STEP THAT CAUSES THE ESP-NOW INIT PANIC ---
    esp_netif_create_default_wifi_sta();
    
    /* Bring up the radio and force the shared race channel. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // Bypass flash storage degradation
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start()); // Physical radio must be fully awake
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    /* ESP-NOW carries both staging requests and gate events. */
    // If it still fails, it will gracefully log the string name instead of a cryptic crash
    esp_err_t esp_now_err = esp_now_init();
    if (esp_now_err != ESP_OK) {
        ESP_LOGE("STARTUP", "ESP-NOW Initialization Failed: %s", esp_err_to_name(esp_now_err));
        return;
    } else {
        ESP_LOGI("STARTUP", "ESP-NOW Initialized Successfully!");
    }
    
    /* Receive callbacks run asynchronously from the main loop. */
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    const esp_timer_create_args_t staging_timeout_timer_args = {
        .callback = &staging_timeout_callback,
        .name = "staging_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&staging_timeout_timer_args, &staging_timeout_timer));

    const esp_timer_create_args_t finish_hold_timer_args = {
        .callback = &finish_hold_timer_callback,
        .name = "finish_hold",
    };
    ESP_ERROR_CHECK(esp_timer_create(&finish_hold_timer_args, &finish_hold_timer));

    /* Add the broadcast peer so staging and telemetry can be sent. */
    esp_now_peer_info_t broadcast_peer = { .channel = ESP_NOW_CHANNEL, .ifidx = WIFI_IF_STA, .encrypt = false };
    memcpy(broadcast_peer.peer_addr, BROADCAST_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

    ESP_LOGI("STARTUP", "Peers Logged Successfully!!!");

    /* Configure the physical staging button and status LEDs. */
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

    /* Start in a known visual state before creating the LED task. */
    gpio_set_level(GREEN_LED_GPIO, 0);
    gpio_set_level(BLUE_LED_GPIO, 0);
    gpio_set_level(RED_LED_GPIO, 0);

    ESP_LOGI("STARTUP", "LEDs created!");

    typedef struct { uint8_t command_id; } button_packet_t;
    button_packet_t stage_packet = { .command_id = 1 };

    ESP_LOGI(TAG, "Vehicle Online. Ready to press button to stage.");
    /* LED rendering is independent of the button polling loop. */
    xTaskCreate(led_indicator_task, "led_indicator_task", 4096, NULL, 1, NULL);

    while (1) {
        /* A low button level requests that the start gate arm itself. */
        if (gpio_get_level(BUTTON_GPIO) == 0 && current_state == CAR_IDLE) {
            ESP_LOGI(TAG, "Staging button pressed! Broadcasting request to Start Gate...");

            /*
             * The start gate accepts beam events only after this packet, and
             * broadcast frames are never acknowledged or retried by the
             * radio. Send a burst, same as every other event in this
             * system, so one lost frame can't leave the gate un-armed.
             */
            for (int i = 0; i < STAGING_BURST_COUNT; i++) {
                esp_now_send(BROADCAST_MAC, (uint8_t *)&stage_packet, sizeof(stage_packet));
                esp_rom_delay_us(STAGING_BURST_DELAY_MS * 1000);
            }

            current_state = WAITING_FOR_GPU_BEAM;
            ESP_ERROR_CHECK(esp_timer_start_once(staging_timeout_timer,
                                                 STAGING_TIMEOUT_MS * 1000));
            
            /* Ignore switch bounce and repeated staging requests briefly. */
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
