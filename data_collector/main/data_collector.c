#include <stdio.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "race_config.h"



static const char *TAG = "DATA_COLLECTOR";

// Helper function to map a hardware MAC address to a friendly display string
const char* get_car_name(const uint8_t *mac) {
    // Replace these sample MAC arrays with your actual physical ESP32 MAC addresses
    static const uint8_t car1_mac[] = {0x24, 0x0A, 0xC4, 0x01, 0x02, 0x03};
    static const uint8_t car2_mac[] = {0x30, 0xAE, 0xA4, 0x07, 0x08, 0x09};
    static const uint8_t car3_mac[] = {0x24, 0x0A, 0xC4, 0x01, 0x02, 0x03};
    static const uint8_t car4_mac[] = {0x30, 0xAE, 0xA4, 0x07, 0x08, 0x09};
    static const uint8_t car5_mac[] = {0x24, 0x0A, 0xC4, 0x01, 0x02, 0x03};
    static const uint8_t car6_mac[] = {0x30, 0xAE, 0xA4, 0x07, 0x08, 0x09};

    if (memcmp(mac, car1_mac, 6) == 0) return "CAR #1";
    if (memcmp(mac, car2_mac, 6) == 0) return "CAR #2";
    if (memcmp(mac, car3_mac, 6) == 0) return "CAR #3";
    if (memcmp(mac, car4_mac, 6) == 0) return "CAR #4";
    if (memcmp(mac, car5_mac, 6) == 0) return "CAR #5";
    if (memcmp(mac, car6_mac, 6) == 0) return "CAR #6";
    
    return "UNKNOWN VEHICLE";
}

// Tracks the last processed session ID to filter out duplicates in telemetry bursts
static uint32_t last_processed_session_id = 0;


static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(telemetry_packet_t)) {
        telemetry_packet_t *telemetry = (telemetry_packet_t *)data;
        
        // TELEMETRY BURST DEDUPLICATION: Ignore if we already processed this session
        if (telemetry->session_id == last_processed_session_id) {
            return; 
        }
        
        // Update tracking to lock out the remaining burst packets
        last_processed_session_id = telemetry->session_id;
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "🏁 NEW OFFICIAL TELEMETRY RECEIVED      ");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Total Elapsed Time : %.3f seconds", (double)telemetry->total_race_time_ms / 1000.0);
        ESP_LOGI(TAG, "Checkpoints Crossed: %d", telemetry->recorded_splits_count);
        ESP_LOGI(TAG, "----------------------------------------");
        
        for (int i = 0; i < telemetry->recorded_splits_count; i++) {
            double split_secs = (double)telemetry->splits[i].split_time_ms / 1000.0;
            ESP_LOGI(TAG, " Split #%d -> Gate ID %3d | Clock: %.3f s", 
                     i + 1, telemetry->splits[i].gate_id, split_secs);
        }
        ESP_LOGI(TAG, "========================================\n");
    }
}


void app_main(void) {
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nvs_ret);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    ESP_LOGI(TAG, "Data collector ready; listening for telemetry on ESP-NOW channel %d.", ESP_NOW_CHANNEL);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
