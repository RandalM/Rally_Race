/**
 * @file data_collector.c
 * @brief ESP-NOW receiver that prints completed race results.
 *
 * The collector is intentionally passive: it does not control gates or cars.
 * A car broadcasts its completed telemetry, and this device validates the
 * packet length, removes duplicate burst copies, and prints the result.
 */

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

/**
 * Convert a known car's Wi-Fi MAC address into the human-readable name used
 * in event results. Unknown addresses remain visible as UNKNOWN VEHICLE.
 */
const char* get_car_name(const uint8_t *mac) {
    // Replace these sample MAC arrays with your actual physical ESP32 MAC addresses
    static const uint8_t car1_mac[] = {0xA0, 0xF2, 0x62, 0x4D, 0x34, 0x48};
    static const uint8_t car2_mac[] = {0x20, 0x6E, 0xF1, 0x14, 0xF8, 0x2C};
    static const uint8_t car3_mac[] = {0x20, 0x6E, 0xF1, 0x14, 0xF8, 0x28};
    static const uint8_t car4_mac[] = {0x20, 0x6E, 0xF1, 0x14, 0xF8, 0x10};
    static const uint8_t car5_mac[] = {0x24, 0x0A, 0xC4, 0x01, 0x02, 0x03};
    static const uint8_t car6_mac[] = {0x30, 0xAE, 0xA4, 0x07, 0x08, 0x09};

    if (memcmp(mac, car1_mac, 6) == 0) return "New Volvo";
    if (memcmp(mac, car2_mac, 6) == 0) return "Old Volvo";
    if (memcmp(mac, car3_mac, 6) == 0) return "Scrapter";
    if (memcmp(mac, car4_mac, 6) == 0) return "Fiat";
    if (memcmp(mac, car5_mac, 6) == 0) return "CAR #5";
    if (memcmp(mac, car6_mac, 6) == 0) return "CAR #6";
    
    return "UNKNOWN VEHICLE";
}

/*
 * Every car sends several identical telemetry packets for radio reliability.
 * This value suppresses the remaining copies of the most recently printed
 * race. It is RAM-only and resets when the collector reboots.
 */
static uint32_t last_processed_session_id = 0;

/**
 * ESP-NOW receive callback for completed-race telemetry.
 *
 * ESP-NOW invokes this callback asynchronously. The packet buffer is only
 * valid during the callback, so values are read/printed immediately rather
 * than retaining the pointer for later use.
 */
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (recv_info != NULL && recv_info->src_addr != NULL &&
        len == sizeof(telemetry_packet_t)) {
        telemetry_packet_t *telemetry = (telemetry_packet_t *)data;
        
        /* Ignore duplicate copies from the car's telemetry burst. */
        if (telemetry->session_id == last_processed_session_id) {
            return; 
        }
        
        /* Claim this session before printing so later copies are discarded. */
        last_processed_session_id = telemetry->session_id;
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "🏁 NEW OFFICIAL TELEMETRY RECEIVED      ");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Sending Vehicle     : %s", get_car_name(recv_info->src_addr));
        ESP_LOGI(TAG, "Sender MAC          : %02X:%02X:%02X:%02X:%02X:%02X",
                 recv_info->src_addr[0], recv_info->src_addr[1],
                 recv_info->src_addr[2], recv_info->src_addr[3],
                 recv_info->src_addr[4], recv_info->src_addr[5]);
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
    /* Initialize NVS because the ESP-IDF Wi-Fi stack depends on it. */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nvs_ret);
    }

    /* Create the minimal network/event-loop infrastructure required by Wi-Fi. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* Start a station-mode radio on the same channel used by the gates/cars. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    /* ESP-NOW has no connection handshake for these broadcast packets. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    ESP_LOGI(TAG, "Data collector ready; listening for telemetry on ESP-NOW channel %d.", ESP_NOW_CHANNEL);

    /* Keep app_main alive; all useful work occurs in the receive callback. */
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
