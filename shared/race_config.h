#ifndef RACE_CONFIG_H
#define RACE_CONFIG_H

#define MAX_INTERMEDIATE_GATES 5  
#define ESP_NOW_CHANNEL        1

#define BEAM_BREAK_TIMEOUT_US  15000  

static const uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
    uint8_t gate_id;
    uint32_t split_time_ms;
} split_record_t;

typedef struct {
    uint8_t command_id;    
    uint8_t gate_id;     
    uint32_t sequence_num; 
} gate_packet_t;

// TELEMETRY PACKET WITH SESSION TRACKING
typedef struct {
    uint32_t session_id;   // Unique ID to filter telemetry burst copies
    uint32_t total_race_time_ms;
    uint8_t recorded_splits_count; 
    split_record_t splits[MAX_INTERMEDIATE_GATES]; 
} telemetry_packet_t;

#endif
