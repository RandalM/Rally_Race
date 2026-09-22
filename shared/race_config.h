/**
 * @file race_config.h
 * @brief Shared wire-format and timing configuration for every ESP32 node.
 *
 * This header is included by the car, every track gate, and the data
 * collector.  The structures below are sent as raw bytes over ESP-NOW, so
 * changing field order or field types requires rebuilding every participant.
 */
#ifndef RACE_CONFIG_H
#define RACE_CONFIG_H

/* Maximum number of split records that fit in one car telemetry packet. */
#define MAX_INTERMEDIATE_GATES 5

/* All nodes must use the same fixed Wi-Fi channel for ESP-NOW broadcasts. */
#define ESP_NOW_CHANNEL        1

/*
 * Normal operation uses separate start and finish hardware.  The optional
 * triple-pass mode reuses Gate 0 for start, split, and finish events.
 */
#define SINGLE_GATE_TRIPLE_PASS 0
#define SINGLE_GATE_FINISH_PASS 3
#define SINGLE_GATE_PASS_LOCKOUT_MS 10000

/*
 * A beam event is declared after this much time without another sensor edge.
 * The value is deliberately short: the sensor produces edges while the beam
 * is clear, and a sustained quiet period means the car has interrupted it.
 */
#define BEAM_BREAK_TIMEOUT_US  15000

/* Broadcast destination used because gates and cars are not paired directly. */
static const uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* One checkpoint measurement relative to the beginning of the current race. */
typedef struct {
    uint8_t gate_id;
    uint32_t split_time_ms;
} split_record_t;

/*
 * Packet sent by a gate.  command_id meanings:
 *   1 = start race
 *   2 = finish race
 *   3 = record an intermediate split
 * sequence_num increases independently on each gate and lets receivers
 * discard duplicate copies from the gate's reliability burst.
 */
typedef struct {
    uint8_t command_id;
    uint8_t gate_id;
    uint32_t sequence_num;
} gate_packet_t;

/*
 * Completed-race packet sent by the car.  The car transmits several copies;
 * session_id identifies the race so the collector prints only one copy.
 */
typedef struct {
    uint32_t session_id;
    uint32_t total_race_time_ms;
    uint8_t recorded_splits_count;
    split_record_t splits[MAX_INTERMEDIATE_GATES];
} telemetry_packet_t;

#endif
