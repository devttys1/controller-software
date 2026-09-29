#include "fpga_module_driver_factory.h"
#include "yaskawa_incremental_tracker.h"
#include "yaskawa_hall_reference.h"
#include "yaskawa_index.h"
#include "yaskawa_velocity.h"
#include "telemetry_ring.h"

#include <memory>


#pragma once

// Driver for the yaskawa_encoders FPGA module (controller-firmware/python/src/yaskawa_encoders.py).
//
// Node variables per encoder, named yaskawa_encoders:<index>.<name>:
//   total_turns        DOUBLE  absolute: signed multiturn + fraction, incremental: turns tracked since startup
//                              (the fraction is the shaft angle)
//   frame_valid        BOOL    a new frame with a good CRC arrived this cycle
//   no_response        BOOL    the encoder did not answer this cycle
//   unindexed          BOOL    encoder reports it has not seen its index mark (incremental: payload bit 6, whatever
//                              the FPGA build's status bit says)
//   battery_fail       BOOL
//   frame_error_count  UINT32  replies that failed the CRC or were cut short
//   no_response_count  UINT32
// incremental ports only:
//   velocity_rpm       DOUBLE  low-pass filtered (user driver config "velocity_filter_hz", default 100, 0 disables
//                              the filter). With the raw payload (FPGA builds from 2026-09-26) it is measured between
//                              the times the encoder reports its position changed (yaskawa_velocity.h: several times
//                              quieter below ~10 rpm, the same at speed), over up to "velocity_window_ms" (default
//                              20) or "velocity_window_counts" (default 64); without it, from the position change
//                              over the cycles since the last good frame
//   hall               UINT32  hall sensors U=bit0, V=bit1, W=bit2
//   electrical_angle   DOUBLE  rotor angle in electrical turns [0, 1), zero at the hall 2|3 edge. Unlike
//                              total_turns this is tied to the rotor across encoder power cycles, once
//                              hall_reference is 2 (see yaskawa_hall_reference.h)
//   hall_reference     UINT32  0 no hall reading yet, 1 estimated from the hall sector (within ~33 electrical
//                              degrees), 2 locked on a hall edge (within ~3 electrical degrees), 3 locked on the
//                              encoder's index (see "index_electrical_angle" below)
//   hall_deviation     UINT32  worst disagreement of a later hall edge with the lock, counts: up to ~300 is
//                              normal, thousands means the hall edge table is from another encoder
//   hall_reference_restarts UINT32  times the hall reference started again because the encoder was silent for
//                              20 ms or more (it may have lost power, which moves single_turn's zero)
//   indexed            BOOL    the shaft has passed the encoder's index mark since the encoder powered up
//   index_angle        DOUBLE  shaft angle from the index mark, turns [0, 1): the true shaft angle, the same after
//                              every power-up (once indexed; see yaskawa_index.h)
//   index_turns        DOUBLE  continuous turns from the index mark (the fraction is index_angle)
//   index_electrical_angle_measured DOUBLE  where the index mark sits in the hall frame, electrical turns, measured
//                              while the hall reference is locked on hall edges (-1 until then): the value for
//                              "index_electrical_angle" below
// (the index needs an FPGA build with raw_payload_*; without it indexed stays false)
//
// User driver config for incremental ports (applies to all of them):
//   "hall_reference": {"pole_pairs": 4, "sequence": [3, 1, 5, 4, 6, 2], "edges": [0, 5234, ...]}
//       the motor's pole pairs and the encoder's hall edge table (config_t in yaskawa_hall_reference.h);
//       the default table was measured on the yaskawa-ffboard encoder
//   "index_electrical_angle": 0.332
//       where the index mark sits in the hall frame (electrical turns, from index_electrical_angle_measured).
//       When set, electrical_angle is taken from the index as soon as it is found (hall_reference 3), which
//       places it ~10x more precisely than the hall edges; until then, and when not set, from the hall edges
//
// For bench testing, "debug_print_interval_ms" in the user driver config prints one line per
// encoder to stdout at that interval (default 0, off). With "debug_print_hall_edges": true it also
// prints the hall edge table measured so far, to paste into "hall_reference" once every edge
// has been crossed several times in both directions.
//
// Every cycle each encoder's values also go to a telemetry ring in shared memory,
// /dev/shm/em_<module name>_<index> (see telemetry_ring.h and monitor_sample below), which the
// monitor web server (zynq_files/controller/monitor) reads. "monitor_capacity" in the user driver
// config sets its length in samples (default 16384, 16 s at 1 kHz; 0 turns it off).
class yaskawa_encoders : public base_driver {
public:
    uint32_t custom_load_config(json* user_driver_config) override;

    uint32_t run() override;

private:

    enum class encoder_mode {
        ABSOLUTE,       // "16bit" / "17bit": the FPGA scales singleturn to 32 bits, multiturn is 16 bits
        INCREMENTAL,    // "17bit_incremental": multiturn is only a 4 bit revolution counter, tracked in software
    };

    Register* trigger = nullptr;
    Register* req_packet = nullptr;

    const uint64_t singleturn_res = 1ULL << 32;
    const uint64_t multiturn_res = 1ULL << 16; // only 16 bits for multiturn count
    const uint64_t half_multiturn_count = multiturn_res >> 1;

    double velocity_filter_hz = 100.0;
    double velocity_window_ms = 20.0;
    int32_t velocity_window_counts = 64;
    static constexpr double encoder_silence_s = 0.02;  // restart the hall reference after this long without a good frame

    yaskawa_hall_reference::config_t hall_config;
    double index_electrical_angle = -1.0;   // < 0: electrical_angle from the hall edges only

    uint32_t monitor_capacity = 16384;
    uint64_t run_count = 0;

    uint32_t debug_print_interval_ms = 0;
    bool debug_print_hall_edges = false;
    uint64_t debug_print_cycles = 0;
    void debug_print(uint32_t update_frequency);

    // one telemetry ring sample; the schema in custom_load_config() must match
    struct monitor_sample {
        uint64_t cycle;             // driver cycles since start
        double total_turns;
        double velocity_rpm;
        double electrical_angle;
        uint32_t single_turn;       // raw 17 bit count of the last good frame (incremental ports)
        uint32_t frame_error_count;
        uint32_t no_response_count;
        uint32_t hall_deviation;
        uint8_t hall;
        uint8_t hall_reference;
        uint8_t flags;              // bit 0 frame_valid, 1 no_response, 2 unindexed, 3 battery_fail
        uint8_t reserved0;
        uint16_t timestamp_fast;    // the encoder's frame timestamp (incremental ports), to join captures on
        uint8_t reserved[2];
        uint32_t raw_payload[3];    // payload bits 0:96 of the last good frame, bit 0 in bit 0 of word 0
                                    // (incremental ports, with an FPGA build that has raw_payload_*)
        uint32_t hall_reference_restarts;
        double index_turns;         // continuous turns from the index mark (0 until indexed)
        double index_angle;         // shaft angle from the index mark, turns [0, 1)
        uint8_t indexed;
        uint8_t reserved2[7];
    };
    static_assert(sizeof(monitor_sample) == 96, "monitor_sample layout is read by em_monitor.py");

    void push_monitor_samples(uint32_t update_frequency);

    struct encoder_data{
        encoder_mode mode = encoder_mode::ABSOLUTE;

        uint32_t multiturn_count = 0;
        uint32_t singleturn_count = 0;
        uint16_t timestamp_fast = 0;

        // published as node variables
        double total_turns = 0.0; // calculated from multiturn and singleturn counts
        double velocity_rpm = 0.0;
        bool frame_valid = false;
        bool no_response = false;
        bool unindexed = false;
        bool battery_fail = false;
        uint32_t hall = 0;
        double electrical_angle = 0.0;
        uint32_t hall_reference_state = 0;
        uint32_t hall_deviation = 0;
        uint32_t hall_reference_restarts = 0;
        bool indexed = false;
        double index_angle = 0.0;
        double index_turns = 0.0;
        double index_electrical_angle_measured = -1.0;
        uint32_t frame_error_count = 0;
        uint32_t no_response_count = 0;

        bool turn_wrap = false; // flag if multiturn count is ambiguous

        // incremental tracking
        yaskawa_incremental_tracker tracker;
        yaskawa_hall_reference hall_ref;
        yaskawa_index index;
        yaskawa_velocity velocity;
        uint32_t single_turn = 0;
        std::unique_ptr<telemetry_ring> monitor;
        int64_t last_position = 0;
        uint16_t last_timestamp_fast = 0;
        uint32_t cycles_since_frame = 0;

        Register* multiturn_count_reg = nullptr;
        Register* singleturn_count_reg = nullptr;
        Register* timestamp_fast_reg = nullptr;
        Register* battery_fail_reg = nullptr;
        Register* unindexed_reg = nullptr;
        Register* no_response_reg = nullptr;
        Register* crc_fail_reg = nullptr;
        Register* done_reg = nullptr;
        Register* hall_reg = nullptr;
        Register* rot3_reg = nullptr;
        Register* raw_payload_reg[3] = {nullptr, nullptr, nullptr};
        Register* status_reg = nullptr;
        uint32_t status_word = 0;   // the FPGA status register as copied, for the debug print
        uint32_t raw_payload[3] = {0, 0, 0};
    };

    void convert_turns(encoder_data* data);
    void update_incremental(encoder_data* data, double update_period_s);
    void publish(uint16_t index, const std::string& name, io_type type, void* data);
    std::vector<encoder_data> encoders;

};
