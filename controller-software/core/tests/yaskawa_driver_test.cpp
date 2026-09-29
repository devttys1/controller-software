// Host test for the yaskawa_encoders FPGA module driver, run by run_host_tests.sh.
//
// Loads the driver against register maps exported from the real FPGA module
// (data/yaskawa_fpga_config.json, built by make_yaskawa_driver_fixtures.py) and the
// older test fpga_config.json, then plays the FPGA's part by writing frames into the
// node variables the DMA copies land in, and checks what the driver publishes.
//
// sources: ../controller/fpga_module_drivers/src/yaskawa_encoders.cpp ../controller/src/fpga_module_driver_factory.cpp ../controller/src/register_helper.cpp ../controller/src/fpga_instructions.cpp ../controller/nodes/src/node_core.cpp ../controller/nodes/src/node_network.cpp ../controller/nodes/src/base_node.cpp ../controller/nodes/src/node_io.cpp

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "yaskawa_encoders.h"

namespace {

const std::string MODULE = "node_4_yaskawa_encoders";
constexpr double COUNTS_PER_REV = yaskawa_incremental_tracker::counts_per_rev;
constexpr uint32_t UPDATE_FREQUENCY = 1000;
using hall_ref_config = yaskawa_hall_reference::config_t;

int failures = 0;

void expect(bool ok, const std::string& what){
    if(!ok){
        std::cout << "  FAIL " << what << std::endl;
        failures++;
    }
}

bool near(double a, double b, double tolerance = 1e-9){
    return std::fabs(a - b) <= tolerance;
}

std::string str(double v){
    std::ostringstream s;
    s.precision(10);
    s << v;
    return s.str();
}

class Harness {
    public:
        json config;
        json user_config;
        Node_Core node_core;
        fpga_instructions instr;
        std::shared_ptr<base_driver> driver;

        bool load(const std::string& config_path, json user = json(), void (*patch)(json&) = nullptr){
            std::ifstream f(config_path);
            if(!f.is_open()){
                std::cout << "  cannot open " << config_path << std::endl;
                return false;
            }
            config = json::parse(f);
            if(patch != nullptr){
                patch(config);
            }
            user_config = user;

            // the node core and driver log every variable they create
            auto* old = std::cout.rdbuf(nullptr);
            driver = Driver_Factory::create_shared("yaskawa_encoders");
            uint32_t ret = driver->load_config(&config, MODULE, &node_core, &instr, user_config.is_null() ? nullptr : &user_config);
            std::cout.rdbuf(old);

            instr.set_update_frequency(UPDATE_FREQUENCY);
            return ret == 0;
        }

        uint32_t starting_bit(const std::string& reg, const std::string& sub){
            return config[MODULE]["node"]["base_group"]["groups"]["encoder"]["registers"][reg]["sub_registers"][sub]["starting_bit"].get<uint32_t>();
        }

        // FPGA register as it appears to the driver after the DMA copy
        uint32_t& reg(int encoder, const std::string& name){
            return *static_cast<uint32_t*>(ptr("node_vars.drivers." + MODULE + ".encoder:" + std::to_string(encoder) + "." + name + ":0"));
        }

        bool has_var(int encoder, const std::string& name){
            void* p = nullptr;
            auto* old = std::cerr.rdbuf(nullptr);
            bool found = node_core.get_global_variable_data_ptr(var_name(encoder, name), &p) == 0;
            std::cerr.rdbuf(old);
            return found;
        }

        template <typename T>
        T var(int encoder, const std::string& name){
            return *static_cast<T*>(ptr(var_name(encoder, name)));
        }

        uint32_t status(bool done, bool crc_fail = false, bool no_response = false, bool unindexed = false){
            return (uint32_t(done) << starting_bit("status", "done")) |
                   (uint32_t(crc_fail) << starting_bit("status", "crc_fail")) |
                   (uint32_t(no_response) << starting_bit("status", "no_response")) |
                   (uint32_t(unindexed) << starting_bit("status", "unindexed"));
        }

        // a good incremental frame as the FPGA would present it
        void incremental_frame(int encoder, uint32_t single_turn, uint8_t rot3, uint8_t hall, uint16_t timestamp, bool unindexed = true){
            reg(encoder, "singleturn_count") = single_turn << 15;
            reg(encoder, "multiturn_count") = 0;
            reg(encoder, "incremental_info") = (uint32_t(hall) << starting_bit("incremental_info", "hall")) |
                                               (uint32_t(rot3) << starting_bit("incremental_info", "rot3"));
            reg(encoder, "timestamp_fast") = timestamp;
            reg(encoder, "status") = status(true, false, false, unindexed);
            if(has_raw_payload(encoder)){
                // as the encoder sends it: bit 0 always 1, bit 6 set until the index is found (count and position 0)
                reg(encoder, "raw_payload_0") = 1u | (uint32_t(unindexed) << 6);
                reg(encoder, "raw_payload_1") = 0;
                reg(encoder, "raw_payload_2") = 0;
            }
        }

        bool has_raw_payload(int encoder){
            void* p = nullptr;
            auto* old = std::cerr.rdbuf(nullptr);
            bool found = node_core.get_global_variable_data_ptr("node_vars.drivers." + MODULE + ".encoder:" + std::to_string(encoder) + ".raw_payload_0:0", &p) == 0;
            std::cerr.rdbuf(old);
            return found;
        }

        void run(){
            driver->run();
        }

    private:
        static std::string var_name(int encoder, const std::string& name){
            return "yaskawa_encoders:" + std::to_string(encoder) + "." + name;
        }

        void* ptr(const std::string& name){
            void* p = nullptr;
            if(node_core.get_global_variable_data_ptr(name, &p) != 0 || p == nullptr){
                throw std::runtime_error("missing node variable " + name);
            }
            return p;
        }
};

struct Frame {
    uint32_t single_turn;
    uint8_t rot3;
    int64_t truth;
};

std::vector<Frame> load_drill_frames(size_t count){
    std::vector<Frame> frames;
    std::ifstream f("data/yaskawa_tracker_sequences.csv");
    std::string line;
    while(std::getline(f, line) && frames.size() < count){
        if(line.rfind("drill_cw,1,", 0) != 0) continue;
        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> v;
        while(std::getline(ss, field, ',')) v.push_back(field);
        frames.push_back({uint32_t(std::stoul(v[2])), uint8_t(std::stoul(v[3])), std::stoll(v[6])});
    }
    return frames;
}

void test_incremental_tracking(){
    Harness h;
    expect(h.load("data/yaskawa_fpga_config.json", {{"velocity_filter_hz", 0}}), "driver loads the new config");

    h.run();    // before the first request the FPGA registers are all zero
    expect(h.var<uint32_t>(1, "frame_error_count") == 0, "no errors counted before the first frame");

    // replies that never pass the CRC count as errors even before the first good frame (a bad cable from power-up)
    {
        Harness bad;
        bad.load("data/yaskawa_fpga_config.json");
        for(int i = 0; i < 7; i++){
            bad.reg(1, "status") = bad.status(true, true);
            bad.run();
        }
        expect(bad.var<uint32_t>(1, "frame_error_count") == 3, "bad replies from the start are counted after the first 4 cycles (" +
               std::to_string(bad.var<uint32_t>(1, "frame_error_count")) + ")");
    }
    expect(!h.var<bool>(1, "frame_valid"), "nothing valid before the first frame");

    auto frames = load_drill_frames(300);
    expect(frames.size() == 300, "loaded drill frames");
    uint16_t timestamp = 60000;
    double origin = 0;
    int position_errors = 0, velocity_errors = 0;
    for(size_t i = 0; i < frames.size(); i++){
        h.incremental_frame(1, frames[i].single_turn, frames[i].rot3, 5, timestamp += 6000);
        h.run();
        double turns = h.var<double>(1, "total_turns");
        if(i == 0){
            origin = turns;
            expect(near(turns, frames[0].single_turn / COUNTS_PER_REV), "first frame: total_turns is the shaft angle");
            continue;
        }
        position_errors += !near((turns - origin) * COUNTS_PER_REV, double(frames[i].truth), 1e-6);
        double expected_rpm = double(frames[i].truth - frames[i - 1].truth) / COUNTS_PER_REV / 0.001 * 60.0;
        // the speed from the encoder's step times needs two frames with a count change: none on the first
        velocity_errors += !near(h.var<double>(1, "velocity_rpm"), i == 1 ? 0.0 : expected_rpm, 1e-6);
    }
    expect(position_errors == 0, "total_turns follows 300 real frames (" + std::to_string(position_errors) + " wrong)");
    expect(velocity_errors == 0, "unfiltered velocity_rpm is the per-frame speed (" + std::to_string(velocity_errors) + " wrong)");
    expect(h.var<bool>(1, "frame_valid"), "frame_valid on good frames");
    expect(h.var<uint32_t>(1, "hall") == 5, "hall published");
    expect(h.var<bool>(1, "unindexed"), "unindexed published");

    const Frame last = frames.back();
    double held = h.var<double>(1, "total_turns");

    // CRC failure: the FPGA keeps the last good values and flags the frame
    h.reg(1, "status") = h.status(true, true);
    h.run();
    expect(!h.var<bool>(1, "frame_valid"), "CRC failure: frame_valid false");
    expect(h.var<uint32_t>(1, "frame_error_count") == 1, "CRC failure counted");
    expect(h.var<double>(1, "total_turns") == held, "CRC failure: position held");

    // no reply at all
    h.reg(1, "status") = h.status(false, true, true);
    h.run();
    expect(h.var<bool>(1, "no_response"), "no_response published");
    expect(h.var<uint32_t>(1, "no_response_count") == 1, "no response counted");
    expect(h.var<uint32_t>(1, "frame_error_count") == 1, "no response is not a frame error");

    // a good status with the previous frame's timestamp is a repeat, not a new frame
    h.incremental_frame(1, last.single_turn, last.rot3, 5, timestamp);
    h.run();
    expect(!h.var<bool>(1, "frame_valid"), "repeated frame is not new");
    expect(h.var<double>(1, "total_turns") == held, "repeated frame: position held");
    expect(h.var<uint32_t>(1, "frame_error_count") == 2, "repeated frame counted as a frame error");

    // next good frame: speed is measured over the 4 cycles since the last good frame
    const uint32_t step = 4000;
    h.incremental_frame(1, (last.single_turn + step) % yaskawa_incremental_tracker::counts_per_rev,
                        last.rot3 + ((last.single_turn + step) >= yaskawa_incremental_tracker::counts_per_rev), 4, timestamp += 24000);
    h.run();
    expect(h.var<bool>(1, "frame_valid"), "recovered after errors");
    expect(near((h.var<double>(1, "total_turns") - held) * COUNTS_PER_REV, step, 1e-6), "position resumes after errors");
    double expected_rpm = step / COUNTS_PER_REV / (4 * 0.001) * 60.0;
    expect(near(h.var<double>(1, "velocity_rpm"), expected_rpm, 1e-6),
           "velocity after missed frames uses the real interval (" + str(h.var<double>(1, "velocity_rpm")) + " vs " + str(expected_rpm) + ")");
}

void test_velocity_filter(){
    Harness h;
    expect(h.load("data/yaskawa_fpga_config.json"), "driver loads with the default filter");

    // constant 1000 rpm: 1000 / 60 rev/s * 1 ms = 2184.53 counts per frame
    const double raw_rpm = 1000.0;
    double position = 0;
    uint16_t timestamp = 60000;
    double alpha = 1.0 - std::exp(-2.0 * M_PI * 100.0 * 0.001);
    double expected = 0;
    bool ok = true;
    for(int i = 0; i < 50; i++){
        uint64_t counts = uint64_t(std::llround(position));
        h.incremental_frame(1, counts % yaskawa_incremental_tracker::counts_per_rev, (counts >> 17) & 7, 0, timestamp += 6000);
        h.run();
        if(i > 0){
            double step_rpm = (std::llround(position) - std::llround(position - raw_rpm / 60.0 * COUNTS_PER_REV * 0.001)) / COUNTS_PER_REV / 0.001 * 60.0;
            if(i == 1){
                step_rpm = 0.0;     // two frames with a count change are needed for a speed
            }
            expected += alpha * (step_rpm - expected);
            ok &= near(h.var<double>(1, "velocity_rpm"), expected, 1e-6);
        }
        position += raw_rpm / 60.0 * COUNTS_PER_REV * 0.001;
    }
    expect(ok, "velocity_rpm is a first order low-pass at 100 Hz by default");
    expect(near(h.var<double>(1, "velocity_rpm"), raw_rpm, 0.5), "filtered velocity settles to 1000 rpm (" + str(h.var<double>(1, "velocity_rpm")) + ")");
}

void test_absolute_port(){
    Harness h;
    expect(h.load("data/yaskawa_fpga_config.json"), "driver loads the new config");
    expect(!h.has_var(0, "velocity_rpm") && !h.has_var(0, "hall"), "absolute port has no incremental variables");

    h.reg(0, "singleturn_count") = 0x80000000;
    h.reg(0, "multiturn_count") = 3;
    h.reg(0, "status") = h.status(true);
    for(int i = 0; i < 4; i++){
        h.run();    // past the startup cycles, whose errors are not counted
    }
    expect(near(h.var<double>(0, "total_turns"), 3.5), "absolute: 3 turns + half a turn");
    expect(h.var<bool>(0, "frame_valid"), "absolute: frame_valid");

    h.reg(0, "singleturn_count") = 0x40000000;
    h.reg(0, "multiturn_count") = 0xFFFF;
    h.run();
    expect(near(h.var<double>(0, "total_turns"), -0.75), "absolute: negative multiturn");

    h.reg(0, "status") = h.status(true, true);
    h.run();
    expect(h.var<uint32_t>(0, "frame_error_count") == 1, "absolute: CRC failure counted");
}

void test_debug_print(){
    // count the lines printed per run, with printing off (default) and every 5 ms at 1 kHz
    auto printed_lines = [](Harness& h, int cycles){
        std::ostringstream captured;
        auto* old = std::cout.rdbuf(captured.rdbuf());
        uint16_t timestamp = 60000;
        for(int i = 0; i < cycles; i++){
            h.incremental_frame(1, 1000 + 100 * i, 0, 3, timestamp += 6000);
            h.run();
        }
        std::cout.rdbuf(old);
        return captured.str();
    };

    Harness off;
    expect(off.load("data/yaskawa_fpga_config.json"), "driver loads");
    expect(printed_lines(off, 12).empty(), "debug print is off by default");

    Harness on;
    expect(on.load("data/yaskawa_fpga_config.json", {{"debug_print_interval_ms", 5}}), "driver loads with debug print");
    std::string out = printed_lines(on, 12);
    size_t lines = std::count(out.begin(), out.end(), '\n');
    expect(lines == 4, "5 ms at 1 kHz prints both encoders on cycles 5 and 10 (" + std::to_string(lines) + " lines)");
    expect(out.find("yaskawa_encoders:1 turns") != std::string::npos && out.find("hall 3") != std::string::npos,
           "debug line shows the incremental encoder: " + out.substr(0, out.find('\n')));
}

void test_hall_reference(){
    // the motor turns slowly through a quarter turn from a random power-up zero, with the default
    // (yaskawa-ffboard) hall edge table; electrical_angle must end up tied to the rotor
    const hall_ref_config table;
    const int32_t elec = yaskawa_hall_reference::counts_per_rev / 4;
    auto hall_at = [&](int64_t rotor){
        int32_t e = int32_t(((rotor % elec) + elec) % elec);
        for(int k = 5; k >= 0; k--){
            if(e >= table.edges[k]) return table.sequence[k];
        }
        return table.sequence[5];
    };

    Harness h;
    expect(h.load("data/yaskawa_fpga_config.json"), "driver loads");
    expect(!h.has_var(0, "electrical_angle") && h.has_var(1, "electrical_angle"), "electrical_angle only on the incremental port");

    const int64_t zero = 1000003;     // encoder counts = rotor + zero
    int64_t rotor = 7 * elec + 2000;  // mid-sector
    uint16_t timestamp = 60000;
    auto frame = [&](){
        int64_t counts = rotor + zero;
        h.incremental_frame(1, uint32_t(counts % yaskawa_incremental_tracker::counts_per_rev), uint8_t((counts >> 17) & 7),
                            hall_at(rotor), timestamp += 6000);
        h.run();
    };

    frame();
    expect(h.var<uint32_t>(1, "hall_reference") == 1, "first frame: sector estimate");
    for(int i = 0; i < elec / 50 + 1000; i++){
        rotor += i < elec / 50 ? 50 : 0;    // a quarter turn at 50 counts per frame, then let the slew settle
        frame();
    }
    double expected = double(rotor % elec) / elec;
    double err = std::fabs(h.var<double>(1, "electrical_angle") - expected);
    expect(h.var<uint32_t>(1, "hall_reference") == 2, "locked after a quarter turn");
    expect(std::min(err, 1.0 - err) * 360.0 < 1.0,
           "electrical_angle tied to the rotor (" + str(h.var<double>(1, "electrical_angle")) + " vs " + str(expected) + ")");
    expect(h.var<uint32_t>(1, "hall_deviation") <= 30, "matching table: small deviation (" + std::to_string(h.var<uint32_t>(1, "hall_deviation")) + ")");

    // the encoder goes silent for 50 ms (a power cycle) and comes back with its zero moved: the old lock is
    // wrong now, so the reference starts again and locks where the rotor really is
    for(int i = 0; i < 50; i++){
        h.reg(1, "status") = h.status(false, true, true);
        h.run();
    }
    const int64_t moved_zero = zero + 12345;
    auto frame_after = [&](){
        int64_t counts = rotor + moved_zero;
        h.incremental_frame(1, uint32_t(counts % yaskawa_incremental_tracker::counts_per_rev), uint8_t((counts >> 17) & 7),
                            hall_at(rotor), timestamp += 6000);
        h.run();
    };
    frame_after();
    expect(h.var<uint32_t>(1, "hall_reference") == 1 && h.var<uint32_t>(1, "hall_reference_restarts") == 1,
           "after 50 ms of silence the hall reference starts again");
    for(int i = 0; i < elec / 50 + 1000; i++){
        rotor += i < elec / 50 ? 50 : 0;
        frame_after();
    }
    expected = double(rotor % elec) / elec;
    err = std::fabs(h.var<double>(1, "electrical_angle") - expected);
    expect(h.var<uint32_t>(1, "hall_reference") == 2 && std::min(err, 1.0 - err) * 360.0 < 1.0,
           "and locks on the rotor again with the new zero (" + str(h.var<double>(1, "electrical_angle")) + " vs " + str(expected) + ")");

    // a single lost frame is not a power cycle
    h.reg(1, "status") = h.status(false, true, true);
    h.run();
    frame_after();
    expect(h.var<uint32_t>(1, "hall_reference") == 2 && h.var<uint32_t>(1, "hall_reference_restarts") == 1, "one lost frame keeps the lock");

    // a table from the user config replaces the default, and a bad one stops the driver loading
    // (index tests are in test_index)
    Harness custom;
    expect(custom.load("data/yaskawa_fpga_config.json", {{"hall_reference", {{"pole_pairs", 2}, {"edges", {0, 10000, 20000, 30000, 40000, 50000}}}}}),
           "driver loads a custom hall table");
    Harness bad;
    auto* old = std::cerr.rdbuf(nullptr);
    bool loaded = bad.load("data/yaskawa_fpga_config.json", {{"hall_reference", {{"edges", {0, 5000, 4000, 16000, 21000, 27000}}}}});
    bool short_loaded = bad.load("data/yaskawa_fpga_config.json", {{"hall_reference", {{"edges", {0, 5000}}}}});
    std::cerr.rdbuf(old);
    expect(!loaded, "edges out of order are rejected");
    expect(!short_loaded, "a short edge table is rejected");

    // debug_print_hall_edges prints the measured table
    Harness printing;
    expect(printing.load("data/yaskawa_fpga_config.json", {{"debug_print_interval_ms", 1000}, {"debug_print_hall_edges", true}}),
           "driver loads with the hall edge print");
    std::ostringstream captured;
    auto* cout_old = std::cout.rdbuf(captured.rdbuf());
    for(int s = 0; s < 8; s++){
        for(int i = 0; i < elec / 50 + 50; i++){
            rotor += s & 1 ? -50 : 50;
            int64_t counts = rotor + zero;
            printing.incremental_frame(1, uint32_t(counts % yaskawa_incremental_tracker::counts_per_rev), uint8_t((counts >> 17) & 7),
                                       hall_at(rotor), timestamp += 6000);
            printing.run();
        }
    }
    std::cout.rdbuf(cout_old);
    std::string out = captured.str();
    expect(out.find("yaskawa_encoders:1 hall edges [0, ") != std::string::npos, "hall edge table printed: " + out.substr(0, 300));
}

void test_index(){
    // a synthetic encoder whose index mark is at rotor position MARK; the payload carries bit 6, index_position
    // and index_count as the UTSIH-B17CK does
    const hall_ref_config table;
    const int32_t elec = yaskawa_hall_reference::counts_per_rev / 4;
    const int32_t cpr = yaskawa_incremental_tracker::counts_per_rev;
    const int64_t MARK = 3 * elec + 10000;      // hall frame: 10000 / 32768 electrical turns past the 2|3 edge
    auto hall_at = [&](int64_t rotor){
        int32_t e = int32_t(((rotor % elec) + elec) % elec);
        for(int k = 5; k >= 0; k--){
            if(e >= table.edges[k]) return table.sequence[k];
        }
        return table.sequence[5];
    };

    for(bool use_index : {false, true}){
        Harness h;
        json user = {{"velocity_filter_hz", 0}};
        if(use_index){
            user["index_electrical_angle"] = 10000.0 / elec;
        }
        expect(h.load("data/yaskawa_fpga_config.json", user), "driver loads");
        const int64_t zero = 777777;
        int64_t rotor = MARK - 20000;
        uint16_t timestamp = 60000;
        bool found = false;
        uint32_t coarse = 0;
        int count = 0;
        auto frame = [&](){
            int64_t counts = rotor + zero;
            uint32_t st = uint32_t(counts % cpr);
            h.incremental_frame(1, st, uint8_t((counts >> 17) & 7), hall_at(rotor), timestamp += 6000);
            h.reg(1, "raw_payload_0") = (found ? 0u : 1u << 6) | 1u;
            h.reg(1, "raw_payload_1") = 0;
            h.reg(1, "raw_payload_2") = (coarse << (80 - 64)) | (uint32_t(count & 0x7F) << (89 - 64));
            h.run();
        };
        auto move = [&](int64_t to, int64_t step){
            while(rotor != to){
                int64_t before = rotor;
                rotor += std::clamp<int64_t>(to - rotor, -step, step);
                auto k_of = [&](int64_t x){ return int64_t(std::floor(double(x - MARK) / cpr)); };
                if(k_of(rotor) != k_of(before)){
                    count += k_of(rotor) > k_of(before) ? 1 : -1;
                    if(!found){
                        found = true;
                        coarse = uint32_t(((MARK + zero) % cpr) >> 8);
                    }
                }
                frame();
            }
        };

        frame();
        expect(!h.var<bool>(1, "indexed") && h.var<bool>(1, "unindexed"), "not indexed at power-up");
        move(MARK + 30000, 40);     // forward over the mark
        move(MARK - 30000, 40);     // and back
        move(MARK + 5000, 40);
        for(int i = 0; i < 1000; i++) frame();     // let any slew finish
        expect(h.var<bool>(1, "indexed") && !h.var<bool>(1, "unindexed"), "indexed after crossing the mark");
        double angle = h.var<double>(1, "index_angle");
        expect(std::fabs(angle - 5000.0 / cpr) * cpr <= 40, "index_angle is the rotor's angle from the mark (" + str(angle * cpr) + " counts)");
        expect(std::fabs(h.var<double>(1, "index_turns") - 5000.0 / cpr) * cpr <= 40, "index_turns");
        double expected_elec = double((rotor % elec + elec) % elec) / elec;
        double err = std::fabs(h.var<double>(1, "electrical_angle") - expected_elec);
        err = std::min(err, 1.0 - err) * 360.0;
        if(use_index){
            expect(h.var<uint32_t>(1, "hall_reference") == 3, "electrical angle from the index");
            expect(err < 0.5, "index-referenced electrical angle within 0.5 electrical degrees (" + str(err) + ")");
        }
        else{
            expect(h.var<uint32_t>(1, "hall_reference") == 2, "without index_electrical_angle the hall edges keep it");
            double measured = h.var<double>(1, "index_electrical_angle_measured");
            expect(std::fabs(measured - 10000.0 / elec) * 360.0 < 3.0,
                   "index_electrical_angle_measured finds the mark's hall-frame angle (" + str(measured) + ")");
        }

        // the encoder powers up again (bit 6 set, count 0): the mark is forgotten until crossed again
        found = false;
        count = 0;
        coarse = 0;
        frame();
        expect(!h.var<bool>(1, "indexed") && h.var<double>(1, "index_turns") == 0.0, "an encoder power-up forgets the mark");
    }
}

void test_monitor_ring(){
    Harness h;
    expect(h.load("data/yaskawa_fpga_config.json", {{"monitor_capacity", 64}}), "driver loads with a monitor ring");
    uint16_t timestamp = 60000;
    for(int i = 0; i < 100; i++){
        h.incremental_frame(1, 5000 + 300 * i, 0, i % 2 ? 3 : 1, timestamp += 6000);
        h.reg(1, "raw_payload_0") = 0x89abcdef;
        h.reg(1, "raw_payload_1") = 0x01234567;
        h.reg(1, "raw_payload_2") = 0xfedcba98;
        h.run();
    }

    // read the newest sample through the schema's offsets, as em_monitor.py does
    const std::string name = "/em_" + MODULE + "_1";
    int fd = shm_open(name.c_str(), O_RDONLY, 0);
    expect(fd >= 0, "ring " + name + " exists");
    if(fd < 0) return;
    struct stat st;
    fstat(fd, &st);
    void* p = mmap(nullptr, size_t(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    const uint8_t* mem = static_cast<const uint8_t*>(p);
    auto u32 = [&](size_t o){ uint32_t v; std::memcpy(&v, mem + o, 4); return v; };
    uint64_t count;
    std::memcpy(&count, mem + 16, 8);
    expect(count == 100 && u32(12) == 64 && u32(24) == UPDATE_FREQUENCY, "100 samples written into 64 slots at 1 kHz");

    json schema = json::parse(reinterpret_cast<const char*>(mem + telemetry_ring::schema_offset));
    expect(schema["mode"] == "17bit_incremental" && schema["pole_pairs"] == 4 && schema["hall_edges"].size() == 6, "schema metadata");
    const uint8_t* sample = mem + telemetry_ring::header_size + ((count - 1) % 64) * u32(8);
    std::map<std::string, double> got;
    for(const auto& f : schema["fields"]){
        size_t o = f["offset"].get<size_t>();
        std::string type = f["type"];
        double v = 0;
        if(type == "d"){ double d; std::memcpy(&d, sample + o, 8); v = d; }
        else if(type == "Q"){ uint64_t q; std::memcpy(&q, sample + o, 8); v = double(q); }
        else if(type == "I"){ uint32_t i; std::memcpy(&i, sample + o, 4); v = i; }
        else if(type == "H"){ uint16_t u; std::memcpy(&u, sample + o, 2); v = u; }
        else if(type == "B"){ v = sample[o]; }
        got[f["name"]] = v;
    }
    munmap(p, size_t(st.st_size));

    expect(got["cycle"] == 100, "cycle counts driver runs");
    expect(got["total_turns"] == h.var<double>(1, "total_turns"), "total_turns matches the node variable");
    expect(got["velocity_rpm"] == h.var<double>(1, "velocity_rpm"), "velocity_rpm matches");
    expect(got["electrical_angle"] == h.var<double>(1, "electrical_angle"), "electrical_angle matches");
    expect(got["single_turn"] == 5000 + 300 * 99, "single_turn is the raw count");
    expect(got["hall"] == h.var<uint32_t>(1, "hall") && got["hall_reference"] == h.var<uint32_t>(1, "hall_reference"), "hall and hall_reference match");
    expect(got["hall_deviation"] == h.var<uint32_t>(1, "hall_deviation"), "hall_deviation matches");
    expect(got["flags"] == (1 | 4), "flags: frame_valid and unindexed");
    expect(got["timestamp_fast"] == uint16_t(60000 + 100 * 6000), "timestamp_fast of the last frame");
    expect(got["raw_payload_0"] == 0x89abcdef && got["raw_payload_1"] == 0x01234567 && got["raw_payload_2"] == 0xfedcba98,
           "raw payload words of the last frame");

    Harness off;
    expect(off.load("data/yaskawa_fpga_config.json", {{"monitor_capacity", 0}}), "driver loads without a monitor ring");

    // FPGA builds from before the raw payload registers still load, and the monitor gets zeros
    Harness older;
    bool loaded = older.load("data/yaskawa_fpga_config.json", {{"monitor_capacity", 16}}, [](json& c){
        auto& regs = c[MODULE]["node"]["base_group"]["groups"]["encoder"]["registers"];
        for(int w = 0; w < 3; w++) regs.erase("raw_payload_" + std::to_string(w));
    });
    expect(loaded, "driver loads an FPGA config without raw payload registers");
    if(loaded){
        older.incremental_frame(1, 1000, 0, 3, 100);
        older.run();
        expect(older.var<bool>(1, "frame_valid"), "and decodes frames");
    }
}

void test_legacy_config(){
    // fpga_config.json from before incremental support: no driver settings, no incremental_info register
    Harness h;
    expect(h.load("../zynq_files/controller/config/fpga_configs/test/fpga_config.json"), "driver loads the legacy config");
    for(int i = 0; i < 6; i++){
        expect(h.has_var(i, "total_turns") && !h.has_var(i, "velocity_rpm"), "legacy config: port " + std::to_string(i) + " is absolute");
    }
    h.reg(5, "singleturn_count") = 0x20000000;
    h.reg(5, "multiturn_count") = 2;
    h.reg(5, "status") = h.status(true);
    h.run();
    expect(near(h.var<double>(5, "total_turns"), 2.125), "legacy config: port 5 converts");
}

}  // namespace

int main(){
    const std::pair<const char*, void (*)()> tests[] = {
        {"incremental tracking and error handling", test_incremental_tracking},
        {"velocity filter", test_velocity_filter},
        {"absolute port", test_absolute_port},
        {"debug print", test_debug_print},
        {"hall reference", test_hall_reference},
        {"index", test_index},
        {"monitor ring", test_monitor_ring},
        {"legacy fpga_config.json", test_legacy_config},
    };
    for(const auto& [name, test] : tests){
        int before = failures;
        try{
            test();
        }
        catch(const std::exception& e){
            std::cout << "  FAIL exception: " << e.what() << std::endl;
            failures++;
        }
        std::cout << (failures == before ? "  PASS " : "  FAIL ") << name << std::endl;
    }
    std::cout << (failures ? std::to_string(failures) + " failed" : "all passed") << std::endl;
    return failures ? 1 : 0;
}
