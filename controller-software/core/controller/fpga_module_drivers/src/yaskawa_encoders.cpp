#include "yaskawa_encoders.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdio>
#include <cstring>

static Driver_Registrar<yaskawa_encoders> registrar("yaskawa_encoders");

uint32_t yaskawa_encoders::custom_load_config(json* user_driver_config){

    // no accesses to register or node variable values are allowed here, only in the run function

    uint16_t encoder_count = loader.get_group("encoder", 0)->group_data.count;

    // modes come from the FPGA build, older configs have no driver settings and only absolute encoders
    std::vector<std::string> modes(encoder_count, "17bit");
    const json& node = config->at(module_name).at("node");
    if(node.contains("driver_settings") && node["driver_settings"].contains("encoder_modes")){
        modes = node["driver_settings"]["encoder_modes"].get<std::vector<std::string>>();
        if(modes.size() != encoder_count){
            std::cerr << "Error: yaskawa_encoders has " << encoder_count << " encoders but " << modes.size() << " encoder modes" << std::endl;
            return 1;
        }
    }

    if(user_driver_config != nullptr && user_driver_config->contains("velocity_filter_hz")){
        velocity_filter_hz = user_driver_config->at("velocity_filter_hz").get<double>();
    }
    if(user_driver_config != nullptr && user_driver_config->contains("debug_print_interval_ms")){
        debug_print_interval_ms = user_driver_config->at("debug_print_interval_ms").get<uint32_t>();
    }
    if(user_driver_config != nullptr && user_driver_config->contains("velocity_window_ms")){
        velocity_window_ms = user_driver_config->at("velocity_window_ms").get<double>();
    }
    if(user_driver_config != nullptr && user_driver_config->contains("velocity_window_counts")){
        velocity_window_counts = user_driver_config->at("velocity_window_counts").get<int32_t>();
    }
    if(user_driver_config != nullptr && user_driver_config->contains("index_electrical_angle")){
        index_electrical_angle = user_driver_config->at("index_electrical_angle").get<double>();
    }
    if(user_driver_config != nullptr && user_driver_config->contains("monitor_capacity")){
        monitor_capacity = user_driver_config->at("monitor_capacity").get<uint32_t>();
    }
    if(user_driver_config != nullptr && user_driver_config->contains("debug_print_hall_edges")){
        debug_print_hall_edges = user_driver_config->at("debug_print_hall_edges").get<bool>();
    }
    if(user_driver_config != nullptr && user_driver_config->contains("hall_reference")){
        const json& h = user_driver_config->at("hall_reference");
        try{
            hall_config.pole_pairs = h.value("pole_pairs", hall_config.pole_pairs);
            for(const char* key : {"sequence", "edges"}){
                if(h.contains(key) && h[key].size() != 6){
                    std::cerr << "Error: yaskawa_encoders hall_reference " << key << " needs 6 values" << std::endl;
                    return 1;
                }
            }
            hall_config.sequence = h.value("sequence", hall_config.sequence);
            hall_config.edges = h.value("edges", hall_config.edges);
        }
        catch(const json::exception& e){
            std::cerr << "Error: yaskawa_encoders hall_reference: " << e.what() << std::endl;
            return 1;
        }
    }

    cpy_time_reference(0);
    cpy_write_priority();
    cpy_execution_window(0, 1000);

    req_packet = loader.get_register("request_packet", 0);
    cpy_destination(req_packet);
    cpy_source("request_packet");
    cpy_add_instruction();

    encoders.resize(encoder_count);    // node variables point into this, it must not be resized after this
    for (uint16_t i = 0; i < encoder_count; i++) {
        auto encoder = loader.get_group("encoder", i);
        auto& data = encoders[i];

        if(modes[i] == "17bit_incremental"){
            data.mode = encoder_mode::INCREMENTAL;
            data.velocity.configure(velocity_window_ms / 1000.0, velocity_window_counts);
            if(!data.hall_ref.configure(hall_config)){
                std::cerr << "Error: yaskawa_encoders hall_reference: pole_pairs must divide 131072, sequence must hold 1..6 once each, "
                             "and edges must start at 0 and increase within one electrical revolution" << std::endl;
                return 1;
            }
        }
        else if(modes[i] == "16bit" || modes[i] == "17bit"){
            data.mode = encoder_mode::ABSOLUTE;
        }
        else{
            std::cerr << "Error: unknown yaskawa encoder mode: " << modes[i] << std::endl;
            return 1;
        }

        data.multiturn_count_reg = encoder->get_register("multiturn_count", 0);
        data.singleturn_count_reg = encoder->get_register("singleturn_count", 0);

        std::string encoder_name = "encoder:" + std::to_string(i) + ".";

        cpy_source(data.singleturn_count_reg);
        cpy_destination(encoder_name+"singleturn_count:0");
        cpy_add_instruction();

        cpy_source(data.multiturn_count_reg);
        cpy_destination(encoder_name+"multiturn_count:0");
        cpy_add_instruction();

        // sub-registers must be created before the copy so they share its node variable
        auto status = encoder->get_register("status", 0);
        data.status_reg = status;
        data.battery_fail_reg = status->get_register("battery_fail");
        data.unindexed_reg = status->get_register("unindexed");
        data.no_response_reg = status->get_register("no_response");
        data.crc_fail_reg = status->get_register("crc_fail");
        data.done_reg = status->get_register("done");
        cpy_source(status);
        cpy_destination(encoder_name+"status:0");
        cpy_add_instruction();

        publish(i, "total_turns", io_type::DOUBLE, &data.total_turns);
        publish(i, "frame_valid", io_type::BOOL, &data.frame_valid);
        publish(i, "no_response", io_type::BOOL, &data.no_response);
        publish(i, "unindexed", io_type::BOOL, &data.unindexed);
        publish(i, "battery_fail", io_type::BOOL, &data.battery_fail);
        publish(i, "frame_error_count", io_type::UINT32, &data.frame_error_count);
        publish(i, "no_response_count", io_type::UINT32, &data.no_response_count);

        if(data.mode == encoder_mode::INCREMENTAL){
            // the timestamp is only used to tell a new frame from a repeated one
            data.timestamp_fast_reg = encoder->get_register("timestamp_fast", 0);
            cpy_source(data.timestamp_fast_reg);
            cpy_destination(encoder_name+"timestamp_fast:0");
            cpy_add_instruction();

            auto incremental_info = encoder->get_register("incremental_info", 0);
            data.hall_reg = incremental_info->get_register("hall");
            data.rot3_reg = incremental_info->get_register("rot3");
            cpy_source(incremental_info);
            cpy_destination(encoder_name+"incremental_info:0");
            cpy_add_instruction();

            // the whole payload, for decoding the fields that are not understood yet (FPGA builds from 2026-09-26)
            if(node.at("base_group").at("groups").at("encoder").at("registers").contains("raw_payload_0")){
                for(int w = 0; w < 3; w++){
                    std::string reg_name = "raw_payload_" + std::to_string(w);
                    data.raw_payload_reg[w] = encoder->get_register(reg_name, 0);
                    cpy_source(data.raw_payload_reg[w]);
                    cpy_destination(encoder_name + reg_name + ":0");
                    cpy_add_instruction();
                }
            }

            publish(i, "velocity_rpm", io_type::DOUBLE, &data.velocity_rpm);
            publish(i, "hall", io_type::UINT32, &data.hall);
            publish(i, "electrical_angle", io_type::DOUBLE, &data.electrical_angle);
            publish(i, "hall_reference", io_type::UINT32, &data.hall_reference_state);
            publish(i, "hall_deviation", io_type::UINT32, &data.hall_deviation);
            publish(i, "hall_reference_restarts", io_type::UINT32, &data.hall_reference_restarts);
            publish(i, "indexed", io_type::BOOL, &data.indexed);
            publish(i, "index_angle", io_type::DOUBLE, &data.index_angle);
            publish(i, "index_turns", io_type::DOUBLE, &data.index_turns);
            publish(i, "index_electrical_angle_measured", io_type::DOUBLE, &data.index_electrical_angle_measured);
        }

        if(monitor_capacity > 0){
            json schema = {
                {"module", "yaskawa_encoders"},
                {"driver", module_name},
                {"index", i},
                {"mode", modes[i]},
                {"fields", {
                    {{"name", "cycle"}, {"type", "Q"}, {"offset", offsetof(monitor_sample, cycle)}},
                    {{"name", "total_turns"}, {"type", "d"}, {"offset", offsetof(monitor_sample, total_turns)}},
                    {{"name", "velocity_rpm"}, {"type", "d"}, {"offset", offsetof(monitor_sample, velocity_rpm)}},
                    {{"name", "electrical_angle"}, {"type", "d"}, {"offset", offsetof(monitor_sample, electrical_angle)}},
                    {{"name", "single_turn"}, {"type", "I"}, {"offset", offsetof(monitor_sample, single_turn)}},
                    {{"name", "frame_error_count"}, {"type", "I"}, {"offset", offsetof(monitor_sample, frame_error_count)}},
                    {{"name", "no_response_count"}, {"type", "I"}, {"offset", offsetof(monitor_sample, no_response_count)}},
                    {{"name", "hall_deviation"}, {"type", "I"}, {"offset", offsetof(monitor_sample, hall_deviation)}},
                    {{"name", "hall"}, {"type", "B"}, {"offset", offsetof(monitor_sample, hall)}},
                    {{"name", "hall_reference"}, {"type", "B"}, {"offset", offsetof(monitor_sample, hall_reference)}},
                    {{"name", "flags"}, {"type", "B"}, {"offset", offsetof(monitor_sample, flags)}},
                    {{"name", "timestamp_fast"}, {"type", "H"}, {"offset", offsetof(monitor_sample, timestamp_fast)}},
                    {{"name", "raw_payload_0"}, {"type", "I"}, {"offset", offsetof(monitor_sample, raw_payload) + 0}},
                    {{"name", "raw_payload_1"}, {"type", "I"}, {"offset", offsetof(monitor_sample, raw_payload) + 4}},
                    {{"name", "raw_payload_2"}, {"type", "I"}, {"offset", offsetof(monitor_sample, raw_payload) + 8}},
                    {{"name", "hall_reference_restarts"}, {"type", "I"}, {"offset", offsetof(monitor_sample, hall_reference_restarts)}},
                    {{"name", "index_turns"}, {"type", "d"}, {"offset", offsetof(monitor_sample, index_turns)}},
                    {{"name", "index_angle"}, {"type", "d"}, {"offset", offsetof(monitor_sample, index_angle)}},
                    {{"name", "indexed"}, {"type", "B"}, {"offset", offsetof(monitor_sample, indexed)}},
                }},
                {"flags", {"frame_valid", "no_response", "unindexed", "battery_fail"}},
            };
            if(data.mode == encoder_mode::INCREMENTAL){
                schema["pole_pairs"] = hall_config.pole_pairs;
                schema["hall_sequence"] = hall_config.sequence;
                schema["hall_edges"] = hall_config.edges;
            }
            data.monitor = std::make_unique<telemetry_ring>();
            if(!data.monitor->open("em_" + module_name + "_" + std::to_string(i), sizeof(monitor_sample), monitor_capacity, schema.dump())){
                std::cerr << "Warning: yaskawa_encoders:" << i << " has no monitor ring" << std::endl;
                data.monitor.reset();
            }
        }
    }


    cpy_time_reference(1001);

    trigger = loader.get_register("trigger", 0);
    cpy_destination(trigger);
    cpy_source("trigger");
    cpy_add_instruction();

    return 0;
}

void yaskawa_encoders::publish(uint16_t index, const std::string& name, io_type type, void* data){
    std::string full_name = "yaskawa_encoders:" + std::to_string(index) + "." + name;
    node_core->create_global_variable(full_name, type);
    node_core->set_global_variable_data_ptr(full_name, data);
}

void yaskawa_encoders::convert_turns(encoder_data* data){

    // create double for multiturn + singleturn

    double fraction = double(data->singleturn_count) / double(singleturn_res);

        // Convert unsigned multiturn to signed (wrap-around handling)
    int64_t signed_multiturn;
    if (data->multiturn_count < half_multiturn_count) {
        signed_multiturn = int64_t(data->multiturn_count);
    } else {
        signed_multiturn = int64_t(data->multiturn_count) - int64_t(multiturn_res);
    }

    data->total_turns = double(signed_multiturn) + fraction;

    // Error if we've used the full ±half-range (i.e. ambiguous wrap)
    if(signed_multiturn <= -int64_t(half_multiturn_count) || signed_multiturn >= int64_t(half_multiturn_count)){
        data->turn_wrap = true;
    }

}

void yaskawa_encoders::update_incremental(encoder_data* data, double update_period_s){

    data->cycles_since_frame++;

    // the FPGA keeps the last good values after a bad frame, and a stalled link repeats the same frame
    bool repeated = data->frame_valid && data->tracker.is_primed() && data->timestamp_fast == data->last_timestamp_fast;
    if(repeated){
        // the timestamp advances ~6000 per ms, so a healthy link never repeats one: the FPGA is showing an old frame
        data->frame_error_count++;
    }
    bool new_frame = data->frame_valid && !repeated;
    data->frame_valid = new_frame;
    if(!new_frame){
        return; // hold the last position and velocity
    }

    bool first_frame = !data->tracker.is_primed();
    uint32_t single_turn = data->singleturn_count >> 15;    // FPGA scales the 17 bit count to 32 bits
    data->single_turn = single_turn;
    int64_t position = data->tracker.update(single_turn, data->rot3_reg->get_value<uint8_t>());

    for(int w = 0; w < 3; w++){
        if(data->raw_payload_reg[w] != nullptr){
            data->raw_payload[w] = data->raw_payload_reg[w]->get_value<uint32_t>();
        }
    }
    bool have_raw = data->raw_payload_reg[0] != nullptr;
    double dt = first_frame ? 0.0 : data->cycles_since_frame * update_period_s;
    if(have_raw){
        uint16_t position_age = uint16_t(data->raw_payload[1] >> (40 - 32));  // payload bits 40:56
        data->velocity.update(position, data->timestamp_fast, position_age, dt);
    }

    if(!first_frame && update_period_s > 0.0){
        double raw_rpm = have_raw ? data->velocity.rpm()
                                  : double(position - data->last_position) / yaskawa_incremental_tracker::counts_per_rev / dt * 60.0;
        double alpha = velocity_filter_hz > 0.0 ? 1.0 - std::exp(-2.0 * M_PI * velocity_filter_hz * dt) : 1.0;
        data->velocity_rpm += alpha * (raw_rpm - data->velocity_rpm);
        if(std::fabs(data->velocity_rpm) < 1e-6){
            data->velocity_rpm = 0.0;   // at standstill the filter decays into denormals (slow, and noise in the CSV)
        }
    }

    data->total_turns = double(position) / yaskawa_incremental_tracker::counts_per_rev;
    data->hall = data->hall_reg->get_value<uint8_t>();

    // no good frame for this long: the encoder may have lost power, which moves single_turn's zero, so the
    // hall lock no longer holds. Start the hall reference again (a noisy frame or two is far shorter)
    if(!first_frame && update_period_s > 0.0 && data->cycles_since_frame * update_period_s >= encoder_silence_s){
        data->hall_ref.reset();
        data->hall_reference_restarts++;
        std::cout << "yaskawa_encoders: no good frame for " << data->cycles_since_frame * update_period_s * 1000.0
                  << " ms, hall reference restarted" << std::endl;
    }
    // the index: found once the shaft passes the mark after the encoder powers up (needs the raw payload)
    if(data->raw_payload_reg[0] != nullptr){
        data->index.update(single_turn, position, data->raw_payload);
        data->unindexed = !data->index.is_found();      // payload bit 6, whatever the FPGA build's status bit says
        if(data->index.is_found() && index_electrical_angle >= 0.0){
            data->hall_ref.lock_to_index(data->index.index_single_turn(), index_electrical_angle);
        }
    }

    data->hall_ref.update(single_turn, uint8_t(data->hall));
    data->electrical_angle = data->hall_ref.electrical_angle();

    data->indexed = data->index.is_found();
    data->index_turns = data->index.turns(position);
    data->index_angle = data->index.angle(position);
    if(data->indexed && data->hall_ref.get_state() == yaskawa_hall_reference::LOCKED){
        data->index_electrical_angle_measured = data->hall_ref.index_electrical_angle(data->index.index_single_turn());
    }
    data->hall_reference_state = data->hall_ref.get_state();
    data->hall_deviation = data->hall_ref.deviation();
    data->last_position = position;
    data->last_timestamp_fast = data->timestamp_fast;
    data->cycles_since_frame = 0;
}

uint32_t yaskawa_encoders::run(){

    req_packet->set_value(0xffff);

    uint32_t update_frequency = fpga_instr->get_update_frequency();
    double update_period_s = update_frequency > 0 ? 1.0 / update_frequency : 0.0;

    for(auto& data : encoders) {

        data.multiturn_count = data.multiturn_count_reg->get_value<uint32_t>();
        data.singleturn_count = data.singleturn_count_reg->get_value<uint32_t>();

        bool done = data.done_reg->get_value<bool>();
        bool crc_fail = data.crc_fail_reg->get_value<bool>();
        data.no_response = data.no_response_reg->get_value<bool>();
        data.unindexed = data.unindexed_reg->get_value<bool>();
        data.battery_fail = data.battery_fail_reg->get_value<bool>();

        data.status_word = data.status_reg->get_value<uint32_t>();
        data.frame_valid = done && !crc_fail && !data.no_response;
        if(data.no_response){
            data.no_response_count++;
        }
        else if(!data.frame_valid && run_count >= 4){   // a reply that was not a good frame (the first cycles read the registers' reset values)
            data.frame_error_count++;
        }

        if(data.mode == encoder_mode::INCREMENTAL){
            data.timestamp_fast = data.timestamp_fast_reg->get_value<uint16_t>();
            update_incremental(&data, update_period_s);
        }
        else{
            convert_turns(&data);
        }
    }

    push_monitor_samples(update_frequency);
    debug_print(update_frequency);

    return 0;
}

void yaskawa_encoders::push_monitor_samples(uint32_t update_frequency){
    run_count++;
    for(auto& data : encoders){
        if(!data.monitor){
            continue;
        }
        monitor_sample s = {};
        s.cycle = run_count;
        s.total_turns = data.total_turns;
        s.velocity_rpm = data.velocity_rpm;
        s.electrical_angle = data.electrical_angle;
        s.single_turn = data.single_turn;
        s.timestamp_fast = data.last_timestamp_fast;
        std::memcpy(s.raw_payload, data.raw_payload, sizeof s.raw_payload);
        s.hall_reference_restarts = data.hall_reference_restarts;
        s.index_turns = data.index_turns;
        s.index_angle = data.index_angle;
        s.indexed = data.indexed;
        s.frame_error_count = data.frame_error_count;
        s.no_response_count = data.no_response_count;
        s.hall_deviation = data.hall_deviation;
        s.hall = uint8_t(data.hall);
        s.hall_reference = uint8_t(data.hall_reference_state);
        s.flags = uint8_t(data.frame_valid) | uint8_t(data.no_response) << 1 | uint8_t(data.unindexed) << 2 | uint8_t(data.battery_fail) << 3;
        data.monitor->set_update_frequency(update_frequency);
        data.monitor->push(&s);
    }
}

void yaskawa_encoders::debug_print(uint32_t update_frequency){
    if(debug_print_interval_ms == 0 || update_frequency == 0){
        return;
    }

    uint64_t interval_cycles = std::max<uint64_t>(1, uint64_t(debug_print_interval_ms) * update_frequency / 1000);
    if(++debug_print_cycles % interval_cycles != 0){
        return;
    }

    for(size_t i = 0; i < encoders.size(); i++){
        const auto& data = encoders[i];
        char line[320];
        std::snprintf(line, sizeof line,
                      "yaskawa_encoders:%zu turns %+.6f rpm %+9.2f hall %u elec %.3f hallref %u halldev %u indexed %d index_angle %.4f index_elec %.4f valid %d no_response %d frame_errors %u no_responses %u fpga_status 0x%02x",
                      i, data.total_turns, data.velocity_rpm, data.hall, data.electrical_angle, data.hall_reference_state,
                      data.hall_deviation, data.indexed, data.index_angle, data.index_electrical_angle_measured,
                      data.frame_valid, data.no_response, data.frame_error_count, data.no_response_count, data.status_word);
        std::cout << line << std::endl;

        std::array<int32_t, 6> edges;
        uint32_t crossings = 0;
        if(debug_print_hall_edges && data.mode == encoder_mode::INCREMENTAL){
            if(data.hall_ref.measured_edges(&edges, &crossings)){
                std::snprintf(line, sizeof line,
                              "yaskawa_encoders:%zu hall edges [%d, %d, %d, %d, %d, %d] fewest crossings per direction %u",
                              i, edges[0], edges[1], edges[2], edges[3], edges[4], edges[5], crossings);
            }
            else{
                std::snprintf(line, sizeof line, "yaskawa_encoders:%zu hall edges: turn the shaft through every edge after the lock", i);
            }
            std::cout << line << std::endl;
        }
    }
}
