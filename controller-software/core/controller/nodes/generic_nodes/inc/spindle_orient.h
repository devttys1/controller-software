#include "node_factory.h"
#include "spindle_orient_core.h"

#pragma once

/*

Spindle orientation (M19), see spindle_orient_core.h.

Inputs:
    enable          bool    orient while true; false releases the spindle and clears a fault
    angle           double  degrees from the orient reference (config "angle" when not connected)
    mode            uint32  0 the shorter way, 1 forward only, 2 reverse only (config "mode" when not connected)
    indexed         bool    yaskawa_encoders:<n>.indexed
    index_turns     double  yaskawa_encoders:<n>.index_turns
    turns           double  yaskawa_encoders:<n>.total_turns (for the index search limit)
    velocity        double  yaskawa_encoders:<n>.velocity_rpm
Outputs:
    velocity_command double rpm, for the spindle's speed loop (0 while not enabled)
    error           double  degrees to go
    is_oriented     bool
    fault           bool
    state           uint32  0 idle, 1 search, 2 move, 3 settle, 4 oriented, 5 fault

Config (all optional; defaults in spindle_orient_core::config_t):
    {
        "angle": 0.0, "mode": 0,
        "angle_offset_deg": 0.0,        where angle 0 is, degrees past the encoder's index mark
        "max_rpm": 120.0, "accel_rpm_per_s": 600.0, "search_rpm": 30.0,
        "kp_per_s": 20.0, "speed_loop_lag_s": 0.0,
        "tolerance_deg": 0.1, "settle_s": 0.05, "still_rpm": 1.0,
        "timeout_s": 10.0, "search_turns_max": 1.25,
        "cycle_s": 0.001                the network's cycle time
    }

*/

class spindle_orient: public base_node {
    private:
        spindle_orient_core core;

        // what unconnected angle and mode inputs read
        double angle_default = 0.0;
        uint32_t mode_default = 0;

        double velocity_command = 0.0;
        double error = 0.0;
        bool is_oriented = false;
        bool fault = false;
        uint32_t state = 0;

        template <typename T>
        T get(const char* name){
            return *reinterpret_cast<T*>(inputs[name].data_pointer);
        }

    public:
        spindle_orient(){
            inputs.emplace("enable", input(io_type::BOOL, nullptr));
            inputs.emplace("angle", input(io_type::DOUBLE, &angle_default));
            inputs.emplace("mode", input(io_type::UINT32, &mode_default));
            inputs.emplace("indexed", input(io_type::BOOL, nullptr));
            inputs.emplace("index_turns", input(io_type::DOUBLE, nullptr));
            inputs.emplace("turns", input(io_type::DOUBLE, nullptr));
            inputs.emplace("velocity", input(io_type::DOUBLE, nullptr));

            outputs.emplace("velocity_command", output(io_type::DOUBLE, &velocity_command, &execution_number));
            outputs.emplace("error", output(io_type::DOUBLE, &error, &execution_number));
            outputs.emplace("is_oriented", output(io_type::BOOL, &is_oriented, &execution_number));
            outputs.emplace("fault", output(io_type::BOOL, &fault, &execution_number));
            outputs.emplace("state", output(io_type::UINT32, &state, &execution_number));
        }

        uint32_t run() override {
            spindle_orient_core::inputs_t in;
            in.enable = get<bool>("enable");
            in.angle_deg = get<double>("angle");
            in.mode = get<uint32_t>("mode");
            in.indexed = get<bool>("indexed");
            in.index_turns = get<double>("index_turns");
            in.turns = get<double>("turns");
            in.velocity_rpm = get<double>("velocity");

            const auto& out = core.update(in);
            velocity_command = out.velocity_command_rpm;
            error = out.error_deg;
            is_oriented = out.oriented;
            fault = out.fault;
            state = out.state;
            return 0;
        }

        uint32_t configure_settings(json* data) override {
            auto& c = core.cfg;
            try{
                angle_default = data->value("angle", angle_default);
                mode_default = data->value("mode", mode_default);
                c.angle_offset_deg = data->value("angle_offset_deg", c.angle_offset_deg);
                c.search_rpm = data->value("search_rpm", c.search_rpm);
                c.max_rpm = data->value("max_rpm", c.max_rpm);
                c.accel_rpm_per_s = data->value("accel_rpm_per_s", c.accel_rpm_per_s);
                c.kp_per_s = data->value("kp_per_s", c.kp_per_s);
                c.speed_loop_lag_s = data->value("speed_loop_lag_s", c.speed_loop_lag_s);
                c.tolerance_deg = data->value("tolerance_deg", c.tolerance_deg);
                c.settle_s = data->value("settle_s", c.settle_s);
                c.still_rpm = data->value("still_rpm", c.still_rpm);
                c.timeout_s = data->value("timeout_s", c.timeout_s);
                c.search_turns_max = data->value("search_turns_max", c.search_turns_max);
                c.cycle_s = data->value("cycle_s", c.cycle_s);
            }
            catch(const json::exception& e){
                std::cerr << "Error: spindle_orient config: " << e.what() << std::endl;
                return 1;
            }
            if(c.max_rpm <= 0 || c.accel_rpm_per_s <= 0 || c.cycle_s <= 0 || c.tolerance_deg <= 0){
                std::cerr << "Error: spindle_orient: max_rpm, accel_rpm_per_s, cycle_s and tolerance_deg must be positive" << std::endl;
                return 1;
            }
            return 0;
        }
};
